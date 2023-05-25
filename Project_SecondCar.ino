#include <DCMotor.h>
#include <CoreLED.h>
#include <CoreKEY.h>
#include <CoreBeep.h>
#include <ExtSRAMInterface.h>
#include <LED.h>
#include <BH1750.h>
#include <Command.h>
#include <BEEP.h>
#include <infrare.h>
#include <Ultrasonic.h>
#include <Move.h>
#include <Tg.h>
#include <SYN7318.h>
#include <Chinese.h>
#include "ZigBee.h"




static uint8_t Plate_Data[6];  //主车返回数据
static uint8_t Plate_RecFlag = 0;

uint8_t auto_mark = 0;

#define TSendCycle 200
#define BASEADDRESS 0x6000
#define WRITEADDRESS (BASEADDRESS + 0x0008)
#define READADDRESS (BASEADDRESS + 0x0100)

#define TRACKOFFSET 0x0000
#define CODEOFFSET 0x0002
uint16_t distance0;  //码盘值
uint16_t distance1;  //码盘值
uint16_t distance3;  //码盘值

uint8_t ZigBee_back[16] = { 0x55, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t ZigBee_command[8];  //主车通过ZigBee发过来的接收数据数组
uint8_t ZigBee_judge;       //作为Temp提取校验字节
uint8_t infrare_com[6];     //红外数据数组
uint8_t sendflag;           //决定发送的数据
unsigned long frisrtime;
unsigned long Tcount;
//测试添加
uint8_t ZigBee_Re[10];
uint8_t DZ_K[8] = { 0x55, 0x03, 0x01, 0x01, 0x00, 0x00, 0x02, 0xBB };
uint8_t distance_0[] = { 0x55, 0x02, 0x07, 0x00, 0x00, 0x00, 0x00, 0xBB };  //码盘值清零


uint8_t Data_Type;
uint8_t Data_Flag;
uint8_t Data_Length;
uint8_t Data_OTABuf[40];
uint8_t OpenMv_AGVData_Flag = 1;
unsigned long openmv_Max_num_time = 12000;
unsigned long openmv_Max_En_timer = 0;
void Analyze_Handle(uint8_t com);
void KEY_Handler(uint8_t k_value);

//添加的函数
String DecIntToHexStr(long long num);
int HexToDec(char s);
int CharToNum(uint8_t st);
void Beep_S(int n);
void Road_Task();  //地形标志物任务
void Road_F4ToB4_Test();
void auto_work(void);
void dec_to_hex(int decimal_num, char *hex_num);
void bobo(void);
//添加的变量
uint8_t OpenMV_Light_Level;  //通过OpenMV识别二维码  提取出来的数据的光挡位
uint8_t ZigBee_Data[8];      //ZigBee数据  与ZigBee_command一个意思
uint8_t ZigBee_judge_lys;    //同上
uint8_t ZigBee_Back_lys[16] = { 0x55, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//ZigBee的返回数据
uint8_t ATO_Data[] = { 0x55, 0x02, 0x92, 0x00, 0x00, 0x00, 0x00, 0xBB };  //二维码识别
char qr_data[40];                                                         //二维码原始数据
char test_date[40];                                                       //二维码提取完的数据
uint8_t u = 0;

void setup() {
  CoreLED.Initialization();
  CoreKEY.Initialization();
  CoreBeep.Initialization();
  ExtSRAMInterface.Initialization();
  LED.Initialization();
  BH1750.Initialization();
  BEEP.Initialization();
  Infrare.Initialization();
  Ultrasonic.Initialization();
  DCMotor.Initialization();
  SYN7318.Initialization();

  Serial.begin(115200);
  while (!Serial)
    ;

  sendflag = 0;  //允许副车上传数据标志位
  frisrtime = 0;
  Tcount = 0;
}

unsigned long gt_getsuf(unsigned long a, unsigned long b) {
  unsigned long c = 0;
  if (a > b) {
    c = a - b;
  } else if (a < b) {
    c = b - a;
  } else {
    c = 0;
  }
}

int LED_Num = 1;
void loop() {

  if (LED_Num == 1)  //程序运行一次灯闪一次  用于观察程序有没有卡死
  {                  //但是不加延时的话根本看不出来
    CoreLED.TurnOn(1);
    LED_Num++;
  } else {
    CoreLED.TurnOff(1);
    LED_Num = 1;
  }


  frisrtime = millis();

  //同步校验  同步字节放在主指令低四位

  if (ExtSRAMInterface.ExMem_Read(0x6100)) {
    ExtSRAMInterface.ExMem_Read_Bytes(ZigBee_command, 8);
    ZigBee_judge = ZigBee_command[6];  //获取校验和
    Command.Judgment(ZigBee_command);  //计算校验和
    if ((ZigBee_judge == ZigBee_command[6]) && (ZigBee_command[0] == 0x55) && (ZigBee_command[1] == 0x02) && (ZigBee_command[7] == 0xbb)) {
      //接收到主车发来的数据后提取低四位的同步字节回发
      ZigBee_back[10] = (ZigBee_command[2] & 0x0f);
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
      Move.Syn_Data(ZigBee_back[10]);
      //由于主指令被压缩到了高四位 所以对数据进行了一些处理
      Analyze_Handle(((ZigBee_command[2]) & 0xF0) >> 4);
      ExtSRAMInterface.ExMem_Read_Bytes(ZigBee_command, 8);
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
    }
    if ((ZigBee_command[1] == 0x0f) && (ZigBee_command[4] == 0x07)) {
      Beep_S(2);
    }
  }





  if (((millis() - frisrtime >= TSendCycle) || (Tcount >= TSendCycle)) && (sendflag == 1)) {
    uint16_t tp = (uint16_t)(Ultrasonic.Ranging(CM) * 10.0);  //超声波数据
    ZigBee_back[5] = (tp >> 8) & 0xff;
    ZigBee_back[4] = tp & 0xff;

    tp = BH1750.ReadLightLevel();  //光照度数据
    ZigBee_back[7] = (tp >> 8) & 0xff;
    ZigBee_back[6] = tp & 0xff;

    ZigBee_back[9] = (uint8_t)ExtSRAMInterface.ExMem_Read(0x6003);  //码盘值
    if (ZigBee_back[9] >= 0x80) ZigBee_back[9] = 0xff - ZigBee_back[9];
    ZigBee_back[8] = (uint8_t)ExtSRAMInterface.ExMem_Read(0x6002);
    //将同步字节放在第十位返回
    ZigBee_back[10] = (ZigBee_command[2] & 0x0F);
    ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
    //再发一次  防止数据返回时丢包
    ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
    Tcount = 0x00;
  } else if (sendflag == 1) {
    Tcount += (millis() - frisrtime);
  }



  if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)  //检测OpenMV识别结果
  {
    Data_Type = ExtSRAMInterface.ExMem_Read(0x603A);
    Data_Flag = ExtSRAMInterface.ExMem_Read(0x603B);
    Data_Length = ExtSRAMInterface.ExMem_Read(0x603C);
    Data_Length = Data_Length + 6;
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OTABuf, Data_Length);
    if ((Data_OTABuf[0] == 0x55) && (Data_OTABuf[1] == 0x02) && (Data_OTABuf[2] == 0x92)) {
      ExtSRAMInterface.ExMem_Write_Bytes(0x6180, Data_OTABuf, Data_Length);  //使用自定义数据区上传OpenMV识别结果
      delay(200);
      ExtSRAMInterface.ExMem_Write_Bytes(0x6180, Data_OTABuf, Data_Length);
      delay(200);
      ExtSRAMInterface.ExMem_Write_Bytes(0x6180, Data_OTABuf, Data_Length);
      delay(200);
      ExtSRAMInterface.ExMem_Write_Bytes(0x6180, Data_OTABuf, Data_Length);
      delay(200);
      //OpenMVRx_Handler(Data_OTABuf);                                        //接收OpenMV，数据处理函数
      for (int i = 0; i < (Data_Length - 6); i++) {
        qr_data[i] = Data_OTABuf[i + 5];
      }

      for (int j = 0; j < (Data_Length - 6); j++)  //提取二维码有效数据
      {
        if ((qr_data[j] >= 0x30 && qr_data[j] <= 0x39) || (qr_data[j] >= 0x41 && qr_data[j] <= 0x5A) || (qr_data[j] >= 0x61 && qr_data[j] <= 0x7A)) {
          test_date[u] = qr_data[j];
          u++;
        }
      }

      //      Serial.write(qr_data, Data_Length - 6);
      //      Serial.print(qr_data);
      //      Serial.print("  ");
      //      Serial.write(test_date, u);
      //      Serial.print("  ");
      u = 0;
    }
  }


  CoreKEY.Kwhile(KEY_Handler);
}


void opmv_start_test(void)  //opmv开启识别二维码测试
{
  ATO_Data[3] = 0x01;                                       //开始识别
  Command.Judgment(ATO_Data);                               //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, ATO_Data, 8);  //开启识别地址6008
}

/*
  功    能：关闭OpenMV识别二维码函数
  参    数：无
  返 回 值：无
*/
void Openmv_data_upload(void) {
  ATO_Data[3] = 0x02;          //关闭识别
  Command.Judgment(ATO_Data);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, ATO_Data, 8);
}

/*
  功    能：二维码数提取
  参    数：提取数据过滤干扰剩余数字，数字变十进制除4的余数加一
  返 回 值：数字变十进制除4的余数加一
*/
uint8_t qr_data_int[40];
uint8_t test_dates_int[40];
uint8_t number[2];

int OpenMVQr_Disc_CloseUp(void) {

  if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)  //检测OpenMV识别结果
  {
    Data_Type = ExtSRAMInterface.ExMem_Read(0x603A);
    Data_Flag = ExtSRAMInterface.ExMem_Read(0x603B);
    Data_Length = ExtSRAMInterface.ExMem_Read(0x603C);
    Data_Length = Data_Length + 6;
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OTABuf, Data_Length);
    if ((Data_OTABuf[0] == 0x55) && (Data_OTABuf[1] == 0x02) && (Data_OTABuf[2] == 0x92)) {

      //接收OpenMV，数据处理函数
      for (int i = 0; i < (Data_Length - 6); i++) {
        qr_data_int[i] = Data_OTABuf[i + 5];
      }
      Serial.write(qr_data_int[0]);
      u = 0;
      for (int j = 0; j < (Data_Length - 6); j++)  //提取二维码有效数据
      {
        if ((qr_data_int[j] >= 0x30 && qr_data_int[j] <= 0x39) || (qr_data_int[j] >= 0x41 && qr_data_int[j] <= 0x47)) {
          test_dates_int[u] = qr_data_int[j];
          u++;
        }
      }

      for (int da = 0; da < u; da++) {
        number[da] = CharToNum(test_dates_int[da]);
        Serial.print("s");
        Serial.print(number[da]);
        Serial.print("e");
      }
    }
  }
  return 0;
}


uint8_t hex[6];
int OpenMVQr_Disc_CloseUp2(void) {

  if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)  //检测OpenMV识别结果
  {
    Data_Type = ExtSRAMInterface.ExMem_Read(0x603A);
    Data_Flag = ExtSRAMInterface.ExMem_Read(0x603B);
    Data_Length = ExtSRAMInterface.ExMem_Read(0x603C);
    Data_Length = Data_Length + 6;
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OTABuf, Data_Length);
    if ((Data_OTABuf[0] == 0x55) && (Data_OTABuf[1] == 0x02) && (Data_OTABuf[2] == 0x92)) {

      //接收OpenMV，数据处理函数
      for (int i = 0; i < (Data_Length - 6); i++) {
        qr_data_int[i] = Data_OTABuf[i + 5];
      }

      Serial.print(qr_data_int[0]);
      Serial.print(qr_data_int[1]);
      Serial.print(qr_data_int[2]);
      Serial.print(qr_data_int[3]);
      Serial.print(qr_data_int[4]);
      Serial.print(qr_data_int[5]);


      Serial.print("  8585");
      sprintf(hex[0], "%02X", qr_data_int[0]);
      sprintf(hex[1], "%02X", qr_data_int[1]);
      sprintf(hex[2], "%02X", qr_data_int[2]);
      sprintf(hex[3], "%02X", qr_data_int[3]);
      sprintf(hex[4], "%02X", qr_data_int[4]);
      sprintf(hex[5], "%02X", qr_data_int[5]);
      Serial.print("  ");





      return qr_data_int;
    }
  }
  return 0;
}



void OpenMVQr_Disc_CloseUp3(void) {

  if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)  //检测OpenMV识别结果
  {
    Data_Type = ExtSRAMInterface.ExMem_Read(0x603A);
    Data_Flag = ExtSRAMInterface.ExMem_Read(0x603B);
    Data_Length = ExtSRAMInterface.ExMem_Read(0x603C);
    Data_Length = Data_Length + 6;
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OTABuf, Data_Length);
    if ((Data_OTABuf[0] == 0x55) && (Data_OTABuf[1] == 0x02) && (Data_OTABuf[2] == 0x92)) {

      //接收OpenMV，数据处理函数
      for (int i = 0; i < (Data_Length - 6); i++) {
        qr_data_int[i] = Data_OTABuf[i + 5];
      }
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, Data_OTABuf, Data_Length);
      delay(200);
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, Data_OTABuf, Data_Length);
    }
  }
  return 0;
}

/*
  功    能：开启机器视觉进入交通灯颜色识别
  参    数：无
  返 回 值：无
*/
uint8_t colock_date[] = { 0x55, 0x02, 0x91, 0x00, 0x00, 0x00, 0x00, 0xBB };
void OpenMVQr_Disc_colock(void) {
  colock_date[3] = 0x00;
  Command.Judgment(colock_date);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, colock_date, 8);
}
/*
  功    能：关闭机器视觉进入交通灯颜色识别
  参    数：无
  返 回 值：无
*/

void OpenMVQr_close_colock(void) {
  colock_date[3] = 0x04;
  Command.Judgment(colock_date);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, colock_date, 8);
}
/*
  功    能：开启交通灯标志物进入识别模式
  参    数：key   key=1控制红绿灯A key=2控制红绿灯B
  返 回 值：无
*/
uint8_t colock_dates[] = { 0x55, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBB };
void colock(uint8_t key) {
  if (key == 1) {
    colock_dates[1] = 0x0E;
  }
  if (key == 2) {
    colock_dates[1] = 0x0F;
  }
  key = 0;
  Command.Judgment(colock_dates);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, colock_dates, 8);
}
//红色请求确认
void red_return(uint8_t key) {
  uint8_t red[8] = { 0x55, 0x0F, 0x02, 0x01, 0x00, 0x00, 0x00, 0xBB };
  if (key == 1) {
    red[1] = 0x0E;
  }
  if (key == 2) {
    red[1] = 0x0F;
  }
  key = 0;
  Command.Judgment(red);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, red, 8);
}
//黄色请求确认
void yellow_return(uint8_t key) {
  uint8_t yellow[8] = { 0x55, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0xBB };
  if (key == 1) {
    yellow[1] = 0x0E;
  }
  if (key == 2) {
    yellow[1] = 0x0F;
  }
  key = 0;
  Command.Judgment(yellow);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, yellow, 8);
}
//绿色请求确认
void green_return(uint8_t key) {
  uint8_t green[8] = { 0x55, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0xBB };
  if (key == 1) {
    green[1] = 0x0E;
  }
  if (key == 2) {
    green[1] = 0x0F;
  }
  Command.Judgment(green);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, green, 8);
}
/*
  功    能：颜色判断
  参    数：无
  返 回 值：无
*/
uint8_t data_Color_judgment[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
void Color_judgment(void) {
  ExtSRAMInterface.ExMem_Read_Bytes(0x6038, data_Color_judgment, 5);
  if (data_Color_judgment[0] == 0x55 && data_Color_judgment[2] == 0x91) {
    if (data_Color_judgment[4] == 0x01) {
      Serial.print("red");
      char str[4] = { 0xBA, 0xEC, 0xB5, 0xC6 };  //（GB2312编码）
      SYN7318.VSPTest(str, 0);                   //语音播 红灯
      red_return(2);
    }
    if (data_Color_judgment[4] == 0x02) {
      Serial.print("green");
      char str[4] = { 0xC2, 0xCC, 0xB5, 0xC6 };  //（GB2312编码）
      SYN7318.VSPTest(str, 0);                   //语音播 绿灯
      green_return(2);
    }
    if (data_Color_judgment[4] == 0x03) {
      Serial.print("yellow");
      char str[4] = { 0xBB, 0xC6, 0xB5, 0xC6 };  //（GB2312编码）
      SYN7318.VSPTest(str, 0);                   //语音播 黄灯
      yellow_return(2);
    }
  }
}
/*
  道闸开启
*/
uint8_t open_road_buf[] = { 0x55, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0xBB };
void Road_Gate_Test(void) {
  Command.Judgment(open_road_buf);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, open_road_buf, 8);
}
// 道闸关闭
uint8_t open_road_bufs[] = { 0x55, 0x03, 0x01, 0x02, 0x00, 0x00, 0x00, 0xBB };
void Road_Gate_Tests(void) {
  Command.Judgment(open_road_bufs);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, open_road_bufs, 8);
}
/*
   立体车库
   cheku_data为车库到达的层数
*/
uint8_t cheku[] = { 0x55, 0x0D, 0x01, 0x01, 0x00, 0x00, 0x00, 0xBB };
void cheku_test(uint8_t cheku_data) {
  switch (cheku_data) {
    case 1:
      cheku[3] = 0x01;
      break;
    case 2:
      cheku[3] = 0x02;
      break;
    case 3:
      cheku[3] = 0x03;
      break;
    case 4:
      cheku[3] = 0x04;
      break;
    default:
      break;
  }
  Command.Judgment(cheku);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, cheku, 8);
}
uint8_t servo_buf[8] = { 0x55, 0x02, 0x91, 0x03, 0x00, 0x00, 0x00, 0xBB };  // 给OpenMV发送舵机角度
/*
  功    能：舵机角度控制函数
  参    数：angle: 舵机角度，范围-80至+40，0度垂直于车身
  返 回 值：无
*/
void Servo_Control(int8_t angle) {
  if (angle >= 0) {
    servo_buf[4] = 0x2B;
  } else {
    servo_buf[4] = 0x2D;
  }
  servo_buf[5] = abs(angle);    //开始识别
  Command.Judgment(servo_buf);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, servo_buf, 8);
  delay(1000);
  // Serial.write(servo_buf,8);
}
/*
  语音播报任意内容
*/
uint8_t test_buf[] = { 0xFD, 0x00, 0x06, 0x01, 0x01, 0xC4, 0xFA, 0xBA, 0xC3 };
void Speech_Sounds_Ctr(void) {
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, test_buf, 13);
}

uint8_t trm_buf[] = { 0xAF, 0x06, 0x00, 0x02, 0x00, 0x00, 0x01, 0xBB };
/*
语音识别
*/
void Speech_Disc(void) {
  uint8_t sph_id = 0;
  SYN7318.Start_ASR_send(4);
  delay(200);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, Command.command33, 8);
  sph_id = SYN7318.Start_ASR_rec(true);
  switch (sph_id) {
    Serial.print(SYN7318.Start_ASR_rec(true));
    case 0x02:
      SYN7318.VSPTest(str[0], 1);
      break;
    case 0x03:
      SYN7318.VSPTest(str[1], 1);
      break;
    case 0x04:
      SYN7318.VSPTest(str[2], 1);
      break;
    case 0x05:
      SYN7318.VSPTest(str[3], 1);
      break;
    case 0x06:
      SYN7318.VSPTest(str[4], 1);
      break;
    default:
      SYN7318.VSPTest(str[10], 1);
      break;
  }
  trm_buf[2] = sph_id;
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, trm_buf, 8);
}

uint8_t trackdi_buf[8] = { 0x55, 0x02, 0x91, 0x00, 0x00, 0x00, 0x00, 0xBB };
/*
功    能：OpenMV巡线启动函数
参    数：无
返 回 值：无
*/
void OpenMVTrack_Disc_StartUp(void) {
  trackdi_buf[3] = 0x01;          //开始识别
  Command.Judgment(trackdi_buf);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, trackdi_buf, 8);
}
/*
功    能：关闭OpenMV巡线函数
参    数：无
返 回 值：无
*/
void OpenMVTrack_Disc_CloseUp(void) {
  trackdi_buf[3] = 0x02;          //关闭识别
  Command.Judgment(trackdi_buf);  //计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, trackdi_buf, 8);
}

//请求主车返回数据
void request_return() {
  Plate_RecFlag = 0;
  uint16_t i = 0, k = 0;
  uint8_t request[8] = { 0x55, 0x02, 0x30, 0x00, 0x00, 0x00, 0x30, 0xBB };
  uint8_t request2[8] = { 0x55, 0x02, 0x31, 0x00, 0x00, 0x00, 0x31, 0xBB };
  uint8_t dddd[8];

  unsigned long endTime = millis() + 5000;  //5s内不接收自动退出while循环
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, request, 8);
  while (millis() < endTime) {
    if (ExtSRAMInterface.ExMem_Read(0x6102) == 0x30) {
      Plate_Data[0] = ExtSRAMInterface.ExMem_Read(0x6103);
      // delay(100);
      Plate_Data[1] = ExtSRAMInterface.ExMem_Read(0x6104);
      Plate_Data[2] = ExtSRAMInterface.ExMem_Read(0x6105);
      Plate_RecFlag = Plate_RecFlag + 1;
      break;
    } else {
      delay(200);
      ExtSRAMInterface.ExMem_Write_Bytes(0x6180, request, 8);
    }
  }
  if (Plate_RecFlag == 1) {
    Serial.println("read1");
    Beep_S(2);
  } else {
    Serial.println("noread1");
  }
  delay(1000);
  endTime = millis() + 5000;  //5s内不接收自动退出while循环
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, request2, 8);
  while (millis() < endTime) {
    ExtSRAMInterface.ExMem_Read_Bytes(0x6100, dddd, 8);
    if (dddd[0] == 0x55 && dddd[1] == 0x02 && dddd[2] == 0x31) {
      Plate_Data[3] = ExtSRAMInterface.ExMem_Read(0x6103);
      Plate_Data[4] = ExtSRAMInterface.ExMem_Read(0x6104);
      Plate_Data[5] = ExtSRAMInterface.ExMem_Read(0x6105);
      Plate_RecFlag = Plate_RecFlag + 1;
      break;
    } else {
      delay(200);
      ExtSRAMInterface.ExMem_Write_Bytes(0x6180, request2, 8);
    }
  }
  if (Plate_RecFlag == 2) {
    Serial.println("read2");
    Beep_S(2);
  } else {
    Serial.println("noread2");
  }
}
/**
  功   能:视频寻迹
  参   数: 
  返回值:无
*/

uint8_t Data_OpenMVBuf[8];
char angle = 0;

void OpenMV_Stop(void) {
  DCMotor.Stop();
}

void OpenMV_Go(uint8_t speed, uint16_t dis) {
  DCMotor.Go(speed, dis);
}

void OpenMV_Track(uint8_t Car_Speed) {
  uint32_t num = 0;
  Car_Speed = 50;
  // 清空串口缓存
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 1);
  OpenMVTrack_Disc_StartUp();
  delay(500);
  DCMotor.SpeedCtr(Car_Speed, Car_Speed);
  while (1) {
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)  //检测OpenMV识别结果
    {
      ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 8);
      if ((Data_OpenMVBuf[0] == 0x55) && (Data_OpenMVBuf[1] == 0x02) && (Data_OpenMVBuf[2] == 0x91)) {
        num++;
        Serial.println(num);
        if (Data_OpenMVBuf[4] == 1)  // 路口
        {
          OpenMV_Stop();
          char str[] = { 0xca, 0xae, 0xd7, 0xd6, 0xc2, 0xb7, 0xbf, 0xda };  //（GB2312编码）
          //SYN7318.VSPTest(str, 0);                                          //语音播十字路口
          OpenMVTrack_Disc_CloseUp();
          break;
        } else  // 调整
        {
          // if(Data_OpenMVBuf[3] == 1)
          // {
          //     DCMotor.SpeedCtr(Car_Speed, Car_Speed);
          // }
          if (Data_OpenMVBuf[6] <= 5)  // 车身正无需校准
          {
            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(Car_Speed, Car_Speed);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(Car_Speed, Car_Speed);
            }
          } else if (Data_OpenMVBuf[6] <= 15)  // 车身微偏
          {
            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(20, 50);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(50, 20);
            }
          } else if (Data_OpenMVBuf[6] <= 25) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(10, 50);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(50, 10);
            }
          } else if (Data_OpenMVBuf[6] <= 35) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(5, 50);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(50, 5);
            }
          } else if (Data_OpenMVBuf[6] <= 50) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(0, 40);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(40, 0);
            }
          } else if (Data_OpenMVBuf[6] <= 70) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(-20, 40);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(40, -20);
            }
          } else if (Data_OpenMVBuf[6] > 70) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(-20, 30);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(30, -20);
            }
          }
        }
      }
    }
  }
}

void OpenMV_TrackForDistance(uint8_t Car_Speed, unsigned long targetDistance) {
  uint32_t num = 0;
  Car_Speed = 50;
  // 清空串口缓存
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 1);
  OpenMVTrack_Disc_StartUp();
  delay(500);
  DCMotor.SpeedCtr(Car_Speed, Car_Speed);

  // 获取开始时的码盘距离
  unsigned long initialDistance = distance_get();

  while (distance_get() - initialDistance <= targetDistance) {
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)  //检测OpenMV识别结果
    {
      ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 8);
      if ((Data_OpenMVBuf[0] == 0x55) && (Data_OpenMVBuf[1] == 0x02) && (Data_OpenMVBuf[2] == 0x91)) {
        num++;
        Serial.println(num);
        if (Data_OpenMVBuf[4] == 1)  // 路口
        {
          OpenMV_Stop();
          char str[] = { 0xca, 0xae, 0xd7, 0xd6, 0xc2, 0xb7, 0xbf, 0xda };  //（GB2312编码）
          // SYN7318.VSPTest(str, 0);                                          //语音播十字路口
          OpenMVTrack_Disc_CloseUp();
          break;
        } else  // 调整
        {
          // 如果需要根据结果调整速度，可以在这里添加相应的代码

          // 调整转向
          if (Data_OpenMVBuf[6] <= 5)  // 车身正无需校准
          {
            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(Car_Speed, Car_Speed);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(Car_Speed, Car_Speed);
            }
          } else if (Data_OpenMVBuf[6] <= 15)  // 车身微偏
          {
            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(20, 50);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(50, 20);
            }
          } else if (Data_OpenMVBuf[6] <= 25) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(10, 50);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(50, 10);
            }
          } else if (Data_OpenMVBuf[6] <= 35) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(5, 50);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(50, 5);
            }
          } else if (Data_OpenMVBuf[6] <= 50) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(0, 40);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(40, 0);
            }
          } else if (Data_OpenMVBuf[6] <= 70) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(-20, 40);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(40, -20);
            }
          } else if (Data_OpenMVBuf[6] > 70) {

            if (Data_OpenMVBuf[5] == 43)  // +  向左调
            {
              DCMotor.SpeedCtr(-20, 30);
            } else if (Data_OpenMVBuf[5] == 45)  // -  向右调
            {
              DCMotor.SpeedCtr(30, -20);
            }
          }
        }
      }
    }
  }

  // 寻迹距离到达，停止寻迹
  OpenMV_Stop();
}
#define PULSES_PER_REVOLUTION 1000  // 每个轮子的脉冲数（根据编码器的配置调整）

void rotate90Degrees(bool isLeftTurn, int targetPulses) {
  int initialPulses = distance_get();  // 获取当前码盘值

  // 计算旋转的目标码盘值
  int targetPulsesLeft = initialPulses + targetPulses;
  int targetPulsesRight = initialPulses - targetPulses;

  if (isLeftTurn) {
    DCMotor.SpeedCtr(-50, 50);  // 左转时左轮反向，右轮正向
  } else {
    DCMotor.SpeedCtr(50, -50);  // 右转时左轮正向，右轮反向
  }

  // 持续检查码盘值是否达到目标值
  while (true) {
    int currentPulses = distance_get();  // 获取当前码盘值

    if (isLeftTurn) {
      if (currentPulses >= targetPulsesLeft) {
        break;  // 达到目标码盘值，停止旋转
      }
    } else {
      if (currentPulses <= targetPulsesRight) {
        break;  // 达到目标码盘值，停止旋转
      }
    }
  }

  DCMotor.SpeedCtr(0, 0);  // 停止小车旋转
}


/**
  功   能:获取当前码盘值
  参   数: 
  返回值:无
*/
uint16_t distance_get() {
  uint16_t distance = 0;
  return distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
}
//码盘值清零
void distance_zero() {
  Command.Judgment(distance_0);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, distance_0, 8);
  delay(200);
}
uint8_t gd = 0;
int openmvdata = 0;

/**
  功   能:按键处理函数
  参   数: K_value 按键值
  返回值:无
*/
int tuning_Left = 870;
int tuning_Right = 850;
int mvgo = 400;
void KEY_Handler(uint8_t k_value) {
  switch (k_value) {
    case 1:

      Servo_Control(-65);
      OpenMVTrack_Disc_StartUp();
      delay(500);
      OpenMV_Track(60);
      OpenMV_Go(80, mvgo);
      Move.Turn_Left(tuning_Left);  //左转90°
      delay(500);
      OpenMV_Track(60);
      OpenMV_Go(80, mvgo);
      Move.Turn_Right(tuning_Right);  //右转90°
      delay(500);
      OpenMV_Track(60);
      OpenMV_Go(80, mvgo);
      OpenMV_Track(60);
      OpenMV_Go(80, mvgo);
      OpenMVTrack_Disc_CloseUp();
      Servo_Control(0);


      break;
    case 2:
      OpenMV_TrackForDistance(80, 800);
      // Servo_Control(-65);
      // OpenMVTrack_Disc_StartUp();
      // delay(500);
      // OpenMV_Track(60);
      // OpenMV_Go(60, 550);

      break;
    case 3:
      rotate90Degrees(false,400);


      // OpenMVQr_Disc_colock();  //交通灯识别
      // delay(100);
      // OpenMVQr_Disc_colock();  //交通灯识别
      // delay(100);
      // Color_judgment();  //颜色判断
      // delay(500);
      // Color_judgment();  //颜色判断
      //cheku_test(2);  //车库测试
      //Speech_Sounds_Ctr();
      // const char str[] = { 0xbb, 0xb6, 0xd3, 0xad, 0xb9, 0xe2, 0xc1, 0xd9, 0xb0, 0xd9, 0xbf, 0xc6, 0xc8, 0xd9, 0xb4, 0xb4 };  //（GB2312编码）
      // SYN7318.VSPTest(str, 0);                                                                                                //语音播报随机
      //Road_Gate_Test();  //道闸测试

      //red_return(2);
      // yellow_return(2);
      // delay(100);
      // OpenMVQr_Disc_colock();  //机器视觉进入识别模式
      // delay(100);
      // OpenMVQr_Disc_colock();  //机器视觉进入识别模式
      // delay(1000);
      // Color_judgment();  //颜色判断
      // delay(100);
      // OpenMVQr_Disc_colock();
      // Color_judgment();
      //Servo_Control(15);
      //auto_work();
      //colock_recognition();
      //Servo_Control(0);

      //delay(500);
      //opmv_start_test();  //开启识别二维码
      break;
    case 4:
      OpenMVTrack_Disc_StartUp();
      delay(500);
      OpenMV_Track(60);
      OpenMV_Go(80, mvgo);






      break;


    default:
      break;
  }
}
//心跳
void bobo(void) {
  ZigBee_back[2] = 0x0A;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(100);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(100);
}
/**
  功   能:auto
  参   数:
  返回值:无
*/
uint8_t date[] = { 0xFF, 0x31, 0x00, 0x00, 0x55, 0x00 };
uint8_t date1_s[] = { 0xFF, 0x15, 0x02, 0x00, 0x00, 0x00 };
void auto_work2(void) {
  Beep_S(2);
  ZigBee_back[2] = 0x9;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(500);
  ZigBee_back[2] = 0x9;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(500);
  ZigBee_back[2] = 0x9;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(500);
  cheku_test(1);
  Servo_Control(0);
  bobo();
  delay(100);
  cheku_test(1);
  delay(100);
  Move.X_Start = 6;
  Move.Y_Start = 7;
  Move.Dir_Start = 2;
  Move.AutoPath(6, 6, 2);
  bobo();
  delay(100);
  DCMotor.Line_Go(60, 200);
  delay(1000);
  Move.Turn_Right(500);  //右转45°
  delay(500);
  DCMotor.Back(60, 80);
  bobo();
  delay(500);
  opmv_start_test();  //开启识别二维码
  delay(500);
  OpenMVQr_Disc_CloseUp3();
  delay(500);
  opmv_start_test();  //开启识别二维码
  delay(500);
  OpenMVQr_Disc_CloseUp3();
  delay(500);
  opmv_start_test();  //开启识别二维码
  delay(500);
  OpenMVQr_Disc_CloseUp3();
  delay(500);
  opmv_start_test();  //开启识别二维码
  delay(500);
  delay(500);
  OpenMVQr_Disc_CloseUp3();
  DCMotor.TurnLeft(60);  //左转45°
  bobo();
  delay(500);
  Move.AutoPath(6, 4, 4);
  delay(500);
  DCMotor.Line_Go(60, 1300);
  delay(100);
  Road_Task();
  Move.X_Start = 2;
  Move.Y_Start = 4;
  Move.Dir_Start = 4;
  Move.AutoPath(2, 2, 2);
  bobo();
  date[2] = qr_data_int[0];
  date[3] = qr_data_int[1];
  Infrare.Transmition(date, 6);
  date[2] = qr_data_int[2];
  date[3] = qr_data_int[3];
  delay(500);
  Infrare.Transmition(date, 6);
  date[2] = qr_data_int[4];
  date[3] = qr_data_int[5];
  delay(500);
  Infrare.Transmition(date, 6);
  delay(500);
  Move.AutoPath(2, 2, 3);
  DCMotor.Line_Go(60, 1000);
  bobo();
  delay(200);
  DCMotor.Back(60, 1800);
  ZigBee_back[2] = 0x0D;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  ZigBee_back[2] = 0x0D;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
}


void auto_work3(void) {
  ZigBee_back[2] = 0x9;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(500);
  ZigBee_back[2] = 0x9;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(500);
  ZigBee_back[2] = 0x9;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(500);
  bobo();
  Servo_Control(0);
  Move.X_Start = 6;
  Move.Y_Start = 7;
  Move.Dir_Start = 2;
  Move.AutoPath(6, 6, 2);
  bobo();
  delay(1000);
  DCMotor.Line_Go(60, 200);
  delay(1000);
  Move.Turn_Right(500);  //右转45°
  delay(500);
  DCMotor.Back(60, 80);
  opmv_start_test();  //开启识别二维码
  delay(500);
  bobo();
  opmv_start_test();  //开启识别二维码

  delay(500);
  OpenMVQr_Disc_CloseUp();
  delay(500);
  OpenMVQr_Disc_CloseUp();
  delay(500);
  OpenMVQr_Disc_CloseUp();
  delay(500);
  bobo();
  DCMotor.TurnLeft(60);  //左转45°
  delay(500);
  Move.AutoPath(6, 4, 2);
  bobo();
  delay(500);
  delay(500);
  DCMotor.TurnLeft(50, 40);
  delay(200);
  DCMotor.Stop();
  delay(500);
  colock(2);  //进入识别模式
  bobo();
  delay(200);
  yellow_return(2);  //黄灯
  Servo_Control(0);
  delay(200);
  Move.AutoPath(6, 2, 2);
  delay(500);
  Move.Turn_Left(437);  //左转45°
  delay(200);
  Infrare.Transmition(date1_s, 6);  //发送红外数据
  bobo();
  delay(200);
  Infrare.Transmition(date1_s, 6);
  delay(800);
  Move.Turn_Left(637);  //左转45°
  delay(200);


  DCMotor.TurnLeft(80);
  delay(500);
  DCMotor.Line_Go(60, 1000);
  delay(500);
  DCMotor.Back(60, 1600);
  bobo();
  delay(200);
  ZigBee_back[2] = 0x0D;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(200);
  ZigBee_back[2] = 0x0D;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(200);
  ZigBee_back[2] = 0x0D;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
}



void auto_work(void) {

  //request_return();
  Beep_S(2);
  ZigBee_back[2] = 0x9;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(500);
  ZigBee_back[2] = 0x9;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(500);
  ZigBee_back[2] = 0x9;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
  delay(500);
  cheku_test(1);
  Servo_Control(0);
  delay(100);
  cheku_test(1);
  delay(100);
  Move.X_Start = 6;
  Move.Y_Start = 7;
  Move.Dir_Start = 2;
  Move.AutoPath(6, 6, 2);
  delay(1000);
  Move.Turn_Right(450);  //右转45°
  opmv_start_test();     //开启识别二维码
  delay(500);
  DCMotor.Back(60, 80);
  delay(500);
  opmv_start_test();  //开启识别二维码
  delay(500);
  opmv_start_test();  //开启识别二维码
  delay(500);
  Move.Turn_Left(437);  //左转45°
  delay(500);
  Move.AutoPath(6, 4, 2);
  delay(500);
  DCMotor.TurnLeft(50, 40);
  delay(200);
  DCMotor.Stop();
  delay(500);
  colock(2);
  OpenMVQr_Disc_colock();  //交通灯识别
  delay(100);
  OpenMVQr_Disc_colock();  //交通灯识别
  delay(100);
  OpenMVQr_Disc_colock();  //交通灯识别
  delay(100);
  OpenMVQr_Disc_colock();  //交通灯识别
  delay(100);
  Color_judgment();  //颜色判断
  delay(500);
  Color_judgment();  //颜色判断
  delay(100);
  Servo_Control(0);
  Move.AutoPath(4, 2, 4);
  delay(100);
  Move.Turn_Right(450);  //右转45°
  delay(1000);
  DCMotor.TurnLeft(80);
  delay(500);
  DCMotor.Line_Go(60, 1600);
  delay(500);
  DCMotor.Back(60, 100);

  ZigBee_back[2] = 0x0D;  //完成回传指令接收
  ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
}
/**
  功   能:从车任务处理函数
  参   数:com 主指令
  返回值:无
*/

//20190328 lys 重写协议后的数据处理函数
void Analyze_Handle(uint8_t com) {

  switch (com) {
    case 0x08:  //从车数据上传控制
      if (ZigBee_command[3] == 0x01) {
        ZigBee_back[2] = 0x11;  //完成回传指令接收
        ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
        sendflag = 0x01;  //开启从车数据上传
        OpenMv_AGVData_Flag = 1;
        BEEP.TurnOn();
        delay(1000);
        BEEP.TurnOff();
      } else {
        sendflag = 0x00;  //关闭从车数据上传
      }
      break;
    case 0x01:                                 //调光挡位
      Tg.Diming(HexToDec(ZigBee_command[3]));  //调光
      ZigBee_back[2] = 0x05;
      break;
    case 0x02:  //前进
      DCMotor.Go(ZigBee_command[3], (ZigBee_command[4] + (ZigBee_command[5] << 8)));
      ZigBee_back[2] = 0x03;
      break;
    case 0x0A:  //后退
      DCMotor.Back(ZigBee_command[3], (ZigBee_command[4] + (ZigBee_command[5] << 8)));
      ZigBee_back[2] = 0x08;
      break;
    case 0x0B:  //左转45°
      Move.Turn_Left(437);
      ZigBee_back[2] = 0x02;
      break;
    case 0x0C:  //右转45°
      Move.Turn_Right(450);
      ZigBee_back[2] = 0x02;
      break;
    case 0x03:  //给起始坐标赋值
      Move.X_Start = HexToDec(ZigBee_command[3]);
      Move.Y_Start = HexToDec(ZigBee_command[4]);
      Move.Dir_Start = ZigBee_command[5] & 0x0f;
      Move.Freely_Flag = ZigBee_command[5] & 0xf0;
      ZigBee_back[2] = 0x88;
      break;
    case 0x04:  //给目的坐标赋值
      ZigBee_back[2] = 0x00;
      Move.AutoPath(ZigBee_command[3], ZigBee_command[4], ZigBee_command[5]);  //自动路径
      ZigBee_back[2] = 0x0f;
      break;
    case 0x05:  //OpenMv二维码识别
      if (ZigBee_command[3] == 0x01) {
        Beep_S(2);
        ATO_Data[3] = 0x01;          //开始识别
        OpenMv_AGVData_Flag = 0;     //关闭上传从车数据
        Command.Judgment(ATO_Data);  //计算校验和
        ExtSRAMInterface.ExMem_Write_Bytes(0x6008, ATO_Data, 8);
        delay(200);
        openmv_Max_En_timer = millis() + openmv_Max_num_time;
      } else if (ZigBee_command[3] == 0x02) {
        ATO_Data[3] = 0x02;  //停止识别
        CoreLED.TurnOnOff(4);
        Command.Judgment(ATO_Data);  //计算校验和
        ExtSRAMInterface.ExMem_Write_Bytes(0x6008, ATO_Data, 8);
      }
      break;
    case 0x06:  //循迹
      ZigBee_back[2] = 0x00;
      DCMotor.CarTrack(ZigBee_command[3]);
      ZigBee_back[2] = 0x01;
      break;
    case 0x07:                                                   //码盘清零
      Command.Judgment(Command.command01);                       //计算校验和
      ExtSRAMInterface.ExMem_Write_Bytes(Command.command01, 8);  //码盘清零
      break;
    case 0x0D:  //保存红外数据
      infrare_com[0] = ZigBee_command[3];
      infrare_com[1] = ZigBee_command[4];
      infrare_com[2] = ZigBee_command[5];
      break;
    case 0x0E:  //保存红外数据
      infrare_com[3] = ZigBee_command[3];
      infrare_com[4] = ZigBee_command[4];
      infrare_com[5] = ZigBee_command[5];
      break;
    case 0x0F:  //发送红外数据
      Infrare.Transmition(infrare_com, 6);
      break;
    case 0x09:  //保留的任务值 可以写死

      ZigBee_back[2] = 0x9;  //完成回传指令接收
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
      delay(500);
      ZigBee_back[2] = 0x9;  //完成回传指令接收
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
      delay(500);
      ZigBee_back[2] = 0x9;  //完成回传指令接收
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
      delay(500);
      bobo();
      Servo_Control(0);
      Move.X_Start = 6;
      Move.Y_Start = 7;
      Move.Dir_Start = 2;
      Move.AutoPath(6, 6, 2);
      bobo();
      delay(1000);
      DCMotor.Line_Go(60, 200);
      delay(1000);
      Move.Turn_Right(500);  //右转45°
      delay(500);
      DCMotor.Back(60, 80);
      opmv_start_test();  //开启识别二维码
      delay(500);
      bobo();
      opmv_start_test();  //开启识别二维码

      delay(500);
      OpenMVQr_Disc_CloseUp();
      delay(500);
      OpenMVQr_Disc_CloseUp();
      delay(500);
      OpenMVQr_Disc_CloseUp();
      delay(500);
      bobo();
      DCMotor.TurnLeft(60);  //左转45°
      delay(500);
      Move.AutoPath(6, 4, 2);
      bobo();
      delay(500);
      delay(500);
      DCMotor.TurnLeft(50, 40);
      delay(200);
      DCMotor.Stop();
      delay(500);
      colock(2);  //进入识别模式
      bobo();
      OpenMVQr_Disc_colock();  //交通灯识别
      delay(100);
      OpenMVQr_Disc_colock();  //交通灯识别
      delay(100);
      Color_judgment();  //颜色判断
      delay(500);
      Color_judgment();  //颜色判断
      delay(200);
      yellow_return(2);  //黄灯
      Servo_Control(0);
      delay(200);
      Move.AutoPath(6, 2, 2);
      delay(500);
      Move.Turn_Left(437);  //左转45°
      delay(200);
      Infrare.Transmition(date1_s, 6);  //发送红外数据
      bobo();
      delay(200);
      Infrare.Transmition(date1_s, 6);
      delay(800);
      Move.Turn_Left(637);  //左转45°
      delay(200);


      DCMotor.TurnLeft(80);
      delay(500);
      DCMotor.Line_Go(60, 1000);
      delay(500);
      DCMotor.Back(60, 1600);
      bobo();
      delay(200);
      ZigBee_back[2] = 0x0D;  //完成回传指令接收
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
      delay(200);
      ZigBee_back[2] = 0x0D;  //完成回传指令接收
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);
      delay(200);
      ZigBee_back[2] = 0x0D;  //完成回传指令接收
      ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_back, 16);

      break;


    default:
      break;
  }
}

//原有的数据通讯协议
//void Analyze_Handle(uint8_t com)
//{
//  switch (com)
//  {
//  case 0x01:                                       //停止
////    DCMotor.Stop();
////   ZigBee_back[2] = 0x00;
////
////    break;
//
//     if (ZigBee_command[3] == 0x01)
//     {
//       sendflag = 0x01;                             //上传从车数据
//       OpenMv_AGVData_Flag = 1;
//     }
//     else{
//       sendflag = 0x00;                              //关闭上从车数据
//     }
//     break;
//  case 0x02:                                        //前进
//    DCMotor.Go(ZigBee_command[3], (ZigBee_command[4] + (ZigBee_command[5] << 8)));
//    ZigBee_back[2] = 0x03;
//    break;
//  case 0x03:                                        //后退
//    DCMotor.Back(ZigBee_command[3], (ZigBee_command[4] + (ZigBee_command[5] << 8)));
//    ZigBee_back[2] = 0x03;
//    break;
//  case 0x04:                                        //左转
//    DCMotor.TurnLeft(ZigBee_command[3]);
//    ZigBee_back[2] = 0x02;
//    break;
//  case 0x05:                                        //右转
//    DCMotor.TurnRight(ZigBee_command[3]);
//    ZigBee_back[2] = 0x02;
//    break;
//  case 0x06:                                        //循迹
//    DCMotor.CarTrack(ZigBee_command[3]);
//    ZigBee_back[2] = 0x01;
//    break;
//  case 0x07:                                        //码盘清零
//    Command.Judgment(Command.command01);                        //计算校验和
//    ExtSRAMInterface.ExMem_Write_Bytes(Command.command01, 8);   //码盘清零
//    break;
//  case 0x10:                                       //保存红外数据
//    infrare_com[0] = ZigBee_command[3];
//    infrare_com[1] = ZigBee_command[4];
//    infrare_com[2] = ZigBee_command[5];
//    break;
//  case 0x11:                                       //保存红外数据
//    infrare_com[3] = ZigBee_command[3];
//    infrare_com[4] = ZigBee_command[4];
//    infrare_com[5] = ZigBee_command[5];
//    break;
//  case 0x12:                                        //发送红外数据
//    Infrare.Transmition(infrare_com, 6);
//    break;
//  case 0x20:                                        //左右转向灯
//    if (ZigBee_command[3] == 0x01)  LED.LeftTurnOn();
//    else              LED.LeftTurnOff();
//    if (ZigBee_command[4] == 0x01)  LED.RightTurnOn();
//    else              LED.RightTurnOff();
//    break;
//  case 0x30:                                        //蜂鸣器
//    if (ZigBee_command[3] == 0x01)  BEEP.TurnOn();
//    else              BEEP.TurnOff();
//    break;
//  case 0x40:                                        //保留
//    break;
//  case 0x50:                                        //LCD图片上翻页
//    Command.Judgment(Command.command13);
//    ExtSRAMInterface.ExMem_Write_Bytes(Command.command13, 8);
//    break;
//  case 0x51:                                        //LCD图片下翻页
//    Command.Judgment(Command.command14);
//    ExtSRAMInterface.ExMem_Write_Bytes(Command.command14, 8);
//    break;
//  case 0x61:                                        //光源加一档
//    Infrare.Transmition(Command.HW_Dimming1, 4);
//    break;
//  case 0x62:                                        //光源加二挡
//    Infrare.Transmition(Command.HW_Dimming2, 4);
//    break;
//  case 0x63:                                        //光源加三挡
//    Infrare.Transmition(Command.HW_Dimming3, 4);
//    break;
//  case 0x80:                                        //从车返回数据切换
//    if (ZigBee_command[3] == 0x01)
//    {
//       sendflag = 0x01;                             //上传从车数据
//       OpenMv_AGVData_Flag = 1;
//    }
//    else{
//       sendflag = 0x00;                              //关闭上从车数据
//    }
//    break;
//  case 0x87:      //给起始坐标赋值
//         Move.X_Start = HexToDec(ZigBee_command[3]);
//         Move.Y_Start = HexToDec(ZigBee_command[4]);
//         Move.Dir_Start = HexToDec(ZigBee_command[5]);
//         break;
//
//  case 0x88:     //给目的坐标赋值
//         Move.AutoPath(ZigBee_command[3],ZigBee_command[4],ZigBee_command[5]);
//        //自动路径
//        break;
//  case 0x89:
//        Tg.Diming(HexToDec(ZigBee_command[3]));//调光
//        break;
//  case 0x92:                                        //OpenMv二维码识别
//    if(ZigBee_command[3] == 0x01)
//    {
//        ATO_Data[3] = 0x01;                         //开始识别
//        OpenMv_AGVData_Flag = 0;                    //关闭上传从车数据
//        Command.Judgment(ATO_Data);                 //计算校验和
//        ExtSRAMInterface.ExMem_Write_Bytes(0x6008, ATO_Data, 8);
//        delay(200);
//        openmv_Max_En_timer = millis()+openmv_Max_num_time;
//      } else if(ZigBee_command[3] == 0x02){
//           ATO_Data[3] = 0x02;                      //停止识别
//           CoreLED.TurnOnOff(4);
//           Command.Judgment(ATO_Data);              //计算校验和
//           ExtSRAMInterface.ExMem_Write_Bytes(0x6008, ATO_Data, 8);
//        }
//    break;
//
//    case 0x99:
//        ZigBee_back[2] = 0x99;
//    break;
//  default:
//    break;
//  }
//}

//十六进制转十进制
int HexToDec(char s) {
  return s - 0x30 + 48;
  //  int t;
  //  long sum = 0;
  //  if(s<'9'){
  //    t = s-'0';
  //  }
  //  else{
  //    t = s-'a'+10;
  //  }
  //  sum = sum*16+t;
  //  return sum+48;
}
//字符转数字
int CharToNum(uint8_t st) {
  if (st == '1')
    return 1;
  if (st == '2')
    return 2;
  if (st == '3')
    return 3;
  if (st == '4')
    return 4;
  if (st == '5')
    return 5;
  if (st == '6')
    return 6;
  if (st == '7')
    return 7;
  if (st == '8')
    return 8;
  if (st == '9')
    return 9;
  if (st == '0')
    return 0;
  if (st == 'A')
    return 1;
  if (st == 'B')
    return 2;
  if (st == 'C')
    return 3;
  if (st == 'D')
    return 4;
  if (st == 'E')
    return 5;
  if (st == 'F')
    return 6;
  if (st == 'G')
    return 7;
}
//蜂鸣器响n秒
void Beep_S(int n) {
  BEEP.TurnOn();
  while (n--) {
    delay(1000);
  }
  BEEP.TurnOff();
}



void Road_Task() {    //地形标志物任务
  Move.Car_Go(1300);  //先直接冲过地形标志物
  delay(200);
  Move.Car_Track(60);  //在循迹到下个路口
  delay(200);
  Move.Car_Go(320);  //向前走一段距离
}



void Road_F4ToB4_Test() {
  delay(1000);
  DCMotor.Back(60, 800);
  delay(500);
  DCMotor.CarTrack(60);
  delay(100);
  DCMotor.Go(60, 100);
  delay(500);
  DCMotor.Line_Go(80, 400);
  delay(100);
  Road_Task();
}
