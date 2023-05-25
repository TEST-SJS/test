#ifndef _Move_h
#define _Move_h

#define UP 1
#define DOWN 2
#define RIGHT 4
#define LEFT 3		//上下左右的定义 
#define LONGITUDINAL 55
#define TRANSVERSER 66

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
class _Move
{
public:
	_Move();
	~_Move();
	//上传同步字节函数 
	void Data_Init(void);
	//读取接收ZigBee字节数组以及变量 
	uint8_t ZigBee_Back_lys[16] = { 0x55, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t Syn_Byte = 0x00;
	void Syn_Data(uint8_t data);//用于提取同步字节 
	void Send_Syn(void);		//发送同步字节 
	
	
	 
	//副车移动 
	void Car_Stop(void);
	void Car_Go(uint16_t _distance); //（实际是延时） 
	void Car_Back(uint16_t _distance);//（实际是延时） 
	void Car_Left(uint16_t _distance);//（实际是延时） 
	void Car_Right(uint16_t _distance);//（实际是延时） 
	void Car_Track(uint8_t speed);
	void Car_LineGo(uint8_t Car_Spend,uint16_t _distance);
	void Turn_Left(uint16_t _distance);
	void Turn_Right(uint16_t _distance);
	//自动路径函数方法声明
	void AutoPath(int xx,int yy,int dir);
	int PointToInt(char N);
	void filtration(void);
	void action_befor(void);
	void action_Last(void);
	void Auto(void);
	void first_dir(void);
	void horizontal_turn(void);
	void vertival_turn(void);
	void horizontal_run(void);
	void vertical_run(void) ;
	void turning(int dir);

	boolean start_xMax_flag=false,start_xMin_flag=false,over_xMax_flag=false,over_xMin_flag=false;
	boolean start_yMax_flag=false,start_yMin_flag=false,over_yMax_flag=false,over_yMin_flag=false;
	//坐标点定义 初始化
	int StartPoint_x,StartPoint_y,OverPoint_x,OverPoint_y,CurrentPoint_x,CurrentPoint_y,OverDir;
	int startCarDir=UP;//起始车头朝向
	int currentDir=UP;//当前朝向
	int obstacleAspect=0;
	int xMax=7,xMin=1,yMax=7,yMin=1;//坐标边界
	
	int X_Start,Y_Start,Dir_Start; 	//坐标点以及车头指向 
	u8 Freely_Flag;					//行走优先级 
	private:
};


extern _Move    Move;
#endif
