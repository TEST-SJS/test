//20190318lys  自写的库  用于基本移动  包含坐标系 
#include "DCMotor.h"
#include <Metro.h>
#include <Move.h>
#include <Command.h>
#include "wiring_private.h"
#include <ExtSRAMInterface.h>
#include <Beep.h>

#define Track_Speed 70  //循迹时的速度 
#define Go_Dis 280		//循迹完前进的一小段距离 （实际是延时） 
//前进电量多时  200GO的距离小 （可能是由于电量多的时候程序的运行更快） 
#define LineGo_Speed 80	//LineGo时的速度 
#define LineGo_Dis 900	//LineGo的距离  （实际是延时） 
#define Left_Mp 875		//左转码盘  （实际是延时） 
// 
#define Right_Mp 890	//右转码盘  （实际是延时） 
				//原900
_Move   Move;

_Move::_Move()
{
}

_Move::~_Move()
{
}

	void action_befor();
	void action_Last();
	void Auto();
	void first_dir();
	void horizontal_turn();
	void vertival_turn();
	void horizontal_run();
	void vertical_run() ;
	void turning(int dir);
	
	void AutoPath(int xx,int yy,int dir);
	int PointToInt(char N);
	void filtration();
	
	void Data_Init(void);
	//上传同步字节数据  方便主车比较 

	int X_Start,Y_Start,Dir_Start; //起始坐标以及车头指向（赋值用） 
	boolean start_xMax_flag=false,start_xMin_flag=false,over_xMax_flag=false,over_xMin_flag=false;
	boolean start_yMax_flag=false,start_yMin_flag=false,over_yMax_flag=false,over_yMin_flag=false;
	//坐标点定义 初始化
	int StartPoint_x,StartPoint_y,OverPoint_x,OverPoint_y,CurrentPoint_x,CurrentPoint_y,OverDir;
	int startCarDir=UP;//起始车头朝向
	int currentDir=UP;//当前朝向
	int obstacleAspect=0;
	int xMax=7,xMin=1,yMax=7,yMin=1;//坐标边界
	int go_speed=80,      //前进速度
        go_distance=280,       //前进距离
        line_speed=80,       //循迹速度
        linego_distance_L=800,//循迹走一定距离 纵向
        linego_distance_T=1000,//循迹走一定距离 横向
        left_speed=80,       //左转速度
        right_speed=80;     //右转速度
	int AutoFlag;   //坐标完成标志位 


/************************************************************************************************************
【函数功能】：	副车前进
【参数说明】：	无
【返 回 值】：	无
【简    例】：	Move.Car_Stop(); 
************************************************************************************************************/
void _Move::Car_Stop(void){
    DCMotor.Stop();
}
void _Move::Car_Go(uint16_t _distance){
    DCMotor.Go(80);
    delay(_distance);
    DCMotor.Stop();
    delay(100);
}
void _Move::Car_Back(uint16_t _distance){
    DCMotor.Back(80);
    delay(_distance);
    DCMotor.Stop();
    delay(100);
}
void _Move::Turn_Left(uint16_t _distance){
    DCMotor.TurnLeft(80,80);
    delay(_distance);
    DCMotor.Stop();
    delay(100);
}
void _Move::Turn_Right(uint16_t _distance){
    DCMotor.TurnRight(80,80);
    delay(_distance);
    DCMotor.Stop();
    delay(100);
}
void _Move::Car_Left(uint16_t _distance){
    DCMotor.TurnLeft(80,80);
    delay(_distance);
    DCMotor.Stop();
    switch(Dir_Start){
    	case UP:
			Dir_Start = LEFT;
		break;
		case DOWN:
			Dir_Start = RIGHT;
		break;
		case LEFT:
			Dir_Start = DOWN;
		break;
		case RIGHT:
			Dir_Start = UP;
		break;
	}
	delay(100);
}
void _Move::Car_Right(uint16_t _distance){
    DCMotor.TurnRight(80,80);
    delay(_distance);
    DCMotor.Stop();
    switch(Dir_Start){
    	case UP:
			Dir_Start = RIGHT;
		break;
		case DOWN:
			Dir_Start = LEFT;
		break;
		case LEFT:
			Dir_Start = UP;
		break;
		case RIGHT:
			Dir_Start = DOWN;
		break;
	}
	delay(100);
}
void _Move::Car_Track(uint8_t speed){
    DCMotor.CarTrack(speed);
    DCMotor.Stop();
    if(Dir_Start == UP){
    	if(Y_Start %2 != 0)
    	Y_Start += 1;
    	else
    	Y_Start += 2;
	}
	else if(Dir_Start == DOWN){
		if(Y_Start %2 != 0)
    	Y_Start -= 1;
    	else
    	Y_Start -= 2;
	}
	else if(Dir_Start == LEFT){
		if(X_Start %2 != 0)
    	X_Start += 1;
    	else
    	X_Start += 2;
	}
	else if(Dir_Start == RIGHT){
		if(X_Start %2 != 0)
    	X_Start -= 1;
    	else
    	X_Start -= 2;
	}
	delay(100);
}
void _Move::Car_LineGo(uint8_t Car_Spend,uint16_t _distance){
    DCMotor.Line_Go(Car_Spend, _distance);
    delay(500);
    DCMotor.Stop();
    
    switch(Dir_Start){
    	case UP:
    		Y_Start += 1;
    		break;
    		case DOWN:
    			Y_Start -= 1;
    		break;
    		case LEFT:
    			X_Start += 1;
    		break;
    		case RIGHT:
    			X_Start -= 1;
    		break;
	}
	delay(100);
}
/*
提取当前同步字节  赋值给局部变量 
*/
void _Move::Syn_Data(uint8_t data)
{
	Syn_Byte = data;
}
/*
函数功能：用于实时返回当前同步字节 

*/ 
void _Move::Send_Syn(void){
	ZigBee_Back_lys[10] = Syn_Byte;
	ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_Back_lys, 16);
}


/*
 * 自动路径
 * /
 */

 //1  action_befor()action_Last()
void _Move::action_befor(){
  if(start_xMax_flag){
        turning(RIGHT);
        Car_Track( Track_Speed);
		Car_Go(Go_Dis);
        CurrentPoint_x-=1;
        start_xMax_flag=false;
    }else if(start_xMin_flag){
        turning(LEFT);
        Car_Track( Track_Speed);
		Car_Go(Go_Dis);
        CurrentPoint_x+=1;
        start_xMin_flag=false;
    }
    if(start_yMax_flag){
                turning(DOWN);
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_y-=1;
                start_yMax_flag=false;
    }else if(start_yMin_flag){
                turning(UP);
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_y+=1;
                start_yMin_flag=false;
    }
}
//处理 不在限定坐标内的状态
 void _Move::action_Last(){
        if(CurrentPoint_x!= OverPoint_x){
            if(over_xMax_flag){
                turning(RIGHT);
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_x+=1;
                over_xMax_flag=false;
            }else if(over_xMin_flag){
                turning(LEFT);
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_x-=1;
                over_xMin_flag=false;
            }
        }
        if(CurrentPoint_y!= OverPoint_y) {
            if(over_yMax_flag){
                turning(UP);
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_y+=1;
                over_yMax_flag=false;
            }else if(over_yMin_flag){
                turning(DOWN);
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_y-=1;
                over_yMin_flag=false;
            }
        }

}
/**
 */
void _Move::AutoPath(int xx,int yy,int dir){
  StartPoint_x = X_Start;
  StartPoint_y = Y_Start;
  startCarDir = Dir_Start;//给起始坐标以及车头指向赋值 

  OverPoint_x = xx;
  OverPoint_y = yy;			
  OverDir = dir;			//目标地点坐标以及车头指向 
  
  CurrentPoint_x = StartPoint_x;
  CurrentPoint_y = StartPoint_y;
  currentDir = Dir_Start;//当前坐标以及车头指向 （会随着转弯变化） 
  
  Auto();
  while(AutoFlag != 1);
  AutoFlag = 0;
}
//判断执行
void _Move::Auto(){
        filtration();
        action_befor();
        boolean stopflag=true;
        while(stopflag){
            first_dir();//判断优先方向
            if(obstacleAspect == TRANSVERSER    ){  //水平走
            	Send_Syn();//实时发送当前同步字节 
                horizontal_run();
            }
            if(obstacleAspect == LONGITUDINAL){//垂直走
            	Send_Syn();//实时发送当前同步字节 
                vertical_run();
            }
            if(CurrentPoint_x == OverPoint_x && CurrentPoint_y == OverPoint_y){//到达目标
                stopflag=false;
            }
        }
        Send_Syn();//实时发送当前同步字节 
        action_Last();
        
        Send_Syn();//实时发送当前同步字节 
        turning(OverDir);
        AutoFlag = 1;
}

 //行驶优先级判定   动态判断当前位置 设置优先型行驶方向
 void _Move::first_dir(){
        if(CurrentPoint_x!=OverPoint_x||CurrentPoint_y!=OverPoint_y){ //x或y未到达
            if(CurrentPoint_x%2==1){//横坐标为奇数
                obstacleAspect=TRANSVERSER;//横向优先
            }else {//横坐标为偶数 判断纵坐标
                if (CurrentPoint_y % 2 == 1) {//纵坐标为奇数
                    obstacleAspect =LONGITUDINAL;//纵向优先
                } else {//纵坐标为偶数.
                    if (CurrentPoint_x == OverPoint_x) {
                        obstacleAspect = LONGITUDINAL;//纵向优先
                    }else if(CurrentPoint_y==OverPoint_y){
                        obstacleAspect= TRANSVERSER;//横向优先
                    }else if((CurrentPoint_x-OverPoint_x>=2||OverPoint_x-CurrentPoint_x>=2)&&(CurrentPoint_y-OverPoint_y==1||OverPoint_y-CurrentPoint_y==1)){
                        obstacleAspect=TRANSVERSER;
                    }else if((CurrentPoint_y-OverPoint_y>=2||OverPoint_y-CurrentPoint_y>=2)&&(CurrentPoint_x-OverPoint_x==1||OverPoint_x-CurrentPoint_x==1)){
                        obstacleAspect = LONGITUDINAL;//纵向优先
                    }else{
                       	if(Freely_Flag){
							obstacleAspect= LONGITUDINAL;		//纵向优先级
						}
						else{
							obstacleAspect=TRANSVERSER;			//默认横向优先
						}
                    }
                }
            }
        }else{ //x到达
            obstacleAspect=0;
        }
    }
   //水平转向
 void _Move::horizontal_turn(){
        if(CurrentPoint_x<OverPoint_x){
            turning(LEFT);
            currentDir=LEFT;
        }else if (CurrentPoint_x>OverPoint_x) {
            turning(RIGHT);
            currentDir= RIGHT;
        }
    }
//垂直转向
 void _Move::vertival_turn(){
        if(CurrentPoint_y<OverPoint_y){
            turning(UP);
        }else
        if (CurrentPoint_y>OverPoint_y) {
            turning(DOWN);
        }
    }

  //水平行驶
void _Move::horizontal_run(){
        horizontal_turn();
        /////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////
        if(CurrentPoint_x<OverPoint_x){//  当前坐标 小于 目的坐标
            if(CurrentPoint_x%2==1){   //位于奇数点
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_x+=1;   //达到偶数点
            }else if(CurrentPoint_x%2==0){//位于偶数点
                if (OverPoint_x-CurrentPoint_x>=2){//距离大于等于两个坐标点
                       Car_Track( Track_Speed);
						Car_Go(Go_Dis);
                        CurrentPoint_x+=2;
                }else if(OverPoint_x-CurrentPoint_x==1){
                    Car_LineGo(LineGo_Speed,LineGo_Dis);
                    CurrentPoint_x+=1;
                }
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////
        if(CurrentPoint_x>OverPoint_x){    //当前坐标大于 目的坐标
            if(CurrentPoint_x%2==1){   //当前位于奇数点
                DCMotor.CarTrack(line_speed);
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
            }else if(CurrentPoint_x%2==0){//位于偶数坐标
                if(CurrentPoint_x-OverPoint_x>=2){
                    Car_Track( Track_Speed);
					Car_Go(Go_Dis);
                    CurrentPoint_x-=2;
                }else if(CurrentPoint_x-OverPoint_x==1){
                    Car_LineGo(LineGo_Speed,LineGo_Dis);
                    CurrentPoint_x-=1;
                }
            }
        }
        if(CurrentPoint_x==OverPoint_x&&CurrentPoint_y!= OverPoint_y){
            if(CurrentPoint_x%2==1){
                if(currentDir==RIGHT){
                    Car_Track( Track_Speed);
					Car_Go(Go_Dis);
                    CurrentPoint_x-=1;
                }else if(currentDir==LEFT){
                    Car_Track( Track_Speed);
					Car_Go(Go_Dis);
                    CurrentPoint_x+=1;
                }
            }
        }
    }
//垂直行驶
void _Move::vertical_run() {
       vertival_turn();
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        if (CurrentPoint_y < OverPoint_y) {
            if(CurrentPoint_y%2==1){   //当前位于奇数点
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_y+=1;
            }else if(CurrentPoint_y%2==0){//位于偶数坐标
                if(OverPoint_y-CurrentPoint_y>=2){
                    Car_Track( Track_Speed);
					Car_Go(Go_Dis);
                    CurrentPoint_y+=2;
                }else if(OverPoint_y-CurrentPoint_y==1){
                   Car_LineGo(LineGo_Speed,LineGo_Dis);
                    CurrentPoint_y+=1;
                }
            }
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        if (CurrentPoint_y > OverPoint_y) {
            if(CurrentPoint_y%2==1){   //当前位于奇数点
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_y-=1;
            }else if(CurrentPoint_y%2==0) {//位于偶数坐标
                if(CurrentPoint_y-OverPoint_y>=2){
                    Car_Track( Track_Speed);
					Car_Go(Go_Dis);
                    CurrentPoint_y-=2;
                }else if(CurrentPoint_y-OverPoint_y==1){
                    Car_LineGo(LineGo_Speed,LineGo_Dis);
                    CurrentPoint_y-=1;
                }
            }
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        if(CurrentPoint_y==OverPoint_y&&CurrentPoint_x!= OverPoint_x){
            if(CurrentPoint_y%2==1){
                if(currentDir==DOWN){
                   	Car_Track( Track_Speed);
					Car_Go(Go_Dis);
                    CurrentPoint_y-=1;
                }else if(currentDir==UP){
                    Car_Track( Track_Speed);
					Car_Go(Go_Dis);
                    CurrentPoint_y+=1;
                }
            }
        }
    } 

 //自动转向
    /**
     * 转向方法  传入目的方向  自动转向  enume Direction类型
     * @param direction   需要转到的方向
     */
void _Move::turning(int  dir){
        switch (currentDir){
            case DOWN: //当前朝向
                switch(dir){//目标朝向
                    case UP:
//                    	delay(100);
                          Car_Left(Left_Mp);
                          delay(100);
                          Car_Left(Left_Mp);
                          DCMotor.Stop();
                          currentDir=UP;
                        break;
                    case LEFT:
//                    	delay(100);
                        Car_Right(Right_Mp);
                        currentDir=LEFT;
                        break;
                    case RIGHT:
//                    	delay(100);
                        Car_Left(Left_Mp);
                        currentDir=RIGHT;
                        break;
                    default:
                        break;
                }
                break;
            case UP://当前朝向
                switch(dir){
                    case DOWN:
//                    	delay(100);
                        Car_Left(Left_Mp);
                          delay(100);
                        Car_Left(Left_Mp);
                          DCMotor.Stop();
                        currentDir=DOWN;
                        break;
                    case LEFT:
//                    	delay(100);
                        Car_Left(Left_Mp);
                        currentDir=LEFT;
                        break;
                    case RIGHT:
//                    	delay(100);
                      	Car_Right(Right_Mp);
                        currentDir=RIGHT;
                        
                        break;
                    default:
                        break;
                }
                break;
            case LEFT://当前朝向
                switch(dir){
                    case UP:
//                    	delay(100);
                       Car_Right(Right_Mp);
                        currentDir=UP;
                        break;
                    case DOWN:
//                    	delay(100);
                        Car_Left(Left_Mp);
                        currentDir=DOWN;
                        break;
                    case RIGHT:
//                    	delay(100);
                        Car_Left(Left_Mp);
                        delay(100);
                        Car_Left(Left_Mp);
                        DCMotor.Stop();
                        currentDir=RIGHT;
                        break;
                    default:
                        break;
                }
                break;
            case RIGHT://当前朝向
                switch(dir){
                    case UP:
//                    	delay(100);
                        Car_Left(Left_Mp);
                        currentDir=UP;
                        break;
                    case LEFT:
//                    	delay(100);
                        Car_Left(Left_Mp);
                          delay(100);
                        Car_Left(Left_Mp);
                          DCMotor.Stop();
                        currentDir=LEFT;
                        break;
                    case DOWN:
//                    	delay(100);
                        Car_Right(Right_Mp);
                        currentDir=DOWN;
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }

//将字符转换为对应数字
    /**
     *  将坐标字符转换为浮点型
     * @param N 传入的字符'A'..'L' '0'...'7'
     * @return F  返回浮点数
     */
int _Move::PointToInt(char N) {
        int F = 0;
        switch (N) {
            case 'A':
                F = 1;
                break;
            case 'B':
                F = 2;
                break;
            case 'C':
                F = 3;
                break;
            case 'D':
                F = 4;
                break;
            case 'E':
                F = 5;
                break;
            case 'F':
                F = 6;
                break;
            case 'G':
                F = 7;
                break;
            case 'H':
                F = 8;
                break;
            case 'I':
                F = 9;
                break;
            case 'a':
                F = 1;
                break;
            case 'b':
                F = 2;
                break;
            case 'c':
                F = 3;
                break;
            case 'd':
                F = 4;
                break;
            case 'e':
                F = 5;
                break;
            case 'f':
                F = 6;
                break;
            case 'g':
                F = 7;
                break;
            case 'h':
                F = 8;
                break;
            case 'i':
                F = 9;
                break;


            case '1':
                F = 1;
                break;
            case '2':
                F = 2;
                break;
            case '3':
                F = 3;
                break;
            case '4':
                F = 4;
                break;
            case '5':
                F = 5;
                break;
            case '6':
                F = 6;
                break;
            case '7':
                F = 7;
                break;
            case '8':
                F=8;
                break;
            case '9':
                F =9;
                break;
            default:
                break;
        }
        return F;
    }

    //过滤输入坐标点   只接受在边界点内一点的值
void _Move::filtration(){
        if(StartPoint_x==xMax){start_xMax_flag=true;}
        if(StartPoint_x==xMin){start_xMin_flag=true;}
        if(StartPoint_y==yMax){start_yMax_flag=true;}
        if(StartPoint_y==yMin){start_yMin_flag=true;}

        if(OverPoint_x==xMax){over_xMax_flag=true;}
        if(OverPoint_x==xMin){over_xMin_flag=true;}
        if(OverPoint_y==yMax){over_yMax_flag=true;}
        if(OverPoint_y==yMin){over_yMin_flag=true;}
}

