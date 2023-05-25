//20190318lys  ��д�Ŀ�  ���ڻ����ƶ�  ��������ϵ 
#include "DCMotor.h"
#include <Metro.h>
#include <Move.h>
#include <Command.h>
#include "wiring_private.h"
#include <ExtSRAMInterface.h>
#include <Beep.h>

#define Track_Speed 70  //ѭ��ʱ���ٶ� 
#define Go_Dis 280		//ѭ����ǰ����һС�ξ��� ��ʵ������ʱ�� 
//ǰ��������ʱ  200GO�ľ���С �����������ڵ������ʱ���������и��죩 
#define LineGo_Speed 80	//LineGoʱ���ٶ� 
#define LineGo_Dis 900	//LineGo�ľ���  ��ʵ������ʱ�� 
#define Left_Mp 875		//��ת����  ��ʵ������ʱ�� 
// 
#define Right_Mp 890	//��ת����  ��ʵ������ʱ�� 
				//ԭ900
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
	//�ϴ�ͬ���ֽ�����  ���������Ƚ� 

	int X_Start,Y_Start,Dir_Start; //��ʼ�����Լ���ͷָ�򣨸�ֵ�ã� 
	boolean start_xMax_flag=false,start_xMin_flag=false,over_xMax_flag=false,over_xMin_flag=false;
	boolean start_yMax_flag=false,start_yMin_flag=false,over_yMax_flag=false,over_yMin_flag=false;
	//����㶨�� ��ʼ��
	int StartPoint_x,StartPoint_y,OverPoint_x,OverPoint_y,CurrentPoint_x,CurrentPoint_y,OverDir;
	int startCarDir=UP;//��ʼ��ͷ����
	int currentDir=UP;//��ǰ����
	int obstacleAspect=0;
	int xMax=7,xMin=1,yMax=7,yMin=1;//����߽�
	int go_speed=80,      //ǰ���ٶ�
        go_distance=280,       //ǰ������
        line_speed=80,       //ѭ���ٶ�
        linego_distance_L=800,//ѭ����һ������ ����
        linego_distance_T=1000,//ѭ����һ������ ����
        left_speed=80,       //��ת�ٶ�
        right_speed=80;     //��ת�ٶ�
	int AutoFlag;   //������ɱ�־λ 


/************************************************************************************************************
���������ܡ���	����ǰ��
������˵������	��
���� �� ֵ����	��
����    ������	Move.Car_Stop(); 
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
��ȡ��ǰͬ���ֽ�  ��ֵ���ֲ����� 
*/
void _Move::Syn_Data(uint8_t data)
{
	Syn_Byte = data;
}
/*
�������ܣ�����ʵʱ���ص�ǰͬ���ֽ� 

*/ 
void _Move::Send_Syn(void){
	ZigBee_Back_lys[10] = Syn_Byte;
	ExtSRAMInterface.ExMem_Write_Bytes(0x6080, ZigBee_Back_lys, 16);
}


/*
 * �Զ�·��
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
//���� �����޶������ڵ�״̬
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
  startCarDir = Dir_Start;//����ʼ�����Լ���ͷָ��ֵ 

  OverPoint_x = xx;
  OverPoint_y = yy;			
  OverDir = dir;			//Ŀ��ص������Լ���ͷָ�� 
  
  CurrentPoint_x = StartPoint_x;
  CurrentPoint_y = StartPoint_y;
  currentDir = Dir_Start;//��ǰ�����Լ���ͷָ�� ��������ת��仯�� 
  
  Auto();
  while(AutoFlag != 1);
  AutoFlag = 0;
}
//�ж�ִ��
void _Move::Auto(){
        filtration();
        action_befor();
        boolean stopflag=true;
        while(stopflag){
            first_dir();//�ж����ȷ���
            if(obstacleAspect == TRANSVERSER    ){  //ˮƽ��
            	Send_Syn();//ʵʱ���͵�ǰͬ���ֽ� 
                horizontal_run();
            }
            if(obstacleAspect == LONGITUDINAL){//��ֱ��
            	Send_Syn();//ʵʱ���͵�ǰͬ���ֽ� 
                vertical_run();
            }
            if(CurrentPoint_x == OverPoint_x && CurrentPoint_y == OverPoint_y){//����Ŀ��
                stopflag=false;
            }
        }
        Send_Syn();//ʵʱ���͵�ǰͬ���ֽ� 
        action_Last();
        
        Send_Syn();//ʵʱ���͵�ǰͬ���ֽ� 
        turning(OverDir);
        AutoFlag = 1;
}

 //��ʻ���ȼ��ж�   ��̬�жϵ�ǰλ�� ������������ʻ����
 void _Move::first_dir(){
        if(CurrentPoint_x!=OverPoint_x||CurrentPoint_y!=OverPoint_y){ //x��yδ����
            if(CurrentPoint_x%2==1){//������Ϊ����
                obstacleAspect=TRANSVERSER;//��������
            }else {//������Ϊż�� �ж�������
                if (CurrentPoint_y % 2 == 1) {//������Ϊ����
                    obstacleAspect =LONGITUDINAL;//��������
                } else {//������Ϊż��.
                    if (CurrentPoint_x == OverPoint_x) {
                        obstacleAspect = LONGITUDINAL;//��������
                    }else if(CurrentPoint_y==OverPoint_y){
                        obstacleAspect= TRANSVERSER;//��������
                    }else if((CurrentPoint_x-OverPoint_x>=2||OverPoint_x-CurrentPoint_x>=2)&&(CurrentPoint_y-OverPoint_y==1||OverPoint_y-CurrentPoint_y==1)){
                        obstacleAspect=TRANSVERSER;
                    }else if((CurrentPoint_y-OverPoint_y>=2||OverPoint_y-CurrentPoint_y>=2)&&(CurrentPoint_x-OverPoint_x==1||OverPoint_x-CurrentPoint_x==1)){
                        obstacleAspect = LONGITUDINAL;//��������
                    }else{
                       	if(Freely_Flag){
							obstacleAspect= LONGITUDINAL;		//�������ȼ�
						}
						else{
							obstacleAspect=TRANSVERSER;			//Ĭ�Ϻ�������
						}
                    }
                }
            }
        }else{ //x����
            obstacleAspect=0;
        }
    }
   //ˮƽת��
 void _Move::horizontal_turn(){
        if(CurrentPoint_x<OverPoint_x){
            turning(LEFT);
            currentDir=LEFT;
        }else if (CurrentPoint_x>OverPoint_x) {
            turning(RIGHT);
            currentDir= RIGHT;
        }
    }
//��ֱת��
 void _Move::vertival_turn(){
        if(CurrentPoint_y<OverPoint_y){
            turning(UP);
        }else
        if (CurrentPoint_y>OverPoint_y) {
            turning(DOWN);
        }
    }

  //ˮƽ��ʻ
void _Move::horizontal_run(){
        horizontal_turn();
        /////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////
        if(CurrentPoint_x<OverPoint_x){//  ��ǰ���� С�� Ŀ������
            if(CurrentPoint_x%2==1){   //λ��������
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_x+=1;   //�ﵽż����
            }else if(CurrentPoint_x%2==0){//λ��ż����
                if (OverPoint_x-CurrentPoint_x>=2){//������ڵ������������
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
        if(CurrentPoint_x>OverPoint_x){    //��ǰ������� Ŀ������
            if(CurrentPoint_x%2==1){   //��ǰλ��������
                DCMotor.CarTrack(line_speed);
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
            }else if(CurrentPoint_x%2==0){//λ��ż������
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
//��ֱ��ʻ
void _Move::vertical_run() {
       vertival_turn();
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        if (CurrentPoint_y < OverPoint_y) {
            if(CurrentPoint_y%2==1){   //��ǰλ��������
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_y+=1;
            }else if(CurrentPoint_y%2==0){//λ��ż������
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
            if(CurrentPoint_y%2==1){   //��ǰλ��������
                Car_Track( Track_Speed);
				Car_Go(Go_Dis);
                CurrentPoint_y-=1;
            }else if(CurrentPoint_y%2==0) {//λ��ż������
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

 //�Զ�ת��
    /**
     * ת�򷽷�  ����Ŀ�ķ���  �Զ�ת��  enume Direction����
     * @param direction   ��Ҫת���ķ���
     */
void _Move::turning(int  dir){
        switch (currentDir){
            case DOWN: //��ǰ����
                switch(dir){//Ŀ�곯��
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
            case UP://��ǰ����
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
            case LEFT://��ǰ����
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
            case RIGHT://��ǰ����
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

//���ַ�ת��Ϊ��Ӧ����
    /**
     *  �������ַ�ת��Ϊ������
     * @param N ������ַ�'A'..'L' '0'...'7'
     * @return F  ���ظ�����
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

    //�������������   ֻ�����ڱ߽����һ���ֵ
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

