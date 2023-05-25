#ifndef _Move_h
#define _Move_h

#define UP 1
#define DOWN 2
#define RIGHT 4
#define LEFT 3		//�������ҵĶ��� 
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
	//�ϴ�ͬ���ֽں��� 
	void Data_Init(void);
	//��ȡ����ZigBee�ֽ������Լ����� 
	uint8_t ZigBee_Back_lys[16] = { 0x55, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t Syn_Byte = 0x00;
	void Syn_Data(uint8_t data);//������ȡͬ���ֽ� 
	void Send_Syn(void);		//����ͬ���ֽ� 
	
	
	 
	//�����ƶ� 
	void Car_Stop(void);
	void Car_Go(uint16_t _distance); //��ʵ������ʱ�� 
	void Car_Back(uint16_t _distance);//��ʵ������ʱ�� 
	void Car_Left(uint16_t _distance);//��ʵ������ʱ�� 
	void Car_Right(uint16_t _distance);//��ʵ������ʱ�� 
	void Car_Track(uint8_t speed);
	void Car_LineGo(uint8_t Car_Spend,uint16_t _distance);
	void Turn_Left(uint16_t _distance);
	void Turn_Right(uint16_t _distance);
	//�Զ�·��������������
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
	//����㶨�� ��ʼ��
	int StartPoint_x,StartPoint_y,OverPoint_x,OverPoint_y,CurrentPoint_x,CurrentPoint_y,OverDir;
	int startCarDir=UP;//��ʼ��ͷ����
	int currentDir=UP;//��ǰ����
	int obstacleAspect=0;
	int xMax=7,xMin=1,yMax=7,yMin=1;//����߽�
	
	int X_Start,Y_Start,Dir_Start; 	//������Լ���ͷָ�� 
	u8 Freely_Flag;					//�������ȼ� 
	private:
};


extern _Move    Move;
#endif
