//
//
//

#include "DCMotor.h"
#include <Command.h>
#include "wiring_private.h"
#include <ExtSRAMInterface.h>
#include <Metro.h>
#include <BEEP.h>


_DCMotor DCMotor;
Metro DCMotorMetro(20);

_DCMotor::_DCMotor() {
	ExtSRAMInterface.Initialization();
}

_DCMotor::~_DCMotor() {
}


/************************************************************************************************************
���� �� ������	Initialization	ֱ�������ʼ��
������˵������	fHz		��		��ʼ��PWM���Ƶ�ʣ���λ��Hz
���� �� ֵ����	��
����    ������	Initialization(8000);
************************************************************************************************************/
void _DCMotor::Initialization(uint32_t fHz) {
	/*ExtSRAMInterface.Initialization();*/
	pinMode(L_CONTROL_PIN, OUTPUT);
	pinMode(R_CONTROL_PIN, OUTPUT);


	//RFSpeed = 0;
	//RBSpeed = 0;
	//LFSpeed = 0;
	//LBSpeed = 0;

	pinMode(R_F_M_PIN, OUTPUT);
	pinMode(R_B_M_PIN, OUTPUT);
	pinMode(L_F_M_PIN, OUTPUT);
	pinMode(L_B_M_PIN, OUTPUT);

	TCCR4A = 0x00;
	TCCR4B = 0x00;
	TCCR3A = 0x00;
	TCCR3B = 0x00;

	//ѡ����ģʽ,ģʽ14--fast PWM
	TCCR4A |= _BV(WGM41);
	TCCR4B |= _BV(WGM42) | _BV(WGM43);
	TCCR3A |= _BV(WGM31);
	TCCR3B |= _BV(WGM32) | _BV(WGM33);

	//����PWM����Ƶ��
	MotorFrequency(fHz);

	//����PWM����ռ�ձ�
	//SpeedCtr(0, 0);
	Stop();
	//MotorSpeed(MOVE_RUNMODE_STOP,_Fspeed, _Bspeed);
	//LeftMotorSpeed(_Fspeed, _Bspeed);

	//�������ͨ��
	//TCCR4A |= _BV(COM4C1) | _BV(COM4B1) | _BV(COM4C0) | _BV(COM4B0);
	//TCCR3A |= _BV(COM3C1) | _BV(COM3B1) | _BV(COM3C0) | _BV(COM3B0);
	TCCR4A |= _BV(COM4C1) | _BV(COM4B1);
	TCCR3A |= _BV(COM3C1) | _BV(COM3B1);

	ParameterInit();
}

void _DCMotor::ParameterInit(void) {
	for (uint8_t i = 0; i < 101; i++) {	//ռ�ձȳ�ʼ��
		speed[i] = fHz * i / 100;
	}
}

void _DCMotor::SpeedSetOne(int16_t s, uint8_t *c1, uint8_t *c2) {
	uint8_t t;
	t = (s >= 0) ? s : s*(-1);
	if (t > 100)
		t = 100;
	if (t < 5)
		t = 5;
	if (s == 0) {
		*c1 = speed[100];		//100;
		*c2 = speed[100];		//100;
	} else if (s > 0) {
		*c1 = speed[t];
		*c2 = speed[0];
	} else {
		*c1 = speed[0];
		*c2 = speed[t];
	}
}

void _DCMotor::SpeedCtr(int16_t L_speed, int16_t R_speed) {
	uint8_t ocr3b, ocr3c, ocr4b, ocr4c;

	SpeedSetOne(L_speed, &ocr4c, &ocr4b);
	SpeedSetOne(R_speed, &ocr3b, &ocr3c);

	(ocr3b == 0) ? (TCCR3A |= _BV(COM3B0), ocr3b = fHz) : (TCCR3A &= ~_BV(COM3B0));
	(ocr3c == 0) ? (TCCR3A |= _BV(COM3C0), ocr3c = fHz) : (TCCR3A &= ~_BV(COM3C0));
	(ocr4b == 0) ? (TCCR4A |= _BV(COM4B0), ocr4b = fHz) : (TCCR4A &= ~_BV(COM4B0));
	(ocr4c == 0) ? (TCCR4A |= _BV(COM4C0), ocr4c = fHz) : (TCCR4A &= ~_BV(COM4C0));

	OCR4C = ocr4c;
	OCR4B = ocr4b;
	OCR3C = ocr3c;
	OCR3B = ocr3b;
}

bool _DCMotor::ClearCodeDisc(void) {
	uint16_t distance;
	unsigned long t;
	Command.Judgment(Command.command01);
	for (size_t i = 0; i < 8; i++) {
		ExtSRAMInterface.ExMem_JudgeWrite(WRITEADDRESS + i, Command.command01[i]);
	}
//	t = millis();
	DCMotorMetro.interval(20);
	for (size_t i = 0; i < 5; i++) {
		if (DCMotorMetro.check() == 1) {
			distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
			if (distance == 0x0000) {
				return false;
			}
		}
	}
	return true;
}
/************************************************************************************************************
С����������غ���
************************************************************************************************************/
/************************************************************************************************************
���� �� ������  Go		С��ǰ������
������˵������	speed	�������ٶ�
distance: ����ǰ���ľ���
���� �� ֵ����	��
����    ������	Go(70);	С��������ǰ����ǰ���ٶȣ�70
************************************************************************************************************/
void _DCMotor::Go(uint8_t speed) {
	SpeedCtr(speed, speed);
	StartUp();
}
uint16_t _DCMotor::Go(uint8_t speed, uint16_t _distance) {
	unsigned long t;
	uint16_t distance;
	while (ClearCodeDisc()) {
		//DCMotor.SpeedCtr(80, 80);
		//DCMotor.StartUp();
		//delay(20);
		//DCMotor.Stop();
	}
	SpeedCtr(speed, speed);
	StartUp();

	t = millis();
	do {
		distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
		if ((65516 > distance) && (distance > 20)) {
			if ((distance >= _distance) || ((millis() - t) >= 30000))
				//if (((distance >= _distance) && ((millis() - t) < 10000)) || (ExtSRAMInterface.ExMem_Read(0x6100) == 0x55))
				//if ((65535 - distance >= _distance) || ((millis() - t) >= 10000))
			{
				Stop();
				//ShutDown();
				delay(50);
				distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
				break;
			}
		}
		delay(1);
	} while (true);
	return distance;
	//return (65535 - distance);
}

/************************************************************************************************************
���� �� ������  Back		С�����˺���
������˵������	speed	:	�����ٶ�
distance:	���ú��˵ľ���
���� �� ֵ����	��
����    ������	Back(70);	С�����������ˣ������ٶȣ�70
************************************************************************************************************/
void _DCMotor::Back(uint8_t speed) {
	SpeedCtr(speed*(-1), speed*(-1));
	StartUp();
}
uint16_t _DCMotor::Back(uint8_t speed, uint16_t _distance)
{
	unsigned long t;
	uint16_t distance;

	while (ClearCodeDisc())
	{

	}
	Back(speed);
	t = millis();
	do
	{
		distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
		if ((65516 > distance) && (distance > 20))
		{	
			if (((65536 - distance) >= _distance) || ((millis() - t) > 30000))
			{
				Stop();
				delay(50);
				break;
			}
		}
		delay(10);
	} while (true);
	return (65536 - distance);
}

/************************************************************************************************************
���� �� ������  TurnLeft	С����ת����,Lspeed <= Rspeed
������˵������	Lspeed	��	���������ٶ�
Rspeed	��	���������ٶ�
���� �� ֵ����	��
����    ������	TurnLeft(70);	С����������ת����ת�ٶȣ�70
************************************************************************************************************/
void _DCMotor::TurnLeft(int8_t Lspeed, int8_t Rspeed) {
	SpeedCtr(Lspeed*(-1), Rspeed);
	StartUp();
}
void _DCMotor::TurnLeft(int8_t speed) {
	uint8_t tgd, tp;
	unsigned long t;
	uint8_t  trackval;
	while (ClearCodeDisc()) {
		//DCMotor.SpeedCtr(-80, 80);
		//DCMotor.StartUp();
		//delay(20);
		//DCMotor.Stop();
	}
	SpeedCtr(speed*(-1), speed);
	StartUp();
	//TurnLeft(speed);
	do {
		tgd = ExtSRAMInterface.ExMem_Read(BASEADDRESS + TRACKOFFSET);
		tp = SearchBit(1,tgd);
		if ((tp <= 0x04) || (ExtSRAMInterface.ExMem_Read(0x6100) == 0x55))	break;
	} while (true);

	t = millis();
	do {
		trackval = ExtSRAMInterface.ExMem_Read(BASEADDRESS + TRACKOFFSET);
		if ((!(trackval & 0x10)) || ((millis() - t) > 10000))
			//if ((SearchBit(1, tgd) <= 0x10) || ((millis() - t) > 10000))
		{
			Stop();
			delay(50);
			//ShutDown();
			break;
		}
	} while (true);
}


/************************************************************************************************************
���� �� ������  TurnRight	С����ת����,Rspeed <= Lspeed
������˵������	Lspeed	��	���������ٶ�
Rspeed	��	���������ٶ�
���� �� ֵ����	��
����    ������	TurnRight(70);	С����������ת����ת�ٶȣ�70
************************************************************************************************************/
void _DCMotor::TurnRight(int8_t Lspeed, int8_t Rspeed) {
	SpeedCtr(Lspeed, Rspeed*(-1));
	StartUp();
}
void _DCMotor::TurnRight(int8_t speed) {
	uint8_t tgd, tp;
	unsigned long t;
	uint8_t  trackval;
	while (ClearCodeDisc()) {
		//DCMotor.SpeedCtr(80, -80);
		//DCMotor.StartUp();
		//delay(20);
		//DCMotor.Stop();
	}

	SpeedCtr(speed, speed*(-1));
	StartUp();
	//TurnRight(speed);
	do {
		tgd = ExtSRAMInterface.ExMem_Read(BASEADDRESS + TRACKOFFSET);
		tp = SearchBit(0,tgd);
		//Serial.println("start TurnRight!");
		//Serial.println(tp,16);
		if ((tp >= 0x20) || (ExtSRAMInterface.ExMem_Read(0x6100) == 0x55))	break;
	} while (true);

	t = millis();
	do {
		trackval = ExtSRAMInterface.ExMem_Read(BASEADDRESS + TRACKOFFSET);

		//if ((SearchBit(0, trackval) >= 0x08) || ((millis() - t) > 10000))
		if ((!(trackval & 0x08)) || ((millis() - t) > 10000)) {
			Stop();
			//ShutDown();
			delay(100);
			break;
		}
		//Serial.println(trackval, 16);
	} while (true);
}

void _DCMotor::TurnLeftToRight(void) {

}
void _DCMotor::TurnRightToLeft(void) {

}
/************************************************************************************************************
���� �� ������  Stop	С��ֹͣ����
������˵������	��
���� �� ֵ����	��
����    ������	Stop();	С��������ֹͣ
************************************************************************************************************/
void _DCMotor::Stop(void) {
	/*�����δ��붼�е��ֹͣ���ܣ���ֹͣ��Ч����һ��*/

	/**********���������******************/
	//PORTE &= ~_BV(PE3);
	//PORTH &= ~_BV(PH3);
	//SpeedCtr(0, 0);
	/***********END**************************/

	//���˲������
	SpeedCtr(0, 0);
	PORTE |= _BV(PE3);
	PORTH |= _BV(PH3);
	/**********END************/
}

/************************************************************************************************************
���� �� ������  StartUp		С����������
������˵������	��
���� �� ֵ����	��
����    ������	StartUp();	С������������
************************************************************************************************************/
void _DCMotor::StartUp(void) {
	PORTE |= _BV(PE3);
	PORTH |= _BV(PH3);
	//digitalWrite(L_CONTROL_PIN, HIGH);
	//digitalWrite(R_CONTROL_PIN, HIGH);
}

/************************************************************************************************************
���� �� ������  ShutDown	С���رպ���
������˵������	��
���� �� ֵ����	��
����    ������	ShutDown();	С���������ر�
************************************************************************************************************/
void _DCMotor::ShutDown(void) {
	SpeedCtr(100, 100);
	PORTE |= _BV(PE3);
	PORTH |= _BV(PH3);
}
/*************************************************************END***********************************************/

void _DCMotor::CarTrack(uint8_t Car_Spend) {
	unsigned long t;
	uint8_t mode, _mode = 10;
	uint8_t tp;
	uint8_t ogd, tgd;
	uint8_t LSpeed, RSpeed;
	uint16_t count = 0;
	uint8_t  firstbit[8];

	ogd = 0xff;
	StartUp();
	while (true) {
		
		tp = 0;
		firstbit[0] = 0;
		//tgd = ExtSRAMInterface.ExMem_Read(TRACK_ADDR);
		//if (ogd != tgd)
		//	gd = ogd & tgd;
		//else
		//	gd = tgd;
		gd = ExtSRAMInterface.ExMem_Read(TRACK_ADDR);
		for (size_t i = 0x01; i<0x100; i <<= 1) {
			if ((gd & i) == 0) {
				firstbit[tp++] = uint8_t(i);
			}
		}
		//20190408lys ��ӵĴ��� �ж��Ƿ��ҵ�ѭ�����㷨 
		TJ();
	
		if ((tp >= 0x07)) {		/*ѭ������ĸ�����7��ʾȫ��*/
			//ShutDown();
			Stop();
			delay(100);
			gd = ExtSRAMInterface.ExMem_Read(TRACK_ADDR);
			TJ();
			break;
		} else if((MN>=3)&&(LN<=1)&&(RN<=1)) {
			Stop();
			delay(100);
			gd = ExtSRAMInterface.ExMem_Read(TRACK_ADDR);
			TJ();
			break;
		} else if((MN<=3)&&(LN>=1)&&(RN==0)) {
			Stop();
			delay(100);
			gd = ExtSRAMInterface.ExMem_Read(TRACK_ADDR);
			TJ();
			break;
		} else if((MN<=3)&&(RN>=1)&&(LN==0)) {
			Stop();
			delay(100);
			gd = ExtSRAMInterface.ExMem_Read(TRACK_ADDR);
			TJ();
			break;
                                 }//else if((MN<=3)&&(RN<=1)&&(LN>=1)) {
			//Stop();
			//delay(100);
			//gd = ExtSRAMInterface.ExMem_Read(TRACK_ADDR);
			//TJ();
			//break;
		//} 
		else {
//#if 0
			switch (firstbit[0]) {
				case 0x00:
					SpeedCtr(Car_Spend, Car_Spend);
					break;
				case 0x01:
					SpeedCtr(Car_Spend + 60, Car_Spend - 120);
					break;
				case 0x02:
					SpeedCtr(Car_Spend + 40, Car_Spend - 70);
					break;
				case 0x04:
					SpeedCtr(Car_Spend + 30, Car_Spend - 30);
					break;
				case 0x08:
					SpeedCtr(Car_Spend, Car_Spend);
					break;
				case 0x10:
					SpeedCtr(Car_Spend - 30, Car_Spend + 30);
					break;
				case 0x20:
					SpeedCtr(Car_Spend - 70, Car_Spend + 40);
					break;
				case 0x40:
					SpeedCtr(Car_Spend - 120, Car_Spend + 60);
					break;
				case 0x80:
					SpeedCtr(Car_Spend - 120, Car_Spend + 60);
					break;
			}
			//if ((firstbit[0] == 0x00) || (firstbit[0] == 0x08))
			//{
			//	//DCMotor.Go(Car_Spend);
			//	SpeedCtr(Car_Spend, Car_Spend);
			//}
			//else if (mode < 4)
			//{
			//	LSpeed = (Car_Spend + 60) - (mode * 10);
			//	RSpeed = (Car_Spend - 120) + (mode * 30);
			//	DCMotor.SpeedCtr(LSpeed, RSpeed);
			//}
			//else
			//{
			//	RSpeed = Car_Spend + ((mode - 1) * 10);
			//	LSpeed = (Car_Spend - ((mode - 3) * 30));
			//	DCMotor.SpeedCtr(LSpeed, RSpeed);
			//}
//#else
//			DCMotor.Go(Car_Spend);
//#endif
		}
		
	}
}

/**Ѱ��ָ������

*/
void _DCMotor::Line_Go(uint8_t Car_Spend,uint16_t _distance) {
	unsigned long t;
	uint8_t mode, _mode = 10;
	uint8_t tp;
	uint8_t gd, ogd, tgd;
	uint8_t LSpeed, RSpeed;
	uint16_t count = 0;
	uint8_t  firstbit[8];
	uint16_t distance;
	ogd = 0xff;



	while (ClearCodeDisc()) { //�������ֵ
	}

	StartUp();

	while (true) {
		tp = 0;
		firstbit[0] = 0;
		gd = ExtSRAMInterface.ExMem_Read(TRACK_ADDR);
		for (size_t i = 0x01; i<0x100; i <<= 1) {
			if ((gd & i) == 0) {
				firstbit[tp++] = uint8_t(i);
			}
		}

		distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));



		if ((65516 > distance) && (distance > 20)) {
			if ((distance >= _distance) ) {
				Stop();


				break;
			}
		}

		switch (firstbit[0]) {
			case 0x00:
				SpeedCtr(Car_Spend, Car_Spend);
				break;
			case 0x01:
				SpeedCtr(Car_Spend + 60, Car_Spend - 120);
				break;
			case 0x02:
				SpeedCtr(Car_Spend + 40, Car_Spend - 70);
				break;
			case 0x04:
				SpeedCtr(Car_Spend + 30, Car_Spend - 30);
				break;
			case 0x08:
				SpeedCtr(Car_Spend, Car_Spend);
				break;
			case 0x10:
				SpeedCtr(Car_Spend - 30, Car_Spend + 30);
				break;
			case 0x20:
				SpeedCtr(Car_Spend - 70, Car_Spend + 40);
				break;
			case 0x40:
				SpeedCtr(Car_Spend - 120, Car_Spend + 60);
				break;
			case 0x80:
				SpeedCtr(Car_Spend - 120, Car_Spend + 60);
				break;
		}

	}
}








uint8_t _DCMotor::SearchBit(uint8_t mode,uint8_t s) {
	if (mode == 1) {
		for (size_t i = 0x80; i > 0x00; i >>= 1) {
			if ((s & i) == 0)	return i;
		}
		return 0;
	} else {
		for (size_t i = 0x01; i < 0x100; i <<= 1) {
			if ((s & i) == 0)	return i;
		}
		return 0xff;
	}
}



/************************************************************************************************************
���� �� ������  RightMotorSpeed	����PWM����ռ�ձȣ�Ƶ�ʱ������һ������ֵ  ͨ��A-----pin6��ͨ��B-----pin7
������˵������	_Fduty	������ͨ�� A ���PWM����ռ�ձ�
				_Bduty	������ͨ�� B ���PWM����ռ�ձ�
���� �� ֵ����	��
����    ������	RightMotorSpeed(70,0);
************************************************************************************************************/
void _DCMotor::MotorSpeed(uint8_t runmode, int8_t l_speed, int8_t r_speed) {
	/*sbi(TCCR4A, COM4C1);
	sbi(TCCR4A, COM4B1);
	sbi(TCCR3A, COM3C1);
	sbi(TCCR3A, COM3B1);*/
	switch (runmode) {
		case MOVE_RUNMODE_STOP:
			PORTE &= ~(_BV(PE4) | _BV(PE5));
			PORTH &= ~(_BV(PH4) | _BV(PH5));
			OCR4C = 0x0000;
			OCR4B = 0x0000;
			OCR3C = 0x0000;
			OCR3B = 0x0000;
			break;
		case MOVE_RUNMODE_FF:
			OCR4C = 0;
			OCR4B = fHz * l_speed / 100;
			OCR3C = 0;
			OCR3B = fHz * r_speed / 100;
			break;
		case MOVE_RUNMODE_BB:
			OCR4B = 0;
			OCR4C = fHz * l_speed / 100;
			OCR3B = 0;
			OCR3C = fHz * r_speed / 100;
			break;
		case MOVE_RUNMODE_LIFT:
			OCR4C = 0;
			OCR4B = fHz * l_speed / 100;
			OCR3B = 0;
			OCR3C = fHz * r_speed / 100;
			break;
		case MOVE_RUNMODE_RIGHT:
			OCR4B = 0;
			OCR4C = fHz * l_speed / 100;
			OCR3C = 0;
			OCR3B = fHz * r_speed / 100;
			break;
		case MOVE_RUNMODE_TRACK:

			break;
	}
}

/************************************************************************************************************
���� �� ������  RightMotorFrequency	����PWM����Ƶ�ʣ�ռ�ձȱ������һ������ֵ  ͨ��A-----pin6��ͨ��B-----pin7
������˵������	_fHz	������ͨ�� A/B ���PWM����Ƶ��
���� �� ֵ����	��
����    ������	RightMotorFrequency(7000);	//Ƶ�ʷ�ΧΪ245Hz��8MHz
************************************************************************************************************/
void _DCMotor::MotorFrequency(uint32_t _fHz) {
	fHz = 16000000 / _fHz;
	TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
	TCCR4B |= _BV(CS40);
	TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
	TCCR3B |= _BV(CS30);

	ICR4 = fHz;
	ICR3 = fHz;
}
/***************************************************END*****************************************************/


/***************************Timer5����Fast PWM����ģʽ��������һЩ���ú���**********************************/

/************************************************************************************************************
���� �� ������  LeftMotorSpeed	����PWM����ռ�ձȣ�Ƶ�ʱ������һ������ֵ  ͨ��A-----pin46��ͨ��B-----pin45
������˵������	_front_speed	������ͨ�� A ���PWM����ռ�ձ�
				_behind_speed	������ͨ�� B ���PWM����ռ�ձ�
���� �� ֵ����	��
����    ������	LeftMotorSpeed(70,50);
************************************************************************************************************/
void _DCMotor::LeftMotorSpeed(uint8_t _front_speed, uint8_t _behind_speed) {
	//LfHz = TCNT3;
	LFSpeed = _front_speed;
	LBSpeed = _behind_speed;
	if ((_front_speed == 100) && (_behind_speed == 100)) {
		sbi(TCCR3A, COM3C1);
		sbi(TCCR3A, COM3B1);
		OCR3C = 0xffff;// LfHz;
		OCR3B = 0xffff;// LfHz;
	} else {
		if (_front_speed >= 100) {
			sbi(TCCR3A, COM3C1);
			OCR3C = 0xffff;
		} else if (_front_speed <= 0) {
			sbi(TCCR3A, COM3C1);
			OCR3C = 0;
		} else {
			sbi(TCCR3A, COM3C1);
			OCR3C = LfHz * _front_speed / 100;
		}

		if (_behind_speed >= 100) {
			sbi(TCCR3A, COM3B1);
			OCR3B = 0xffff;

		} else if(_behind_speed <= 0) {
			sbi(TCCR3A, COM3B1);
			OCR3B = 0;

		} else {
			sbi(TCCR3A, COM3B1);
			OCR3B = LfHz * _behind_speed / 100;
		}

	}


}

/************************************************************************************************************
���� �� ������  LeftMotorFrequency	����PWM����Ƶ�ʣ�ռ�ձȱ������һ������ֵ  ͨ��A-----pin46��ͨ��B-----pin45
������˵������	_fHz	������ͨ�� A/B ���PWM����Ƶ��
���� �� ֵ����	��
����    ������	LeftMotorFrequency(7000);
************************************************************************************************************/
void _DCMotor::LeftMotorFrequency(uint32_t _fHz) {
	LfHz = _fHz;

	if (_fHz > 1000000) {	//Ƶ�����Ϊ1MHz
		TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
		TCCR3B |= _BV(CS30);
		ICR3 = 16;
		LfHz = 16;
		LeftMotorSpeed(LFSpeed, LBSpeed);
	} else if (_fHz < 30) {	//Ƶ����СΪ30Hz
		TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
		TCCR3B |= _BV(CS31);
		ICR3 = 0xffff;
		LfHz = 0xffff;
		LeftMotorSpeed(LFSpeed, LBSpeed);
	} else {			//Ƶ�ʷ�ΧΪ30Hz~1MHz
		LfHz = 16000000 / _fHz;
		if (LfHz >= 0xffff) {
			TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
			TCCR3B |= _BV(CS31);
			LfHz = 2000000 / _fHz;
			ICR3 = LfHz;
			LeftMotorSpeed(LFSpeed, LBSpeed);
		} else {
			TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
			TCCR3B |= _BV(CS30);
			/*LfHz = 16000000 / _fHz;*/
			ICR3 = LfHz;
			LeftMotorSpeed(LFSpeed, LBSpeed);
		}
	}
}


uint8_t _DCMotor::ShiftBitAdd(uint8_t n) {
	uint8_t bit = 0;
	for (uint8_t i = 0; i < 8; i++) {
		/*if (((n >> i) & 0x01) == 0x01)	bit ++;*/
		bit += bitRead(n, i);
	}
	return bit;
}

uint8_t _DCMotor::JudgeAround(uint8_t n) {
	for (uint8_t i = 0; i < 4; i++) {
		if ((((n >> 3) & 0x01) == 0x00) && (((n >> 4) & 0x01) == 0x00)) {
			return 8;
		} else if (((n >> (3 - i)) & 0x01) == 0x00) {
			return (3 - i);
		} else if (((n >> (4 + i)) & 0x01) == 0x00) {
			return (4 + i);
		}
	}
	return 9;
}
void _DCMotor::TJ(void) {
	RN=0;
	LN=0;
	MN=0;
//�ұ�����***************//
	if(gd&0x01) {
		RN++;
	}

	if(gd&0x02) {
		RN++;
	}
//�м���λ*******************//
	if(gd&0x04) {
		MN++;
	}

	if(gd&0x08) {
		MN++;
	}

	if(gd&0x10) {
		MN++;
	}

	if(gd&0x20) {
		MN++;
	}
//�����λ*********************//
	if(gd&0x40) {
		LN++;
	}

	if(gd&0x80) {
		LN++;
	}

}
