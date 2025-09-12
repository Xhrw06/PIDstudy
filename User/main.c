#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "RP.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "AD.h"
#include "PID.h"

#define CENTER_ANGLE 1995
#define CENTER_RANGE 500
#define START_PWM 60
#define START_TIME 120

uint8_t KeyNum;
uint8_t RunState;//PID工作状态
uint16_t Angle;
float Speed,Location;

PID_t Angle_PID = 
{
	.Target = CENTER_ANGLE,
	.Kp = 0.255,
	.Ki = 0.005,
	.Kd = 0.410,
	.OutMax = 100,
	.OutMin = -100
};

PID_t Location_PID = 
{
	.Target = 0,
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.OutMax = 100,
	.OutMin = -100
};

int main(void)
{
	OLED_Init();
	LED_Init();
	Key_Init();
	RP_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	AD_Init();
	Timer_Init();
	while (1)
	{
		KeyNum = Key_GetNum();
		if (KeyNum == 2)
		{
			Location_PID.Target += 408;
			if (Location_PID.Target > 4080)
			{
				Location_PID.Target = 4080;
			}
		}
		if (KeyNum == 3)
		{
			Location_PID.Target -= 408;
			if (Location_PID.Target < -4080)
			{
				Location_PID.Target = -4080;
			}
		}
		
		if (KeyNum == 1)
		{
			if (RunState == 0)
			{
				RunState = 21;
			}
			else if (RunState == 4)
			{
				RunState = 0;
			}
			
		}
		if (RunState)
		{
			LED_ON();
		}
		else
		{
			LED_OFF();
		}
		OLED_Printf(42,0,OLED_6X8,"%02d",RunState);

		Location_PID.Kp = RP_GetValue(1) / 4095.0 * 1;
		Location_PID.Ki = RP_GetValue(2) / 4095.0 * 1;
		Location_PID.Kd = RP_GetValue(3) / 2048.0 * 1;
		OLED_Printf(0,0,OLED_6X8,"Angle");
		OLED_Printf(0,12,OLED_6X8,"Kp:%05.3f",Angle_PID.Kp);
		OLED_Printf(0,20,OLED_6X8,"Ki:%05.3f",Angle_PID.Ki);
		OLED_Printf(0,28,OLED_6X8,"Kd:%05.3f",Angle_PID.Kd);
		OLED_Printf(0,40,OLED_6X8,"Tar:%04.0f",Angle_PID.Target);
		OLED_Printf(0,48,OLED_6X8,"Act:%04.0f",Angle_PID.Actual);
		OLED_Printf(0,56,OLED_6X8,"Out:%+04.0f",Angle_PID.Out);

		OLED_Printf(64,0,OLED_6X8,"Location");
		OLED_Printf(64,12,OLED_6X8,"Kp:%05.3f",Location_PID.Kp);
		OLED_Printf(64,20,OLED_6X8,"Ki:%05.3f",Location_PID.Ki);
		OLED_Printf(64,28,OLED_6X8,"Kd:%05.3f",Location_PID.Kd);
		OLED_Printf(64,40,OLED_6X8,"Tar:%+04.0f",Location_PID.Target);
		OLED_Printf(64,48,OLED_6X8,"Act:%+04.0f",Location_PID.Actual);
		OLED_Printf(64,56,OLED_6X8,"Out:%+04.0f",Location_PID.Out);

		OLED_Update();
	}
}

void TIM1_UP_IRQHandler(void)
{	
	static uint16_t Count0,Count1,Count2,CountTime;
	static uint16_t Angle0,Angle1,Angle2;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		
		Key_Tick();
		Angle = AD_GetValue();
		Speed = Encoder_Get();
		Angle_PID.Actual = Angle;
		Location += Speed;
		Location_PID.Actual = Location;
		if (RunState == 0)
		{
			Angle_PID.ErrorInt = 0;
			Location_PID.ErrorInt = 0;
			Motor_SetPWM(0);
		}
		else if (RunState == 1)//判断摆杆位置，实现自动共振
		{
			Count0 ++;
			if (Count0 >= 40)
			{
				Count0 = 0;
				
				Angle2 = Angle1;
				Angle1 = Angle0;
				Angle0 = Angle;
				
				if (Angle0 > CENTER_ANGLE + CENTER_RANGE
				 && Angle1 > CENTER_ANGLE + CENTER_RANGE
				 && Angle2 > CENTER_ANGLE + CENTER_RANGE
				 && Angle1 < Angle0
				 && Angle1 < Angle2)
				{
					RunState = 21;
				}
				if (Angle0 < CENTER_ANGLE - CENTER_RANGE
				 && Angle1 < CENTER_ANGLE - CENTER_RANGE
				 && Angle2 < CENTER_ANGLE - CENTER_RANGE
				 && Angle1 > Angle0
				 && Angle1 > Angle2)
				{
					RunState = 31;
				}
				if (Angle0 > CENTER_ANGLE - CENTER_RANGE + 200
				 && Angle0 < CENTER_ANGLE + CENTER_RANGE - 200
				 && Angle1 > CENTER_ANGLE - CENTER_RANGE + 200
				 && Angle1 < CENTER_ANGLE + CENTER_RANGE - 200)
				{
					Location = 0;
					Angle_PID.ErrorInt = 0;
					Location_PID.ErrorInt = 0;
					RunState = 4;
				}
			}
		}

		//right
		else if (RunState == 21)
		{
			Motor_SetPWM(START_PWM);
			CountTime = START_TIME;
			RunState = 22;
		}
		else if (RunState == 22)
		{
			CountTime --;
			if (CountTime == 0)
			{
				RunState = 23;
			}
		}
		else if (RunState == 23)
		{
			Motor_SetPWM(-START_PWM);
			CountTime = START_TIME;
			RunState = 24;
		}
		else if (RunState == 24)
		{
			CountTime --;
			if (CountTime == 0)
			{
				Motor_SetPWM(0);
				RunState = 1;
			}
		}

		//left
		else if (RunState == 31)
		{
			Motor_SetPWM(-START_PWM);
			CountTime = START_TIME;
			RunState = 32;
		}
		else if (RunState == 32)
		{
			CountTime --;
			if (CountTime == 0)
			{
				RunState = 33;
			}
		}
		else if (RunState == 33)
		{
			Motor_SetPWM(START_PWM);
			CountTime = START_TIME;
			RunState = 34;
		}
		else if (RunState == 34)
		{
			CountTime --;
			if (CountTime == 0)
			{
				Motor_SetPWM(0);
				RunState = 1;
			}
		}

		
		if (RunState == 4)
		{
			if (!(Angle > CENTER_ANGLE - CENTER_RANGE 
			&& Angle < CENTER_ANGLE + CENTER_RANGE))
			{
			RunState = 0;
			}
			Count1 ++;
			Count2 ++;
			if (Count1 >= 5)
			{
				Count1 = 0;
				Angle_PID.Actual = Angle;
				PID_Update(&Angle_PID);
				Motor_SetPWM(-Angle_PID.Out);
			}
			if (Count2 >= 50)
			{
				Count2 = 0;
				Location_PID.Actual = Location;
				PID_Update(&Location_PID);
				Angle_PID.Target = CENTER_ANGLE - Location_PID.Out;
			}
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
