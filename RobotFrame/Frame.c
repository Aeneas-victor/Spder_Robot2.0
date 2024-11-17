/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Frame.c
  * @brief          : 框架定义文件
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024-aeneas
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE BEGIN Header */

#include "BlueSerial.h"
#include "Frame.h"
#include <string.h>
#include "Delay.h"
#include "Serial.h"
#include <stdio.h>
#include <stdlib.h>
#include "RobotServoController.h"
#include "Action.h"
#include "OLED.h"
#include "AsrProSerial.h"

extern char Bluetooch_RxPacket[100];//数据接收包
extern uint8_t Bluetooch_RxFlag;//数据标志位
extern char Asr_RxPacket[100];
enum SelectAction{
	Stop=0,
	Forward=1,
	Retreat=2,
	Rotation=3,
	Speed_High=4,
	Speed_Midium=5,
	Speed_Low=6,
	Posture_Midium=7,
	Posture_Low=8,
	Posture_High=9,
	Turn_Left=10,
	Turn_Right=11,
	Side_Left=12,
	Side_Right
	
}SelectAction;
enum CtrlMode
{
	BlueTooch=0,
	WiFi,
	Hander
}CtrlMode;
/**
  * @brief  action callback 函数指针枚举
  * @param  none
  * @retval none
  */
Action actionenum[14]=
{
	Move_Stop,Move_Advance,Move_Retreat,Move_Rotation,
	Set_Speed_High,Set_Speed_Midium,Set_Speed_Low,
	Set_Posture_Midium,Set_Posture_Low,Set_Posture_High,
	Move_Left_Turn,Move_Right_Turn,Move_Side_Left,Move_Side_Right
};
/**
  * @brief  HexapodRobot 初始化函数 包括模块的初始化函数
  * @param  Robot_Positure_Init   动作初始化函数，传入需要时初始化是表现得动作
  * @retval  none
  */
void HexapodRobot_Init(InitFunc Robot_Positure_Init)
{ 
#define refresh
	for(uint8_t i=0;i<18;i++)
	{
		Robot_Posture_Begin[i]=Robot_Low_Start[i];
	}
	OLED_Init();
	Delay_Init();
	Robot_Serial_Init();
	Robot_Positure_Init();
}
/**
  * @brief  框架入口函数
  * @param  none
  * @retval  none
  */

void FrameEntry()
{
	HexapodRobot_Init(Robot_Action);
	Delay_Ms(2);
	//OLED_ShowString(1,1,"welcome");
	while(1)
	{
		
		//CtrlEntry(BluetoochMode);
		CtrlEntry(TestMode);
		//test();
	}
}
/**
  * @brief  控制模式入口函数
  * @param  传入控制的模式函数
  * @retval  none
  */
void CtrlEntry(CallBackFunc CtrlMode)
{
	CtrlMode();
}
/**
  * @brief  Action
* @param  action:函数数组索引
  * @retval  none
  */
void ActionCallBack(uint8_t action)
{
	actionenum[action]();
}
/**
  * @brief  蓝牙模式
  * @param  none
  * @retval  none
  */
void BluetoochMode(void)
{
	Blue_Serial_Init();
	uint8_t actnumber;
	while(1)
	{
		if(Bluetooch_RxFlag==1)
		{
			actnumber=atoi(Bluetooch_RxPacket);
		}
		ActionCallBack(actnumber);
	}
}
/**
  * @brief  测试函数
  * @param  void
  * @retval  none
  */
void TestMode(void)
{
	AsrPro_Init();
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	GPIO_InitTypeDef teststr;
	teststr.GPIO_Mode=GPIO_Mode_OUT;
	teststr.GPIO_OType=GPIO_OType_PP;
	teststr.GPIO_Pin=GPIO_Pin_8;
	teststr.GPIO_PuPd=GPIO_PuPd_NOPULL;
	teststr.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOG,&teststr);
	//OLED_ShowString(1,1,"TestMode");
	while(1)
	{
		
	}
}

/**
  * @brief  步态选择函数
  * @param  none
  * @retval  none
  */
void SwitchGait()
{
	
	return;
}

