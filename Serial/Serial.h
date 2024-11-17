/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Serial.h
  * @brief          : Serial 定义声明
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
 
#ifndef __SERIAL_H__
#define __SERIAL_H__
 
#include "stm32f4xx.h"
 
 
#define USART_RCC       	    RCC_APB2Periph_USART1
#define USART_TX_GPIO_RCC       RCC_AHB1Periph_GPIOA
#define USART_RX_GPIO_RCC       RCC_AHB1Periph_GPIOA
 
#define USARTx				    USART1
#define USART_PORT              GPIOA
#define USART_TX_PORT           GPIOA
#define USART_TX_PIN            GPIO_Pin_9
#define USART_RX_PORT           GPIOA
#define USART_RX_PIN            GPIO_Pin_10
#define RobotUSART_BaudRate     9600
#define USART_AF                GPIO_AF_USART1
#define USART_TX_AF_PIN		    GPIO_PinSource9
#define USART_RX_AF_PIN		    GPIO_PinSource10
 

void Robot_Serial_Init(void);
void Robot_Send_Byte(uint8_t data);
void Robot_Send_String(uint8_t *ucstr);
void Robot_SendArray(uint8_t* Array,uint16_t Length);




#endif
