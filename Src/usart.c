/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */



#include "usart.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��os,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//os ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32H7������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2017/6/8
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.0�޸�˵�� 
////////////////////////////////////////////////////////////////////////////////// 	  
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
//#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
//int fputc(int ch, FILE *f)
//{ 	
//	while((USART1->ISR&0X40)==0);//ѭ������,ֱ���������   
//	USART1->TDR=(u8)ch;      
//	return ch;
//}
#endif 

#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

u8 aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���
UART_HandleTypeDef UART1_Handler; //UART���

//��ʼ��IO ����1 
//bound:������
//void uart_init(u32 bound)
//{	
//	//UART ��ʼ������
//	UART1_Handler.Instance=USART1;					    //USART1
//	UART1_Handler.Init.BaudRate=bound;				    //������
//	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
//	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
//	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
//	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
//	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
//	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()��ʹ��UART1
//	
//	HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
//  
//}

//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��

//void HAL_UART_MspInit(UART_HandleTypeDef *huart)
//{
//    //GPIO�˿�����
//	GPIO_InitTypeDef GPIO_Initure;
//	
//	if(huart->Instance==USART1)//����Ǵ���1�����д���1 MSP��ʼ��
//	{
//		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
//		__HAL_RCC_USART1_CLK_ENABLE();			//ʹ��USART1ʱ��
//	
//		GPIO_Initure.Pin=GPIO_PIN_2;			//PA9
//		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
//		GPIO_Initure.Pull=GPIO_PULLUP;			//����
//		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
//		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//����ΪUSART1
//		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA9

//		GPIO_Initure.Pin=GPIO_PIN_3;			//PA10
//		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA10
//		
//#if EN_USART1_RX
//		HAL_NVIC_EnableIRQ(USART1_IRQn);				//ʹ��USART1�ж�ͨ��
//		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
//#endif	
//	}

//}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{

//	if(huart->Instance==USART2)//����Ǵ���1
//	{
//		if(aRxBuffer[0]==0x23)USART_RX_STA|=0x8000;	//��������� 
//		if(aRxBuffer[0]==0x06)waveFlag=1;
//		if((USART_RX_STA&0x8000)==0)//����δ���
//		{
//			if(USART_RX_STA&0x4000)//���յ���0x0d
//			{
//				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
//				else USART_RX_STA=0;//USART_RX_STA|=0x8000;	//��������� 
//			}
//			else //��û�յ�0X0D
//			{	
//				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
//				}		 
//			}
//			
//		}
//		
//		
////		USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
////		if(aRxBuffer[0]==0x23)
////		{
////			USART_RX_STA|=0x8000;
////			USART_RX_STA=0;
////		}
////		USART_RX_STA++;
//		//printf("hello");
//		

//	}
//	if(huart->Instance == USART1)
////		{if(Uart3_Rx_Cnt >= 255)  //����ж�
////		{
////			Uart3_Rx_Cnt = 0;
////			memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff));
////			HAL_UART_Transmit(&huart3, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);	
////		}
////		else
////		{
////			Uart3_RxBuff[Uart3_Rx_Cnt++] = aRxBuffer3;   //��������ת��
////				//HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
////			if(aRxBuffer3==0x06){	printf_flag=0;
////									HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
////									for (int i=0;i<1024;i++){printf("%f\t%f\r\n",adc1[i],adc0[i]);}
////									printf("���ƶȣ�%f\t©��ֵ1��%f\t©��ֵ0��%f\tͻ��ֵ��%i\r\n",cos0,I1/530,I0/530,har/530);
////									printf("©��ʱ�䣺\r\n");
////									printf("%02d/%02d/%02d\r\n",2000 + year, month, date);
////									printf("%02d:%02d:%02d\r\n",hour, minute, second);
////									printf("��ǰʱ�䣺\r\n");
////									HAL_RTC_GetTime(&hrtc, &stimestruct, RTC_FORMAT_BIN);
////									HAL_RTC_GetDate(&hrtc, &sdatestruct, RTC_FORMAT_BIN);
////									printf("%02d/%02d/%02d\r\n",2000 + sdatestruct.Year, sdatestruct.Month, sdatestruct.Date);
////									printf("%02d:%02d:%02d\r\n",stimestruct.Hours, stimestruct.Minutes, stimestruct.Seconds);
////									
////									HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_RESET);
////									printf_flag=1;
////									}
////			if((Uart3_RxBuff[Uart3_Rx_Cnt-1] == 0x0A)&&(Uart3_RxBuff[Uart3_Rx_Cnt-2] == 0x0D)) //�жϽ���λ
////			{
//////											HAL_UART_Transmit(&huart2, (uint8_t *)&Uart3_RxBuff, Uart3_Rx_Cnt,0xFFFF);//��485�����յ�����Ϣ���͵�����2
////				Uart3_Rx_Cnt = 0;
////				memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff)); //�������
////			}
////		}
////		HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1);}
//		{
//			//uartFlag=1;
//			if(aRxBuffer[0]==0x23)USART_RX_STA|=0x8000;	//���������
//			if(aRxBuffer[0]==0x06)waveFlag=1;
//		if((USART_RX_STA&0x8000)==0)//����δ���
//		{
//			if(USART_RX_STA&0x4000)//���յ���0x0d
//			{
//				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
//				else USART_RX_STA=0;//USART_RX_STA|=0x8000;	//��������� 
//			}
//			else //��û�յ�0X0D
//			{	
//				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
//				}		 
//			}
//			
//		}
//		HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
//		}
//}
// 
////����1�жϷ������
void USART2_IRQHandler(void)                	
{ 
	u32 timeout=0;
    u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntEnter();    
#endif
	
	HAL_UART_IRQHandler(&huart2);	//����HAL���жϴ����ú���
	
	timeout=0;
    while (HAL_UART_GetState(&huart2)!=HAL_UART_STATE_READY)//�ȴ�����
	{
        timeout++;////��ʱ����
        if(timeout>maxDelay) break;		
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer, RXBUFFERSIZE)!=HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
        timeout++; //��ʱ����
        if(timeout>maxDelay) break;	
	}
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntExit();  											 
#endif
} 
#endif	

/*�����������ֱ�Ӱ��жϿ����߼�д���жϷ������ڲ���*/
/*

//����1�жϷ������
void USART1_IRQHandler(void)                	
{ 
	u8 Res;
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntEnter();    
#endif
	if((__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_RXNE)!=RESET))  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
        HAL_UART_Receive(&UART1_Handler,&Res,1,1000); 
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
	}
	HAL_UART_IRQHandler(&UART1_Handler);	
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntExit();  											 
#endif
} 
#endif	
*/











/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
