#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"   
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32H7������
//STM32�ڲ�FLASH��д ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2018/7/18
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved				
//********************************************************************************
//�޸�˵�� 
//20180816
//1,��������ע��.
//2,��Ϊд���ַ����Ϊ32�ı���!����д�뽫����
////////////////////////////////////////////////////////////////////////////////// 

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ
//FLASH������ֵ
//#define FLASH_KEY1               0X45670123
//#define FLASH_KEY2               0XCDEF89AB

//FLASH ��������ʼ��ַ,��2��bank,ÿ��bank 1MB
#define BANK1_FLASH_SECTOR_0     ((u32)0x08000000) 	//Bank1����0��ʼ��ַ, 128 Kbytes  
#define BANK1_FLASH_SECTOR_1     ((u32)0x08020000) 	//Bank1����1��ʼ��ַ, 128 Kbytes  
#define BANK1_FLASH_SECTOR_2     ((u32)0x08040000) 	//Bank1����2��ʼ��ַ, 128 Kbytes  
#define BANK1_FLASH_SECTOR_3     ((u32)0x08060000) 	//Bank1����3��ʼ��ַ, 128 Kbytes  
#define BANK1_FLASH_SECTOR_4     ((u32)0x08080000) 	//Bank1����4��ʼ��ַ, 128 Kbytes  
#define BANK1_FLASH_SECTOR_5     ((u32)0x080A0000) 	//Bank1����5��ʼ��ַ, 128 Kbytes  
#define BANK1_FLASH_SECTOR_6     ((u32)0x080C0000) 	//Bank1����6��ʼ��ַ, 128 Kbytes  
#define BANK1_FLASH_SECTOR_7     ((u32)0x080E0000) 	//Bank1����7��ʼ��ַ, 128 Kbytes 
#define BANK2_FLASH_SECTOR_0     ((u32)0x08100000) 	//Bank2����0��ʼ��ַ, 128 Kbytes  
#define BANK2_FLASH_SECTOR_1     ((u32)0x08120000) 	//Bank2����1��ʼ��ַ, 128 Kbytes  
#define BANK2_FLASH_SECTOR_2     ((u32)0x08140000) 	//Bank2����2��ʼ��ַ, 128 Kbytes  
#define BANK2_FLASH_SECTOR_3     ((u32)0x08160000) 	//Bank2����3��ʼ��ַ, 128 Kbytes  
#define BANK2_FLASH_SECTOR_4     ((u32)0x08180000) 	//Bank2����4��ʼ��ַ, 128 Kbytes  
#define BANK2_FLASH_SECTOR_5     ((u32)0x081A0000) 	//Bank2����5��ʼ��ַ, 128 Kbytes  
#define BANK2_FLASH_SECTOR_6     ((u32)0x081C0000) 	//Bank2����6��ʼ��ַ, 128 Kbytes  
#define BANK2_FLASH_SECTOR_7     ((u32)0x081E0000)  //Bank2����7��ʼ��ַ, 128 Kbytes    

 
 
void STMFLASH_Unlock(void);						//FLASH����
void STMFLASH_Lock(void);				 		//FLASH����
u8 STMFLASH_GetStatus(u8 bankx);				//���״̬
u8 STMFLASH_WaitDone(u8 bankx,u32 time); 		//�ȴ���������
u8 STMFLASH_EraseSector(u8 sectornum);	 		//����ҳ
u8 STMFLASH_Write8Word(u32 faddr, u32* pdata);	//һ��д��8����
u32 STMFLASH_ReadWord(u32 faddr);				//��һ����
u8 STMFLASH_GetFlashSector(u32 addr);			//��ȡ������� 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);	//ָ����ַ��ʼд��ָ�����ȵ����� 
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);	//��ָ����ַ��ʼ����ָ�����ȵ�����

//����д��
void Test_Write(u32 WriteAddr,u32 WriteData);								   
#endif

















