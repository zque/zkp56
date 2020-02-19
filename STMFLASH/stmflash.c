#include "stmflash.h"
#include "delay.h"
#include "usart.h" 
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

//����STM32��FLASH
void STMFLASH_Unlock(void)
{
	FLASH->KEYR1=FLASH_KEY1;	//Bank1,д���������.
	FLASH->KEYR1=FLASH_KEY2; 
	
	FLASH->KEYR2=FLASH_KEY1;	//Bank2,д���������.
	FLASH->KEYR2=FLASH_KEY2; 
}
//flash����
void STMFLASH_Lock(void)
{
	FLASH->CR1|=1<<0;	//Bank1,����
	FLASH->CR2|=1<<0;	//Bank2,����
}
//�õ�FLASH�Ĵ���״̬
//bankx:0,��ȡbank1��״̬
//      1,��ȡbank2��״̬
//����ֵ:
//0,�޴���
//����,������
u8 STMFLASH_GetErrorStatus(u8 bankx)
{	
	u32 res=0;	
	if(bankx==0)res=FLASH->SR1;
	else res=FLASH->SR2; 
	if(res&(1<<17))return 1;   		//WRPERR=1,д��������
	else if(res&(1<<18))return 2;	//PGSERR=1,������д���
	else if(res&(1<<19))return 3;	//STRBERR=1,��д���� 
	else if(res&(1<<21))return 4;	//INCERR=1,����һ���Դ���
	else if(res&(1<<22))return 5;	//OPERR=1,д/�������� 
	else if(res&(1<<23))return 6;	//RDPERR=1,����������
	else if(res&(1<<24))return 7;	//RDSERR=1,�Ƿ����ʼ��������� 
	else if(res&(1<<25))return 8;	//SNECCERR=1,1bit eccУ������ 
	else if(res&(1<<26))return 9;	//DBECCERR=1,2bit ecc����
	return 0;						//û���κ�״̬/�������.
} 
//�ȴ��������
//bankx:0,bank1; 1,bank2
//time:Ҫ��ʱ�ĳ���(��λ:10us)
//����ֵ:
//0,���      
//1~9,�������.
//0XFF,��ʱ
u8 STMFLASH_WaitDone(u8 bankx,u32 time)
{
	u8 res=0;
	u32 tempreg=0; 
	while(1)
	{
		if(bankx==0)tempreg=FLASH->SR1;
		else tempreg=FLASH->SR2;
		if((tempreg&0X07)==0)break;			//BSY=0,WBNE=0,QW=0,��������
		delay_us(10);
		time--;
		if(time==0)return 0XFF;
	}
	res=STMFLASH_GetErrorStatus(bankx);
	if(res)
	{
		if(bankx==0)FLASH->CCR1=0X07EE0000;	//�����д����־
		else FLASH->CCR2=0X07EE0000; 		//�����д����־
		
	}
	return res;
}
//��������
//sectoraddr:������ַ,��Χ��:0~15.
//0~7,addr���ڵ�bank1�������
//8~15,addr���ڵ�bank2�������+8,��Ҫ��ȥ8,�ŵ�bank2�������
//����ֵ:ִ�����
u8 STMFLASH_EraseSector(u8 sectornum)
{
	u8 res=0;
	res=STMFLASH_WaitDone(sectornum/8,200000);	//�ȴ��ϴβ�������,���2s    
	if(res==0)
	{  
		if(sectornum<8)	//BANK1	����
		{
			FLASH->CR1&=~(7<<8);	//SNB1[2:0]=0,���ԭ��������
			FLASH->CR1&=~(3<<4);	//PSIZE1[1:0]=0,���ԭ��������
			FLASH->CR1|=(u32)sectornum<<8;	//����Ҫ�������������,0~7
			FLASH->CR1|=2<<4;		//����Ϊ32bit��
			FLASH->CR1|=1<<2;		//SER1=1,�������� 
			FLASH->CR1|=1<<7;		//START1=1,��ʼ����
		}else			//BANK2 ����
		{
			FLASH->CR2&=~(7<<8);	//SNB2[2:0]=0,���ԭ��������
			FLASH->CR2&=~(3<<4);	//PSIZE2[1:0]=0,���ԭ��������
			FLASH->CR2|=(u32)(sectornum-8)<<8;//����Ҫ�������������,0~7
			FLASH->CR2|=2<<4;		//����Ϊ32bit��
			FLASH->CR2|=1<<2;		//SER2=1,�������� 
			FLASH->CR2|=1<<7;		//START2=1,��ʼ����
		} 
		res=STMFLASH_WaitDone(sectornum/8,200000);	//�ȴ���������,���2s  
		if(sectornum<8)FLASH->CR1&=~(1<<2);	//SER1=0,�������������־
		else FLASH->CR2&=~(1<<2);			//SER2=0,�������������־ 
	}
	return res;
}
//��FLASHָ����ַд8����,��256bit
//������256bitΪ��λ���!!
//faddr:ָ����ַ(�˵�ַ����Ϊ4�ı���!!)
//dat:Ҫд�������
//����ֵ:0,д��ɹ�
//    ����,д��ʧ��
u8 STMFLASH_Write8Word(u32 faddr, u32* pdata)
{
	u8 nword=8;	//ÿ��д8����,256bit
	u8 res;
	u8 bankx=0;
	if(faddr<BANK2_FLASH_SECTOR_0)bankx=0;	//�жϵ�ַ����bank0,������bank1
	else bankx=1;
	res=STMFLASH_WaitDone(bankx,0XFF);	 
	if(res==0)	//OK
	{
		if(bankx==0)	//BANK1	���
		{ 
			FLASH->CR1&=~(3<<4);	//PSIZE1[1:0]=0,���ԭ�������� 
			FLASH->CR1|=2<<4;		//����Ϊ32bit��
			FLASH->CR1|=1<<1;		//PG1=1,���ʹ�� 
		}else			//BANK2 ���
		{
			FLASH->CR2&=~(3<<4);	//PSIZE2[1:0]=0,���ԭ�������� 
			FLASH->CR2|=2<<4;		//����Ϊ32bit��
			FLASH->CR2|=1<<1;		//PG2=1,���ʹ�� 
		} 
		while(nword)
		{
			*(vu32*)faddr=*pdata;	//д������
			faddr+=4;				//д��ַ+4
			pdata++;				//ƫ�Ƶ���һ�������׵�ַ
			nword--;
		}
		__DSB();					//д������ɺ�,��������ͬ��,ʹCPU����ִ��ָ������
		res=STMFLASH_WaitDone(bankx,0XFF);	//�ȴ��������,һ���ֱ��,���100us.

		if(bankx==0)FLASH->CR1&=~(1<<1);//PG1=0,�������������־
		else FLASH->CR2&=~(1<<1);		//PG2=0,�������������־
	} 
	return res;
} 
//��ȡָ����ַ��һ����(32λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~7,addr���ڵ�bank1�������
//      8~15,addr���ڵ�bank2�������+8,��Ҫ��ȥ8,�ŵ�bank2�������
u8 STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<BANK1_FLASH_SECTOR_1)return 0;
	else if(addr<BANK1_FLASH_SECTOR_2)return 1;
	else if(addr<BANK1_FLASH_SECTOR_3)return 2;
	else if(addr<BANK1_FLASH_SECTOR_4)return 3;
	else if(addr<BANK1_FLASH_SECTOR_5)return 4;
	else if(addr<BANK1_FLASH_SECTOR_6)return 5;
	else if(addr<BANK1_FLASH_SECTOR_7)return 6; 
	else if(addr<BANK2_FLASH_SECTOR_0)return 7;
	else if(addr<BANK2_FLASH_SECTOR_1)return 8;
	else if(addr<BANK2_FLASH_SECTOR_2)return 9;
	else if(addr<BANK2_FLASH_SECTOR_3)return 10; 
	else if(addr<BANK2_FLASH_SECTOR_4)return 11;
	else if(addr<BANK2_FLASH_SECTOR_5)return 12;
	else if(addr<BANK2_FLASH_SECTOR_6)return 13; 
	else if(addr<BANK2_FLASH_SECTOR_7)return 14;    
	return 15;	
}
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32H7������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ32�ı���!!,����д�����!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���,һ������д��32�ֽ�,��8����) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
	u8 status=0;
	u32 addrx=0;
	u32 endaddr=0;	
  	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%32)return;	//�Ƿ���ַ
	STMFLASH_Unlock();									//���� 
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FF00000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=STMFLASH_EraseSector(STMFLASH_GetFlashSector(addrx));
				if(status)break;	//���������� 
                SCB_CleanInvalidateDCache();	//�����Ч��D-Cache
			}else addrx+=4;
		} 
	}
	if(status==0)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(STMFLASH_Write8Word(WriteAddr,pBuffer))//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=32;
			pBuffer+=8;
		} 
	}
	STMFLASH_Lock();//����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(32λ)��
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}

//////////////////////////////////////////������///////////////////////////////////////////
//WriteAddr:��ʼ��ַ
//WriteData:Ҫд�������
void Test_Write(u32 WriteAddr,u32 WriteData)   	
{
	STMFLASH_Write(WriteAddr,&WriteData,1);//д��һ���� 
}
 













