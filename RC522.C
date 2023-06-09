/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-10     35166       the first version
 */
#include <rtthread.h>
#include "board.h"
#include <rtdevice.h>
#include "RC522.h"
#include <drv_common.h> 
 
#define DBG_TAG "RC522"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include <drv_lcd.h>


 
/**********�߳����ȼ�*********/
#define RC522_THREAD_PRIORITY     15
 
// M1����Ϊ16��������ÿ���������ĸ��飨��0����1����2����3�����
// ��16��������64���鰴���Ե�ַ���Ϊ��0~63
// ��0�������Ŀ�0�������Ե�ַ0�飩�����ڴ�ų��̴��룬�Ѿ��̻����ɸ���
// ÿ�������Ŀ�0����1����2Ϊ���ݿ飬�����ڴ������
// ÿ�������Ŀ�3Ϊ���ƿ飨���Ե�ַΪ:��3����7����11.....����������A����ȡ���ơ�����B��
 
/*******************************
*����˵����
*1--SDA  <----->PA4
*2--SCK  <----->PA5
*3--MOSI <----->PA7
*4--MISO <----->PA6
*5--����
*6--GND <----->GND
*7--RST <----->PC4
*8--VCC <----->VCC
************************************/
#define MAXRLEN 18
 
/*
 * ��������PcdInit
 * ����  �����ų�ʼ��
 * ����  : ״ֵ̬
 *         = MI_OK���ɹ�
 */
void PcdInit()
{
    rt_pin_mode(MF522_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MF522_MISO_PIN, PIN_MODE_INPUT);
    rt_pin_mode(MF522_MOSI_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MF522_SCK_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MF522_NSS_PIN, PIN_MODE_OUTPUT);
}
 
/*
 * ��������PcdRequest
 * ����  ��Ѱ��
 * ����  ��req_code��Ѱ����ʽ
 *                     = 0x52��Ѱ��Ӧ�������з���14443A��׼�Ŀ�
 *                     = 0x26��Ѱδ��������״̬�Ŀ�
 *         pTagType����Ƭ���ʹ���
 *         = MI_OK���ɹ�
 * ����  ���ⲿ����
 */
char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
   char status;
   unsigned int  unLen;
   unsigned char ucComMF522Buf[MAXRLEN];
   ClearBitMask(Status2Reg,0x08);
   WriteRawRC(BitFramingReg,0x07);
 
   SetBitMask(TxControlReg,0x03);
 
   ucComMF522Buf[0] = req_code;
 
   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
 
   if ((status == MI_OK) && (unLen == 0x10))
   {
       *pTagType     = ucComMF522Buf[0];
       *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {   status = MI_ERR;   }
 
   return status;
}
 
/*
 * ��������PcdAnticoll
 * ����  ������ײ
 * ����  ��pSnr����Ƭ���кţ�4�ֽ�
 * ����  : ״ֵ̬
 *         = MI_OK���ɹ�
 * ����  ���ⲿ����
 */
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];
 
 
    ClearBitMask(Status2Reg,0x08);
    WriteRawRC(BitFramingReg,0x00);
    ClearBitMask(CollReg,0x80);
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);
 
    if (status == MI_OK)
    {
         for (i=0; i<4; i++)
         {
             *(pSnr+i)  = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])
         {   status = MI_ERR;    }
    }
 
    SetBitMask(CollReg,0x80);
    return status;
}
 
/*
 * ��������PcdSelect
 * ����  ��ѡ����Ƭ
 * ����  ��pSnr����Ƭ���кţ�4�ֽ�
 * ����  : ״ֵ̬
 *         = MI_OK���ɹ�
 * ����  ���ⲿ����
 */
char PcdSelect(unsigned char *pSnr)
{
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
        ucComMF522Buf[i+2] = *(pSnr+i);
        ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
 
    ClearBitMask(Status2Reg,0x08);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
 
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }
 
    return status;
}
 
/*
 * ��������PcdAuthState
 * ����  ����֤��Ƭ����
 * ����  ��auth_mode��������֤ģʽ
 *                     = 0x60����֤A��Կ
 *                     = 0x61����֤B��Կ
 *         u8 ucAddr�����ַ
 *         pKey������
 *         pSnr����Ƭ���кţ�4�ֽ�
 * ����  : ״ֵ̬
 *         = MI_OK���ɹ�
 * ����  ���ⲿ����
 */
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];
 
    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
 
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
 
    return status;
}
 
/*
 * ��������PcdRead
 * ����  ����ȡM1��һ������
 * ����  ��u8 addr�����ַ
 *         pData�����������ݣ�16�ֽ�
 * ����  : ״ֵ̬
 *         = MI_OK���ɹ�
 * ����  ���ⲿ����
 */
char PcdRead(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];
 
    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }
 
    return status;
}
 
/*
 * ��������PcdWrite
 * ����  ��д���ݵ�M1��һ��
 * ����  ��u8 addr�����ַ
 *         pData��д������ݣ�16�ֽ�
 * ����  : ״ֵ̬
 *         = MI_OK���ɹ�
 * ����  ���ⲿ����
 */
char PcdWrite(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];
 
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
 
    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
 
    if (status == MI_OK)
    {
 
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);
 
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
 
    return status;
}
 
/*
 * ��������PcdSpicelWrite
 * ����  ��д���ݵ�M1��һ��
 * ����  ��u8 addr�����ַ
 *        pData��д������ݣ�16�ֽ�
 * ����  : ״ֵ̬
 *         = MI_OK���ɹ�
 * ����  ���ⲿ����
 */
char PcdSpicelWrite(unsigned char addr, unsigned char *pData)
{
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];
 
    //��Ҫʹ�����²��迪������
    PcdHalt();
    WriteRawRC(BitFramingReg, 0x07);
    ucComMF522Buf[0] = 0x40;
    PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
 
    WriteRawRC(BitFramingReg, 0x00);
    ucComMF522Buf[0] = 0x43;
    PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
 
    return PcdWrite(addr, pData);
}
 
/*
 * ��������PcdHalt
 * ����  �����Ƭ��������״̬
 * ����  ����
 * ����  : ״ֵ̬
 *         = MI_OK���ɹ�
 * ����  ���ⲿ����
 */
char PcdHalt(void)
{
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];
 
    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
 
    return MI_OK;
}
 
/*
 * ��������CalulateCRC
 * ����  ����RC522����CRC16
 * ����  ��    pIndata������CRC16������
 *         len������CRC16�������ֽڳ���
 *         pOutData����ż�������ŵ��׵�ַ
 * ����  : ��
 * ����  ���ڲ�����
 */
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   WriteRawRC(FIFODataReg, *(pIndata+i));   }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
}
 
/*
 * ��������PcdRese
 * ����  ����λRC522
 * ����  ����
 * ����  : ��
 * ����  ���ⲿ����
 */
char PcdReset(void)
{
    RST_H;
    rt_thread_mdelay(10);
    RST_L;
    rt_thread_mdelay(10);
    RST_H;
    rt_thread_mdelay(100);
 
    if(ReadRawRC(0x02) == 0x80)
    {
    }
 
    WriteRawRC(CommandReg,PCD_RESETPHASE);
 
    WriteRawRC(ModeReg,0x3D);
    WriteRawRC(TReloadRegL,30);
    WriteRawRC(TReloadRegH,0);
    WriteRawRC(TModeReg,0x8D);
    WriteRawRC(TPrescalerReg,0x3E);
    WriteRawRC(TxAutoReg,0x40);
    return MI_OK;
}
 
/*
 * ��������M500PcdConfigISOType
 * ����  ������RC522�Ĺ�����ʽ
 * ����  ��ucType��������ʽ
 * ����  : ��
 * ����  ���ⲿ����
 */
char M500PcdConfigISOType(unsigned char type)
{
   if (type == 'A')
   {
      ClearBitMask(Status2Reg,0x08);
      WriteRawRC(ModeReg,0x3D);//3F
      WriteRawRC(RxSelReg,0x86);//84
      WriteRawRC(RFCfgReg,0x7F);   //4F
      WriteRawRC(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec)
      WriteRawRC(TReloadRegH,0);
      WriteRawRC(TModeReg,0x8D);
      WriteRawRC(TPrescalerReg,0x3E);
      rt_thread_mdelay(10);
      PcdAntennaOn();
   }
   else{ return (char)-1; }
   return MI_OK;
}
/*
 * ��������ReadRawRC
 * ����  ����RC522�Ĵ���
 * ����  ��ucAddress���Ĵ�����ַ
 * ����  : �Ĵ����ĵ�ǰֵ
 * ����  ���ڲ�����
 */
unsigned char ReadRawRC(unsigned char Address)
{
     unsigned char i, ucAddr;
     unsigned char ucResult=0;
     NSS_L;
     ucAddr = ((Address<<1)&0x7E)|0x80;
     for(i=8;i>0;i--)
     {
         SCK_L;
         if(ucAddr&0x80)
            MOSI_H;
         else
            MOSI_L;
         SCK_H;
         ucAddr <<= 1;
     }
     for(i=8;i>0;i--)
     {
         SCK_L;
         ucResult <<= 1;
         SCK_H;
         if(READ_MISO == 1)
            ucResult |= 1;
     }
     NSS_H;
     SCK_H;
     return ucResult;
}
/*
 * ��������WriteRawRC
 * ����  ��дRC522�Ĵ���
 * ����  ��ucAddress���Ĵ�����ַ
 *         ucValue��д��Ĵ�����ֵ
 * ����  : ��
 * ����  ���ڲ�����
 */
void WriteRawRC(unsigned char Address, unsigned char value)
{
    unsigned char i, ucAddr;
    SCK_L;
    NSS_L;
    ucAddr = ((Address<<1)&0x7E);
    for(i=8;i>0;i--)
    {
        if(ucAddr&0x80)
            MOSI_H;
        else
            MOSI_L;
        SCK_H;
        ucAddr <<= 1;
        SCK_L;
    }
    for(i=8;i>0;i--)
    {
        if(value&0x80)
            MOSI_H;
        else
            MOSI_L;
        SCK_H;
        value <<= 1;
        SCK_L;
    }
    NSS_H;
    SCK_H;
}
/*
 * ��������SetBitMask
 * ����  ����RC522�Ĵ�����λ
 * ����  ��ucReg���Ĵ�����ַ
 *         ucMask����λֵ
 * ����  : ��
 * ����  ���ڲ�����
 */
void SetBitMask(unsigned char reg,unsigned char mask)
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
}
/*
 * ��������ClearBitMask
 * ����  ����RC522�Ĵ�����λ
 * ����  ��ucReg���Ĵ�����ַ
 *         ucMask����λֵ
 * ����  : ��
 * ����  ���ڲ�����
 */
void ClearBitMask(unsigned char reg,unsigned char mask)
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
}
/*
 * ��������PcdComMF522
 * ����  ��ͨ��RC522��ISO14443��ͨѶ
 * ����  ��Command��RC522������
 *         pInData��ͨ��RC522���͵���Ƭ������
 *         InLenByte���������ݵ��ֽڳ���
 *         pOutData�����յ��Ŀ�Ƭ��������
 *         pOutLenBit���������ݵ�λ����
 * ����  : ״ֵ̬
 *         = MI_OK���ɹ�
 * ����  ���ڲ�����
 */
char PcdComMF522(unsigned char Command,
                 unsigned char *pInData,
                 unsigned char InLenByte,
                 unsigned char *pOutData,
                 unsigned int  *pOutLenBit)
{
    char status = MI_ERR;
    unsigned char irqEn   = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
    switch (Command)
    {
       case PCD_AUTHENT:
          irqEn   = 0x12;
          waitFor = 0x10;
          break;
       case PCD_TRANSCEIVE:
       case PCD_SPECIAL_COPY:
          irqEn   = 0x77;
          waitFor = 0x30;
          break;
       default:
         break;
    }
    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<InLenByte; i++)
    {   WriteRawRC(FIFODataReg, pInData[i]);    }
    WriteRawRC(CommandReg, Command);
    if (Command == PCD_TRANSCEIVE)
    {    SetBitMask(BitFramingReg,0x80);  }
 i = 2000;
    do
    {
         n = ReadRawRC(ComIrqReg);
         i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));
    ClearBitMask(BitFramingReg,0x80);
    if (i!=0)
    {
         if(!(ReadRawRC(ErrorReg)&0x1B))
         {
             status = MI_OK;
             if (n & irqEn & 0x01)
             {   status = MI_NOTAGERR;   }
             if (Command == PCD_TRANSCEIVE)
             {
                n = ReadRawRC(FIFOLevelReg);
                lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {   *pOutLenBit = (n-1)*8 + lastBits;   }
                else
                {   *pOutLenBit = n*8;   }
                if (n == 0)
                {   n = 1;    }
                if (n > MAXRLEN)
                {   n = MAXRLEN;   }
                for (i=0; i<n; i++)
                {   pOutData[i] = ReadRawRC(FIFODataReg);    }
            }
         }
         else
         {   status = MI_ERR;   }
   }
   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE);
   return status;
}
/*
 * ��������PcdAntennaOn
 * ����  ����������
 * ����  ����
 * ����  : ��
 * ����  ���ڲ�����
 */
void PcdAntennaOn()
{
    unsigned char i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}
/*
 * ��������PcdAntennaOff
 * ����  ����������
 * ����  ����
 * ����  : ��
 * ����  ���ڲ�����
 */
void PcdAntennaOff()
{
    ClearBitMask(TxControlReg, 0x03);
}
void WaitCardOff(void)
{
    char          status;
  unsigned char TagType[2];
    while(1)
    {
        status = PcdRequest(REQ_ALL, TagType);
        if(status)
        {
            status = PcdRequest(REQ_ALL, TagType);
            if(status)
            {
                status = PcdRequest(REQ_ALL, TagType);
                if(status)
                {
                    return;
                }
            }
        }
        rt_thread_mdelay(100);
    }
}

static void draw_line(void)
{
	int i=0;
	for(i=0;i<241;i++)
	{
		lcd_draw_point(i, 10+32+32+10);
	}
	
}



static void RC522_thread_entry(void *parameter)
{
	  int flag=0;
	  int count=1,i;
	  uint16_t rightmima[50][4]={0x3a,0xf1,0xdf,0x80};
	  uint16_t mima[4];
    char status;
    unsigned char buf[16];                                               //���ݻ���
    unsigned char TagType[2];                                            //��Ƭ����
    unsigned char SelectedSnr[4];                                          //��Ƭ���к�
//    unsigned char DefaultKey1[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};     //��Ĭ������
    unsigned char DefaultKey2[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};     //��Ĭ������
    unsigned char inDoor[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};                                                                  //д�������
    PcdInit();
    PcdReset();
    PcdAntennaOff();
    PcdAntennaOn();
    M500PcdConfigISOType( 'A' );
    rt_kprintf( "rc522 init over!\n" );
    while(1)
    {
        status= PcdRequest( REQ_ALL , TagType );    //Ѱ��
        if(status == MI_OK)
        {
            status = PcdAnticoll(SelectedSnr);      //����ײ
            if(status == MI_OK)
            {
                status = PcdSelect(SelectedSnr);    //ѡ����Ƭ
                if(status == MI_OK)
                {
                    for(int i = 0;i < 4 ;i++)                                              //��д16������
                        {
                            status = PcdAuthState(KEYA, (i*4+3), DefaultKey2, SelectedSnr);  //��֤��Ƭ��ǰ����������
                            status = PcdAuthState(KEYB, (i*4+3), DefaultKey2, SelectedSnr);  //��֤��Ƭ��ǰ����������
//                            if(i == 0)
//                            {
//                                status = PcdWrite(1, inDoor);                              //��һ������1��дֵ(��������0�鲻��д) (��Ҫ����֤)
//                                rt_kprintf( "sta5:%d!\n",status );
//                            }
                            if(status == MI_OK)
                            {
                                for(int j = 0; j < 4; j++)
                                {
                                    status = PcdRead((i*4+j), buf);                             //��ȡ��ǰ����ÿ�����ֵ
                                    if(status == MI_OK)
                                    {
                                        for(int k = 0; k < 16; k++)                             //��ӡ��ǰ���ֵ
                                        {
                                            rt_kprintf("%x ", (uint16_t)buf[k]);
																					  if(i==0&&j==0)
																						{
/*																					  lcd_show_string(10, 69+16+24+32, 32, "%x",(uint16_t)buf[0]);
																						
																						lcd_show_string(10+42, 69+16+24+32, 32, "%x",(uint16_t)buf[1]);
																						
																						lcd_show_string(10+42+42, 69+16+24+32, 32, "%x",(uint16_t)buf[2]);
																							
																						lcd_show_string(10+42+42+42, 69+16+24+32, 32, "%x",(uint16_t)buf[3]);*/
																							if(flag==0)
																							{
																							mima[0]=(uint16_t)buf[0];
																							mima[1]=(uint16_t)buf[1];
																							mima[2]=(uint16_t)buf[2];
																							mima[3]=(uint16_t)buf[3];
																							}
																							else
																							{
																							rightmima[count][0]=(uint16_t)buf[0];
																							rightmima[count][1]=(uint16_t)buf[1];
																							rightmima[count][2]=(uint16_t)buf[2];
																							rightmima[count][3]=(uint16_t)buf[3];
																							}
																						}
																						
                                        }
                                        rt_kprintf("\n");
                                    }
                                }
                            }
                        }							
										rt_kprintf("operate finished! %x \n", status);		
                    if(flag==1)		
											{count++;		
												lcd_clear(WHITE); 
									for(i=0;i<count;i++)
									{
								  lcd_show_string(10, 10, 32, "memory card");
                  lcd_show_string(10, 10+32+i*32, 32, "%x",rightmima[i][0]);												
									lcd_show_string(10+42, 10+32+i*32, 32, "%x",rightmima[i][1]);
									lcd_show_string(10+42+42, 10+32+i*32, 32, "%x",rightmima[i][2]);
									lcd_show_string(10+42+42+42, 10+32+i*32, 32, "%x",rightmima[i][3]);
									}
								}
                    if((flag==0&&mima[0]==rightmima[0][0]&&mima[1]==rightmima[0][1]&&mima[2]==rightmima[0][2]&&mima[3]==rightmima[0][3])||
											 (flag==0&&mima[0]==rightmima[1][0]&&mima[1]==rightmima[1][1]&&mima[2]==rightmima[1][2]&&mima[3]==rightmima[1][3])||
										    (flag==0&&mima[0]==rightmima[2][0]&&mima[1]==rightmima[2][1]&&mima[2]==rightmima[2][2]&&mima[3]==rightmima[2][3])||
										     (flag==0&&mima[0]==rightmima[3][0]&&mima[1]==rightmima[3][1]&&mima[2]==rightmima[3][2]&&mima[3]==rightmima[3][3])||
										     (flag==0&&mima[0]==rightmima[4][0]&&mima[1]==rightmima[4][1]&&mima[2]==rightmima[4][2]&&mima[3]==rightmima[4][3])||
										     (flag==0&&mima[0]==rightmima[5][0]&&mima[1]==rightmima[5][1]&&mima[2]==rightmima[5][2]&&mima[3]==rightmima[5][3])) 
										{
										 rt_pin_write(PIN_BEEP, PIN_HIGH);
				             rt_thread_mdelay(500);  // ��ʱ500ms
                     rt_pin_write(PIN_BEEP, PIN_LOW);
											

										rt_pin_write(PIN_MOTOR_A, PIN_LOW);
                    rt_pin_write(PIN_MOTOR_B, PIN_HIGH);
										rt_thread_mdelay(500);  // ��ʱ500ms
										rt_pin_write(PIN_MOTOR_A, PIN_LOW);
                    rt_pin_write(PIN_MOTOR_B, PIN_LOW);
										} 
										else
										{
											if(flag==0)
											{
										rt_pin_write(PIN_BEEP, PIN_HIGH);rt_pin_write(GET_PIN(E, 7),PIN_HIGH);
				            rt_thread_mdelay(500);  // ��ʱ500ms
                    rt_pin_write(PIN_BEEP, PIN_LOW);rt_pin_write(GET_PIN(E, 7),PIN_LOW);
										rt_thread_mdelay(500);  // ��ʱ500ms
												rt_pin_write(PIN_BEEP, PIN_HIGH);rt_pin_write(GET_PIN(E, 7),PIN_HIGH);
				            rt_thread_mdelay(500);  // ��ʱ500ms
                    rt_pin_write(PIN_BEEP, PIN_LOW);rt_pin_write(GET_PIN(E, 7),PIN_LOW);
										rt_thread_mdelay(500);  // ��ʱ500ms
												rt_pin_write(PIN_BEEP, PIN_HIGH);rt_pin_write(GET_PIN(E, 7),PIN_HIGH);
				            rt_thread_mdelay(500);  // ��ʱ500ms
                    rt_pin_write(PIN_BEEP, PIN_LOW);rt_pin_write(GET_PIN(E, 7 ),PIN_LOW);
										rt_thread_mdelay(500);  // ��ʱ500ms
											}
											else
												{rt_pin_write(PIN_BEEP, PIN_HIGH);
				             rt_thread_mdelay(500);  // ��ʱ500ms
                     rt_pin_write(PIN_BEEP, PIN_LOW);}
										}
                    WaitCardOff();
                }
            }
        }
        rt_thread_mdelay(50);
				rt_pin_mode(PIN_KEY0, PIN_MODE_INPUT);
				if (rt_pin_read(PIN_KEY0) == PIN_LOW)
        {
            rt_thread_mdelay(50);
            if (rt_pin_read(PIN_KEY0) == PIN_LOW)
            {
                /* ??????,?? log,?? LED ? */
                LOG_D("KEY0 pressed!");
                rt_pin_write(PIN_LED_R, PIN_LOW);						  
							  if(flag==0) 
                {
								  flag=1;
									lcd_clear(WHITE); 
									for(i=0;i<count;i++)
									{
										lcd_show_string(10, 10, 32, "memory card:");
                  lcd_show_string(10, 10+32+i*32, 32, "%x",rightmima[i][0]);												
									lcd_show_string(10+42, 10+32+i*32, 32, "%x",rightmima[i][1]);
									lcd_show_string(10+42+42, 10+32+i*32, 32, "%x",rightmima[i][2]);
									lcd_show_string(10+42+42+42, 10+32+i*32, 32, "%x",rightmima[i][3]);
									}
								}
							  else 
									{
										flag=0;
										
									  lcd_clear(WHITE); 
										lcd_set_color(WHITE, BLACK);
		                lcd_show_string(0, 10, 32, "Access control system");
//                     lcd_draw_line(0, 10+32+32+10, 240, 10+32+32+10);
										draw_line();
										lcd_show_string(0, 10+32+32+10+10, 32, "Please do face recognition or swipe the card");
								  }
            }
        }
        else
        {
            /* ??????,?? LED ? */
            rt_pin_write(PIN_LED_R, PIN_HIGH);
					  
        }
    }
}
static int RC522_Priority_temp(void)
{
    rt_thread_t RC522_thread;
    RC522_thread = rt_thread_create("RC522_thread",
                                     RC522_thread_entry,
                                     "RC522_thread",
                                     1024,
                                     RC522_THREAD_PRIORITY,
                                     20);
    if (RC522_thread != RT_NULL)
    {
        rt_thread_startup(RC522_thread);
    }
    return RT_EOK;
}
INIT_APP_EXPORT(RC522_Priority_temp);