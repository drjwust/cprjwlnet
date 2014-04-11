/*
 * Rf_Communication.c
 *
 *  Created on: 2014年3月24日
 *      Author: Administrator
 */

#include <includes.h>

uint8_t PaTabel[8] =
{ 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60 };

void rf_thread_entry(void *para)
{
	uint8_t leng = 0;
	uint8_t tf = 0;
	uint8_t TxBuf[8] =
	{ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };	// 8字节, 如果需要更长的数据包,请正确设置
	uint8_t RxBuf[8] =	{ 0 };

	rf_init();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
	halSpiReadStatus(CCxxx0_VERSION);
#if 0
	halRfReceiveEnable();
	while (1)
	{
		leng = 8;
		halRfReceivePacket(RxBuf, leng);	// Transmit Tx buffer data
		if (RxBuf[0] == 0x01)
		{
			rt_thread_delay(600);
			rt_hw_led_off(2);
			rt_thread_delay(600);
			rt_hw_led_on(2);
			RxBuf[0] = 0;
		}
	}
#else
	halRfTransmitEnable();
//	halRfReceiveEnable();
	while (1)
	{
		leng=8;
		halRfSendPacket(TxBuf,leng);	// Transmit Tx buffer data
		rt_thread_delay(300);
		rt_hw_led_on(2);
		rt_thread_delay(300);
		rt_hw_led_off(2);
	}
#endif
}
