/*
 * Rf_Communication.c
 *
 *  Created on: 2014Äê3ÔÂ24ÈÕ
 *      Author: Administrator
 */

#include <includes.h>

uint8_t PaTabel[8] =
{ 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60 };

void rf_thread_entry(void *para)
{
	uint8_t leng = 0;
	uint8_t tf = 0;
	RF_DATA* pRfData;

	rf_init();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
	halSpiReadStatus(CCxxx0_VERSION);

	halRfTransmitEnable();
	pRfData = rt_malloc(sizeof(RF_DATA));
	pRfData->dst_addr = 1;
	pRfData->src_addr = 0;
	pRfData->temperature = 1;
	pRfData->state = 0;
	while (1)
	{
		pRfData->src_addr += 1;
		if (pRfData->src_addr == 1024)
			pRfData->src_addr =0;
		pRfData->temperature += 1;
		if (pRfData->temperature == 150)
			pRfData->temperature = 1;
		halRfSendPacket(pRfData,sizeof(RF_DATA));	// Transmit Tx buffer data
		rt_thread_delay(300);
		rt_hw_led_on(2);
		rt_thread_delay(300);
		rt_hw_led_off(2);
	}
}
