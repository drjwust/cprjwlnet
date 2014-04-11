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
	RF_DATA * pRfData;
	rf_init();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
	halSpiReadStatus(CCxxx0_VERSION);

	halRfReceiveEnable();
	while (1)
	{
		leng = sizeof(RF_DATA);
		pRfData = rt_malloc(leng);
		halRfReceivePacket((uint8_t* )pRfData, &leng);	// Transmit Tx buffer data
		if (pRfData->dst_addr == 12)
		{
			rt_thread_delay(600);
			rt_hw_led_off(2);
			rt_thread_delay(600);
			rt_hw_led_on(2);
			pRfData->dst_addr =0;
		}
	}
}
