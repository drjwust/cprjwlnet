/*
 * Rf_Communication.c
 *
 *  Created on: 2014Äê3ÔÂ24ÈÕ
 *      Author: Administrator
 */

#include <includes.h>

uint8_t PaTabel[8] =
{ 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60 };

static void rf_thread_entry(void *para)
{
	uint8_t leng = 0;
	uint8_t tf = 0;
	RF_DATA * pRfData;
	rf_init();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
	halSpiReadStatus(CCxxx0_VERSION);

	halRfReceiveEnable();
	leng = sizeof(RF_DATA);
	pRfData = rt_malloc(leng);

	while (1)
	{
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

void rf_thread_init(void)
{
	rt_thread_t init_thread;

	rt_event_init(&rf_event,"rf",RT_IPC_FLAG_FIFO);
	init_thread = rt_thread_create("rf", rf_thread_entry, RT_NULL, 2048,
			8, 20);
	if (init_thread != RT_NULL)
	{
		rt_thread_startup(init_thread);
	}
}


