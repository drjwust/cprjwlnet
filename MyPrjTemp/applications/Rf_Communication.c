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
	uint8_t leng = 0,state;
	RF_DATA * pRfData;

	rf_init();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
	halSpiReadStatus(CCxxx0_VERSION);

	halRfReceiveEnable();
	leng = sizeof(RF_DATA);
	pRfData = rt_malloc(leng);

	while (1)
	{
		halRfSetRxMode();
		rt_sem_take(&rf_sem,RT_WAITING_FOREVER);
		halRfReceivePacket((uint8_t* )pRfData, &leng);	// Transmit Tx buffer data
		if (pRfData->dst_addr == 12)
		{
			rt_kprintf("received data\n");
			if (state == RESET)
			{
				state = SET;
			}
			else
			{
				state = RESET;
			}
			rt_hw_led_set_state(2,state);
			pRfData->dst_addr =0;
		}
//		rt_thread_delay(300);
	}
}

void rf_thread_init(void)
{
	rt_thread_t init_thread;

	rt_sem_init(&rf_sem,"rf",0,RT_IPC_FLAG_FIFO);
	init_thread = rt_thread_create("rf", rf_thread_entry, RT_NULL, 2048,
			8, 20);
	if (init_thread != RT_NULL)
	{
		rt_thread_startup(init_thread);
	}
}


