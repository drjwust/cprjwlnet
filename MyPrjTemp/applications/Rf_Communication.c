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
	uint8_t leng = 0, error;
	uint8_t tf = 0, status[2];
	RF_DATA* pRfData;

	rf_init();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
	halSpiReadStatus(CCxxx0_VERSION);

	halRfReceiveEnable();
//	halRfTransmitEnable();
	pRfData = rt_malloc(sizeof(RF_DATA));
	pRfData->dst_addr = 1;
	pRfData->src_addr = 0;
	pRfData->temperature = 1;
	pRfData->state = 0;
	while (1)
	{
//		pRfData->src_addr += 1;
//		if (pRfData->src_addr == 20)
//			pRfData->src_addr = 0;
//		pRfData->temperature += 1;
//		if (pRfData->temperature == 150)
//			pRfData->temperature = 1;
//		halRfSendPacket(pRfData,sizeof(RF_DATA));	// Transmit Tx buffer data
		halRfReceiveEnable();
		halSpiStrobe(CCxxx0_SRX);
		error = rt_sem_take(&rf_sem, 15);
		if (error == RT_EOK)
		{
			leng = halRfReceivePacket((uint8_t*) pRfData, status);
			if (leng != 0)
				rt_kprintf("\n接收成功(1),RSSI:%d\t编号(%d)\n", status[0]),pRfData->dst_addr;
			else
			{
				status[0] = halSpiReadStatus(CCxxx0_RSSI);
				rt_kprintf("接收错误(2),RSSI:%d\t", status[0]);
			}
		}
		else
		{
			status[0] = halSpiReadStatus(CCxxx0_RSSI);
			halRfTransmitEnable();
			rt_kprintf("接收失败(3),RSSI:%d\n", status[0]);
			halRfSendPacket((uint8_t* )pRfData, sizeof(RF_DATA));
		}
//		if (leng == 0)
//		{
//			status[0] = halSpiReadStatus(CCxxx0_RSSI);
//		}

//		rt_thread_delay(100);
		rt_hw_led_on(2);
//		rt_thread_delay(100);
		rt_hw_led_off(2);
	}
}

void rf_thread_init(void)
{
	rt_thread_t init_thread;

	rt_sem_init(&rf_sem, "rf", 0, RT_IPC_FLAG_FIFO);
	init_thread = rt_thread_create("rf", rf_thread_entry, RT_NULL, 2048, 10,
			20);
	if (init_thread != RT_NULL)
	{
		rt_thread_startup(init_thread);
	}
}
