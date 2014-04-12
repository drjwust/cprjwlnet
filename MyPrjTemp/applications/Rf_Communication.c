/*
 * Rf_Communication.c
 *
 *  Created on: 2014年3月24日
 *      Author: Administrator
 */

#include <includes.h>

uint8_t PaTabel[8] =
{ 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60 };

static void rf_thread_entry(void *para)
{
	uint8_t leng = 0, state, filedata[100];
	RF_DATA * pRfData;
	int fd, node;
	time_t now;
	struct tm *tmp;

	rf_init();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
	halSpiReadStatus(CCxxx0_VERSION);

	halRfReceiveEnable();
	leng = sizeof(RF_DATA);
	pRfData = rt_malloc(leng);
	if (pRfData == RT_NULL)
	{
		rt_kprintf("内存不足！");
	}

	while (1)
	{
		halRfSetRxMode();
		rt_sem_take(&rf_sem, RT_WAITING_FOREVER);	//等待数据接收完成
		halRfReceivePacket((uint8_t*) pRfData, &leng);	//接收到的数据放入pRfData中
		node = pRfData->src_addr;
		NodeList->node[node].temperature = pRfData->temperature & 0X3F;	//低10位数据为温度
		NodeList->node[node].state = pRfData->state;
		now = time(RT_NULL), tmp = localtime(&now);
		rt_sprintf(filedata, "%d\t%d\t%d\t%d\t%d\t\t%d年%d月%d日\t%d:%d\n", node,
				NodeList->node[node].temperature, NodeList->node[node].state,
				NodeList->node[node].relative_alarm_value,
				NodeList->node[node].abs_alram_value, tmp->tm_year + 1900,
				tmp->tm_mon + 1, tmp->tm_mday, tmp->tm_hour, tmp->tm_min);
		leng = strlen(filedata);
		fd = open("/data.dat", O_RDWR, 0);
		lseek(fd, 0, DFS_SEEK_END);
		write(fd,filedata,leng);
		close(fd);
		if (pRfData->state != 0)
		{
			NodeList->error_node[NodeList->error_num] = &NodeList->node[node];
			fd = open("/alarm.dat", O_RDWR, 0);
			lseek(fd, 0, DFS_SEEK_END);
			write(fd,filedata,leng);
			close(fd);
		}
		if (pRfData->dst_addr == 12)
		{
//			rt_kprintf("received data\n");
			if (state == RESET)
			{
				state = SET;
			}
			else
			{
				state = RESET;
			}
			rt_hw_led_set_state(2, state);
			pRfData->dst_addr = 0;
		}
	}
}

void rf_thread_init(void)
{
	rt_thread_t init_thread;

	rt_sem_init(&rf_sem, "rf", 0, RT_IPC_FLAG_FIFO);
	init_thread = rt_thread_create("rf", rf_thread_entry, RT_NULL, 2048, 10, 20);
	if (init_thread != RT_NULL)
	{
		rt_thread_startup(init_thread);
	}
}

