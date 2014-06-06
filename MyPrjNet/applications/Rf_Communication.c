/*
 * Rf_Communication.c
 *
 *  Created on: 2014年3月24日
 *      Author: Administrator
 */

#include <includes.h>

#define RSSI_OFFSET 74

uint8_t PaTabel[8] =
{ 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60 };

int8_t CalRssiValue(uint8_t rssi)
{
	int8_t db;
	if (rssi >= 128)
	{
		db = (int8_t) (rssi - 256) / 2 - RSSI_OFFSET;
	}
	else
	{
		db = (int8_t) (rssi / 2) - RSSI_OFFSET;
	}

	return db;
}

void rf_thread_entry(void *para)
{
	uint8_t leng = 0, state, filedata[200];
	uint8_t tf = 0, status[2];
	uint32_t event;
	RF_DATA* pRfData;
	int fd, node;
	time_t now;
	struct tm *tmp;

	RfState = RFSTATE_IDLE;
	rf_init();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
	halSpiReadStatus(CCxxx0_VERSION);

	pRfData = rt_malloc(sizeof(RF_DATA));
	while (1)
	{
		halRfRxModeOn();
		rt_event_recv(&event_rf, RFSTATE_RX,
				RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
				&event);
		RfState = RFSTATE_IDLE;
		leng = halRfReceivePacket((uint8_t *) pRfData, status);
		halRfRxModeOff();

		if (pRfData->dst_addr == 1 )
		{
			node = pRfData->src_addr;
			NodeList->node[node].temperature = pRfData->temperature;	//低10位数据为温度
			NodeList->node[node].state = pRfData->state;
			now = time(RT_NULL), tmp = localtime(&now);
			rt_sprintf(filedata, "\n%d\t%d\t%d\t%d\t%d\t%d年%d月%d日\t%d:%d", node,
					NodeList->node[node].temperature,
					NodeList->node[node].state,
					NodeList->node[node].relative_alarm_value,
					NodeList->node[node].abs_alram_value, tmp->tm_year + 1900,
					tmp->tm_mon + 1, tmp->tm_mday, tmp->tm_hour, tmp->tm_min);
			leng = strlen(filedata);
			fd = open("/data.dat", O_RDWR, 0);
			lseek(fd, 0, DFS_SEEK_END);
			write(fd, filedata, leng);
			close(fd);
			rt_kprintf(filedata);
			if (pRfData->state != 0)
			{
				NodeList->error_node[NodeList->error_num] =
						&NodeList->node[node];
				fd = open("/alarm.dat", O_RDWR, 0);
				lseek(fd, 0, DFS_SEEK_END);
				write(fd, filedata, leng);
				close(fd);
			}
		}
	}
}

void rf_thread_init(void)
{
	rt_thread_t init_thread;

	rt_event_init(&event_rf, "rf", 0);
	init_thread = rt_thread_create("rf", rf_thread_entry, RT_NULL, 2048, 10,
			20);
	if (init_thread != RT_NULL)
	{
		rt_thread_startup(init_thread);
	}
}

void inrf(void)
{
	uint8_t state;
	state = halSpiReadStatus(CCxxx0_PKTSTATUS);
	rt_kprintf("%d\n", state);
}

FINSH_FUNCTION_EXPORT(inrf, "inqurey the rf state")
