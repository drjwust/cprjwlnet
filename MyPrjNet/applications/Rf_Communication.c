/*
 * Rf_Communication.c
 *
 *  Created on: 2014年3月24日
 *      Author: Administrator
 */

#include <includes.h>


uint8_t PaTabel[8] =
//{ 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60 };
		{ 0xC2, 0XC2, 0xC2, 0XC2, 0xC2, 0XC2, 0xC2, 0XC2 };


void rf_thread_entry(void *para)
{
	uint8_t leng = 0, state, filedata[200];
	uint8_t rfbuffer[80];
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
	rt_kprintf("节点\t温度\t状态\t信号强度\t相对报警温度\t绝对报警温度\t时间\n");

	pRfData = (RF_DATA*) rfbuffer;
	while (1)
	{
		halRfRxModeOn();
		rt_event_recv(&event_rf, RFSTATE_RX,
		RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &event);
		RfState = RFSTATE_IDLE;
		leng = halRfReceivePacket((uint8_t *) pRfData, status);
		halRfRxModeOff();
		if ((status[1] & 0X80) && leng)	//只有CRC检验成功才保存数据
		{
			//将节点发过来的数据保存在data.dat文件和内存中
			node = pRfData->packet_addr;
			//因为NodeList->node[]数据的大小是1024，所以超过时，说明数据不准确要丢掉
			if (node < 1024)
			{
				NodeList->node[node].temperature = pRfData->temperature;//低10位数据为温度
				NodeList->node[node].state = pRfData->state;
				NodeList->node[node].signal_intensity = RfCalRssiValue(status[0]);
//				NodeList->node[node].signal_intensity = status[1] & 0X7F,
				now = time(RT_NULL), tmp = localtime(&now);
				rt_sprintf(filedata, "\n%d\t%d\t%d\t%d\t%d\t%d\t%d年%d月%d日\t%d:%d",
						node, NodeList->node[node].temperature,
						NodeList->node[node].state,
						NodeList->node[node].signal_intensity,
						NodeList->node[node].relative_alarm_value,
						NodeList->node[node].abs_alram_value,
						tmp->tm_year + 1900, tmp->tm_mon + 1, tmp->tm_mday,
						tmp->tm_hour, tmp->tm_min);
				leng = strlen(filedata);
				fd = open("/data.dat", O_RDWR, 0);
				lseek(fd, 0, DFS_SEEK_END);
				write(fd, filedata, leng);
				close(fd);
				rt_kprintf(filedata);
				//如果节点发生错误，则将节点的信息保存在alarm.dat文件中
				if (pRfData->state != 0)
				{
					NodeList->error_node[NodeList->error_num] =
							&NodeList->node[node];
					fd = open("/alarm.dat", O_RDWR, 0);
					lseek(fd, 0, DFS_SEEK_END);
					write(fd, filedata, leng);
					close(fd);
				}
				pRfData->temperature = NodeList->node[node].abs_alram_value;
				pRfData->state = NodeList->node[node].relative_alarm_value;
				halRfSendPacket((uint8_t *) pRfData, 6);
				GPIO_WriteBit(GPIOA, GPIO_Pin_0,
						1 - GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
				status[1] = 0;
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
