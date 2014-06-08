/*
 * vara_define.h
 *
 *  Created on: 2013年12月9日
 *      Author: Administrator
 */

#ifndef VARA_DEFINE_H_
#define VARA_DEFINE_H_

#ifdef   APP_GLOBALS
	#define  APP_EXT
#else
	#define  APP_EXT extern
#endif

typedef struct node_data
{
	uint16_t num;
	uint16_t temperature;
	long address;
	long name;
	uint8_t state;
	int8_t signal_intensity;
	uint8_t relative_alarm_value;
	uint8_t abs_alram_value;
} NODE_DATA;

typedef struct node_list
{
	NODE_DATA node[1024];
	uint16_t count;
	uint16_t current;
	NODE_DATA *error_node[1024];
	uint16_t error_num;
} NODE_LIST;

typedef struct rf_packet{
	uint16_t packet_addr;
	uint16_t temperature;
	uint8_t state;
}RF_DATA;

APP_EXT uint16_t NodeNums;			//测温节点个数
APP_EXT uint16_t ErrorNodeNums;		//异常节点个数
APP_EXT uint16_t NodeInNets;		//已搜索到的节点
APP_EXT volatile NODE_LIST * NodeList;

APP_EXT struct rt_event event_rf;
APP_EXT struct rt_event rf_sem;
APP_EXT volatile uint8_t RfState;
APP_EXT volatile uint8_t RfGDOIntFlag;


#define RFSTATE_IDLE	0
#define RFSTATE_RX		1
#define RFSTATE_TX		2



#endif /* VARA_DEFINE_H_ */
