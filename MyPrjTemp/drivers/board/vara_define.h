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
	int16_t num;
	char temperature;
	char state;
	char signal_intensity;
	long address;
	long name;
	char relative_alarm_value;
	char abs_alram_value;
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
	uint16_t dst_addr;
	uint16_t src_addr;
	uint16_t temperature;
	uint8_t state;
}RF_DATA;

typedef struct rf_ack{
	uint16_t dst_addr;
	uint16_t src_addr;
	uint8_t ack;
}RF_ACK;

APP_EXT uint16_t NodeNums;			//测温节点个数
APP_EXT uint16_t ErrorNodeNums;		//异常节点个数
APP_EXT uint16_t NodeInNets;		//已搜索到的节点
APP_EXT NODE_LIST * NodeList;



#endif /* VARA_DEFINE_H_ */
