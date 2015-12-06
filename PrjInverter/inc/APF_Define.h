/*
 * APF_Define.h
 *
 *  Created on: 2015年2月3日
 *      Author: Administrator
 */

#ifndef APF_DEFINE_H_
#define APF_DEFINE_H_


#define APF_SWITCH_FREQ				(256*50)	//APF的开关频率
#define GID_FREQUENCE				(50)		//电网频率
#define FFT_LENGTH					256
#define DC_MIN_VOLTAGE				200			//直流母线最低电压
#define DC_REF_DEFAULT				330			//默认直流母线给定电压
#define Q_REF_DEFAULT				1			//无功电流默认给定值

#define APF_SWITCH_PERIOD			(1.0 / APF_SWITCH_FREQ)
#define POINT_NUM					(APF_SWITCH_FREQ / GID_FREQUENCE)
#define APF_STATE_BIT_STOP			1	//APF启动停止标志位 0:APF启动
#define APF_STATE_BIT_SWSTOP		2	//APF软停止标志位	1:APF停止
#define APF_STATE_BIT_OC			4	//APF过流标志位		1:APF过流
#define APF_STATE_BIT_OV			8	//APF直流母线过压标志位	1:APF过压


#define DISABLE			0
#define ENABLE			(!DISABLE)
#define SET				1
#define RESET			0




#endif /* APF_DEFINE_H_ */
