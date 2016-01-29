/*
 * APF_Variables.h
 *
 *  Created on: 2015年2月3日
 *      Author: Administrator
 */

#ifndef APF_VARIABLES_H_
#define APF_VARIABLES_H_

#ifndef APF
#define APF_EXT extern
#else
#define APF_EXT
#endif

#include <includes.h>

/******************************************************************************/
/*                        APF Configuration	Variables						  */
/******************************************************************************/

APF_EXT int16 UdcRef;					//直流母线电压给定值，单位V
APF_EXT int16 QRef;					//无功功率给定，单位A
APF_EXT int16 CmpsateSet;				//谐波补偿率，单位%

/******************************************************************************/
/*                        APF Control Paramets								  */
/******************************************************************************/
APF_EXT float RpGain;
APF_EXT float RpKr;
APF_EXT char RpLeadingBeat;


/******************************************************************************/
/*                        APF internal State Variables						  */
/******************************************************************************/
APF_EXT Uint16 APF_State;				//APF状态
APF_EXT Uint16 Carrier_Wave_Count;
APF_EXT float Iapf_a,Iapf_b,Iapf_c;		//APF发出的线电流，单位A
APF_EXT float Iload_a,Iload_b,Iload_c;	//负载的线电流，单位A
APF_EXT float Upcc_ab,Upcc_bc,Upcc_ca;	//PCC点线电压，单位V
APF_EXT float Upcc_a,Upcc_b,Upcc_c;		//PCC点相电压，单位V
APF_EXT float UdcA,UdcB,UdcC,Udc;		//直流侧母线电压，单位V
APF_EXT float Udc_average;				//直流母线平均电压，单位V
APF_EXT float VectorAngle;				//电压矢量角度，单位rad
APF_EXT float GridFrequence;			//电网频率，单位Hz
APF_EXT float Iload_d[POINT_NUM],Iload_q[POINT_NUM];
APF_EXT Uint16 PointCnt;
//APF_EXT float RpBuffer[2][POINT_NUM];

APF_EXT iir_filter_instance IIR_for_Rpcontroller[2];
APF_EXT iir_filter_instance IIR_for_Id[2];
APF_EXT iir_filter_instance IIR_for_Iq[2];
APF_EXT iir_filter_instance IIR_for_Pll;
//APF_EXT float FFT_DataBuffer[FFT_LENGTH];
//APF_EXT float FFT_CalBuffer[FFT_LENGTH];
//APF_EXT float FFT_OutBuffer[FFT_LENGTH];
//APF_EXT struct rt_mailbox mb_fft;


APF_EXT pid_instance pid_instance_Ialfa;		//电流调节器PID
APF_EXT pid_instance pid_instance_Ibeta;
APF_EXT pid_instance pid_instance_Udcp;		//平均电压调节器PID
APF_EXT pid_instance pid_instance_Udcn_d;	//平衡电压调节器PID
APF_EXT pid_instance pid_instance_Udcn_q;
APF_EXT pid_instance pid_instance_Pll;



/******************************************************************************/
/*                        APF 电压电流有效值								  */
/******************************************************************************/
APF_EXT float Uab_rms;
APF_EXT float Ubc_rms;
APF_EXT float IloadA_rms;
APF_EXT float IloadB_rms;
APF_EXT float IapfA_rms;
APF_EXT float IapfB_rms;

APF_EXT char MainTskTrigger;
APF_EXT tCANMsgObject sTXCANMessage;
APF_EXT tCANMsgObject sRXCANMessage;
APF_EXT unsigned char ucTXMsgData[8];
APF_EXT unsigned char ucRXMsgData[8];
APF_EXT char FlagRxCAN;

#endif /* APF_VARIABLES_H_ */
