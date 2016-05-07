/*
 * apc.c
 *
 *  Created on: 2015-9-27
 *      Author: Administrator
 */

#define APF
#include <includes.h>

/* 两相静止坐标系下变量 *******************************************************/
static float Iapf_alfa, Iapf_beta;
static float Iload_alfa, Iload_beta;
/* 两相旋转坐标系下变量 *******************************************************/
static float Iapf_d, Iapf_q;
static float Iload_d, Iload_q;
static float UdcRampRef;

static float GetPllAngle(float ua, float ub);
static void Para_Init(void);
static void PID_Init(void);
static float AverageFilter(float ui, average_filter_instance* filter);
static void HarmonicDetection(float ia, float ib, float *pifa, float *pifb,
		float *pihd, float *pihq);

#ifdef _FLASH
#pragma CODE_SECTION(APF_Main, "ramfuncs");
#pragma CODE_SECTION(GetPllAngle, "ramfuncs");
#pragma CODE_SECTION(CheckPwrState, "ramfuncs");
#pragma CODE_SECTION(AverageFilter, "ramfuncs");
#pragma CODE_SECTION(HarmonicDetection, "ramfuncs");
#pragma CODE_SECTION(DCL_runDF22, "ramfuncs");
#endif
#pragma DATA_SECTION(graph_data1,"Filter1_RegsFile");
#pragma DATA_SECTION(graph_data2,"Filter2_RegsFile");
#pragma DATA_SECTION(graph_data3,"Filter3_RegsFile");

interrupt void FaultProcess(void)
{

	EPwm2Regs.AQCSFRC.all = 5;	//AB的输出连续强制为低
	EPwm3Regs.AQCSFRC.all = 5;
	EPwm4Regs.AQCSFRC.all = 5;
	EPwm6Regs.AQCSFRC.all = 5;
	EPwm7Regs.AQCSFRC.all = 5;
	EPwm8Regs.AQCSFRC.all = 5;

	EALLOW;
	EPwm2Regs.TZCLR.bit.CBC = 1;
	EPwm2Regs.TZCLR.bit.INT = 1;
	EDIS;

	// Acknowledge this interrupt to receive more interrupts from group 2
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

interrupt void APF_Main(void)
{
	static float compensatepercentage = 0;
	float udcp_out;
	float temp_alfa, temp_beta;
	float error_alfa, error_beta;
	float temp_d = 0, temp_q = 0;
	float temp_a, temp_b, temp_c;
	float sinVal, cosVal;
	float pwm_a, pwm_b, pwm_c;
	float ihd, ihq, ifa, ifb, id, iq, iha, ihb;
	float ux, uy, upccd_average;
	Uint16 temp;
	static Uint16 i;

	GPIO_WritePin(58, 1);
	MainTskTrigger = 1;		//主程序任务触发
	ADValueConvert();
	VectorAngle = GetPllAngle(Upcc_a, Upcc_b);	//得到电压矢量的夹角
	Udc_average = (UdcA + UdcB + UdcC) / 3;
	sincos(VectorAngle, &sinVal, &cosVal);
	clarke(Iapf_a, Iapf_b, &Iapf_alfa, &Iapf_beta);
	park(Iapf_alfa, Iapf_beta, &Iapf_d, &Iapf_q, sinVal, cosVal);

////	sincos(VectorAngle, &sinval, &cosval);
//	clarke(Iload_a, Iload_b, &Iload_alfa, &Iload_beta);
//	park(Iload_alfa, Iload_beta, &Iload_d, &Iload_q, sinVal, cosVal);
//
////	ia = AverageFilter(id, &IhdFilter);
////	ib = AverageFilter(iq, &IhqFilter);
//
//	id = DCL_runDF22(&DF22_Ihd,Iload_d);
//	iq = DCL_runDF22(&DF22_Ihq,Iload_q);
//
//	ihd = Iload_d - id;
//	ihq = Iload_q - iq;
//	inv_park(id, iq, &Iload_alfa, &Iload_beta, sinVal, cosVal);
//	inv_clarke(Iload_alfa, Iload_beta, &ifa, &ifb);
//	iha = Iload_a - ifa;
	HarmonicDetection(Iload_a, Iload_b, &ifa, &ifb, &ihd, &ihq);
	iha = Iload_a - ifa;
	/****************************电网电压反馈******************************/
	clarke(Upcc_a, Upcc_b, &ux, &uy);
	park(ux, uy, &ux, &uy, sinVal, cosVal);
	UpccDm = DCL_runDF22(&DF22_Udaverage, ux);

	/******************************检测APF是否有故障发生**************************************/
	if (GPIO_ReadPin(70) == DISABLE)
	{
		APF_State |= APF_STATE_BIT_OV;
	}
	else
	{
		APF_State &= ~ APF_STATE_BIT_OV;
	}
	if (GPIO_ReadPin(68) == DISABLE)
	{
		APF_State |= APF_STATE_BIT_OC1;
	}
	else
	{
		APF_State &= ~ APF_STATE_BIT_OC1;
	}
	if (GPIO_ReadPin(69) == DISABLE)
	{
		APF_State |= APF_STATE_BIT_OC2;
	}
	else
	{
		APF_State &= ~ APF_STATE_BIT_OC2;
	}
	if (GPIO_ReadPin(71) == DISABLE)
	{
		APF_State |= APF_STATE_BIT_IGBT_ERR;
	}
	else
	{
		APF_State &= ~ APF_STATE_BIT_IGBT_ERR;
	}

	/*****************检测APF是否启动运行或启动IGBT检查************************/
//由于开关检测时好时坏，因此启动均上由上位机确定
//	if (APF_STATE_BIT_STOP == APF_State || APF_STATE_BIT_SWSTOP == APF_State
//			|| APF_STATE_BIT_TEST == APF_State || 0 == APF_State)
//	{
//		if (GPIO_ReadPin(54) == SET)	//检查APF启动按钮是否按下
//		{
//			APF_State = 0;
//			GPIO_WritePin(GPIO_LED33, 0);
//		}
//		else
//		{
//			APF_State = APF_STATE_BIT_STOP;
//			GPIO_WritePin(GPIO_LED33, 1);
//		}
//		if (GPIO_ReadPin(53) == SET)	//检查APF是否启动IGBT检测
//		{
//			APF_State = APF_STATE_BIT_TEST;
//			GPIO_WritePin(GPIO_LED34, 0);
//		}
//		else
//		{
//			APF_State = APF_STATE_BIT_STOP;
//			GPIO_WritePin(GPIO_LED34, 1);
//		}
//	}
	/*****************************根据APF的状态进行操作*************************************/
	if (0 == APF_State)	//如果APF启动了
	{
		/*
		 * 直流母线平均电压控制
		 */
		UdcRampRef += 60.0 * APF_SWITCH_PERIOD;	//1s内给定值会增加60V
		if (UdcRampRef > UdcRef)
		{
			UdcRampRef = UdcRef;
		}
		udcp_out = pid_calculate(&pid_instance_Udcp, Udc_average - UdcRampRef);

		if ((UdcRampRef == UdcRef) && ((UdcRef - Udc_average) < 30)
				&& ((UdcRef - Udc_average) > -30))	//只有稳压结束后，才会进行谐波补偿
		{
			compensatepercentage += 100.0 * APF_SWITCH_PERIOD;	//1s内百分比会上增加100%
			if (compensatepercentage > CmpsateSet)
			{
				compensatepercentage = CmpsateSet;
			}
		}

//#if 1	//1在dq轴下进行谐波电流补偿

		/******************直流母线相间平衡基波负序电流计算********************/
		clarke(Udc_average - UdcA, Udc_average - UdcB, &temp_alfa, &temp_beta);
		temp_alfa = pid_calculate(&pid_instance_Udcn_d, temp_alfa);
		temp_beta = pid_calculate(&pid_instance_Udcn_q, temp_beta);
		sincos(VectorAngle * 2, &sinVal, &cosVal);
		inv_park(temp_alfa, temp_beta, &temp_alfa, &temp_beta, sinVal, cosVal);
		/****************************dq轴电流控制******************************/
		//电流环给定计算
		temp_d = udcp_out - temp_alfa - Iapf_d + ihd * compensatepercentage;
		temp_q = temp_beta - Iapf_q + ihq * compensatepercentage
				+ QRef * compensatepercentage * 0.01 * 1.414213562f;
		//PI调节
		temp_alfa = pid_calculate(&pid_instance_Ialfa, temp_d);
		temp_beta = pid_calculate(&pid_instance_Ibeta, temp_q);
		//重复控制
		RpBuffer[0][PointCnt] = temp_d + RpKr * RpBuffer[0][PointCnt];
		RpBuffer[1][PointCnt] = temp_q + RpKr * RpBuffer[1][PointCnt];

		temp = (PointCnt + POINT_NUM - RpLeadingBeat) % POINT_NUM;
		//重复控制器的结果需要滤波，去除高频分量
		error_alfa = DCL_runDF22(&DF22_RpFilter0, RpBuffer[0][temp]);
		error_beta = DCL_runDF22(&DF22_RpFilter1, RpBuffer[1][temp]);

		temp_alfa += error_alfa * RpGain + UpccDm;
		temp_beta += error_beta * RpGain;
		/*****************************坐标逆变换*******************************/
		inv_park(temp_alfa, temp_beta, &temp_alfa, &temp_beta, sinVal, cosVal);
		inv_clarke(temp_alfa, temp_beta, &pwm_a, &pwm_b);

		pwm_c = -pwm_a - pwm_b;

		/*
		 * 输出PWM波，单极性倍频PWM调制
		 */
		if (UdcA < DC_MIN_VOLTAGE)	//直流母线的电压值，不应低于整流电压值
		{
			UdcA = DC_MIN_VOLTAGE;
		}
		if (UdcB < DC_MIN_VOLTAGE)
		{
			UdcB = DC_MIN_VOLTAGE;
		}
		if (UdcC < DC_MIN_VOLTAGE)
		{
			UdcC = DC_MIN_VOLTAGE;
		}
		pwm_a = pwm_a / UdcA * 0.5;
		pwm_b = pwm_b / UdcB * 0.5;
		pwm_c = pwm_c / UdcC * 0.5;

		if (pwm_a > 0.5)
		{
			pwm_a = 0.5;
		}
		else if (pwm_a < -0.5)
		{
			pwm_a = -0.5;
		}

		if (pwm_b > 0.5)
		{
			pwm_b = 0.5;
		}
		else if (pwm_b < -0.5)
		{
			pwm_b = -0.5;
		}

		if (pwm_c > 0.5)
		{
			pwm_c = 0.5;
		}
		else if (pwm_c < -0.5)
		{
			pwm_c = -0.5;
		}

		temp_a = (0.5 + pwm_a) * Carrier_Wave_Count;
		temp_b = (0.5 + pwm_b) * Carrier_Wave_Count;
		temp_c = (0.5 + pwm_c) * Carrier_Wave_Count;

		EPwm2Regs.CMPA.bit.CMPA = temp_a;
		EPwm3Regs.CMPA.bit.CMPA = temp_b;
		EPwm4Regs.CMPA.bit.CMPA = temp_c;

		temp_a = (0.5 - pwm_a) * Carrier_Wave_Count;
		temp_b = (0.5 - pwm_b) * Carrier_Wave_Count;
		temp_c = (0.5 - pwm_c) * Carrier_Wave_Count;

		EPwm6Regs.CMPA.bit.CMPA = temp_a;
		EPwm7Regs.CMPA.bit.CMPA = temp_b;
		EPwm8Regs.CMPA.bit.CMPA = temp_c;

		/* Main Output Enable */
		EALLOW;
		EPwm2Regs.TZCLR.bit.OST = 1;
		EPwm3Regs.TZCLR.bit.OST = 1;
		EPwm4Regs.TZCLR.bit.OST = 1;
		EPwm6Regs.TZCLR.bit.OST = 1;
		EPwm7Regs.TZCLR.bit.OST = 1;
		EPwm8Regs.TZCLR.bit.OST = 1;
		EDIS;

	}
	else
	{
		if (APF_STATE_BIT_TEST == APF_State)
		{
			/* Main Output Enable */
			EALLOW;
			EPwm2Regs.TZCLR.bit.OST = 1;
			EPwm3Regs.TZCLR.bit.OST = 1;
			EPwm4Regs.TZCLR.bit.OST = 1;
			EPwm6Regs.TZCLR.bit.OST = 1;
			EPwm7Regs.TZCLR.bit.OST = 1;
			EPwm8Regs.TZCLR.bit.OST = 1;
			EDIS;
			EPwm2Regs.CMPA.bit.CMPA = 500;
			EPwm3Regs.CMPA.bit.CMPA = 1000;
			EPwm4Regs.CMPA.bit.CMPA = 1500;
			EPwm6Regs.CMPA.bit.CMPA = 500;
			EPwm7Regs.CMPA.bit.CMPA = 1000;
			EPwm8Regs.CMPA.bit.CMPA = 1500;
		}
		else
		{
			APF_State |= APF_STATE_BIT_STOP;
			/* Main Output Disable */
			EALLOW;
			EPwm2Regs.TZFRC.bit.OST = 1;
			EPwm3Regs.TZFRC.bit.OST = 1;
			EPwm4Regs.TZFRC.bit.OST = 1;
			EPwm6Regs.TZFRC.bit.OST = 1;
			EPwm7Regs.TZFRC.bit.OST = 1;
			EPwm8Regs.TZFRC.bit.OST = 1;
			EDIS;

			pid_reset(&pid_instance_Udcp);
			pid_reset(&pid_instance_Udcn_d);
			pid_reset(&pid_instance_Udcn_q);
			pid_reset(&pid_instance_Ialfa);
			pid_reset(&pid_instance_Ibeta);

			if (4000 == RpCutoffFreqence)
			{
				DF22 df = DF22_4000Hz;
				DF22_RpFilter0 = df;
				DF22_RpFilter1 = df;
			}
			else if (2500 == RpCutoffFreqence)
			{
				DF22 df = DF22_2500Hz;
				DF22_RpFilter0 = df;
				DF22_RpFilter1 = df;
			}
			else
			{
				DF22 df = DF22_3000Hz;
				DF22_RpFilter0 = df;
				DF22_RpFilter1 = df;
			}

			memset_fast(RpBuffer[0], 0, POINT_NUM * 2);	//这两条代码比较耗时
			memset_fast(RpBuffer[1], 0, POINT_NUM * 2);
		}
		UdcRampRef = Udc_average;
		compensatepercentage = 0;
	} /*end if (0 == APF_State)*/
	PointCnt = (PointCnt + 1) % POINT_NUM;

////	park(100*cosVal,100*sinVal,&Iload_d,&Iload_q,sinVal,cosVal);
	graph_data1[i] = Upcc_ab;
	graph_data2[i] = Upcc_bc;
//	graph_data3[i] = Iload_a;
//	graph_data4[i] = Iload_b;
	i++;
	if (i >= 500)
		i = 0;

	ExtDA_Output(0, VectorAngle * 500 + 4096);  //DA8
	ExtDA_Output(1, Upcc_a * 1 + 4096);			//DA7
	ExtDA_Output(2, Upcc_ab * 1 + 4096);		//DA6
	ExtDA_Output(3, Upcc_bc * 1 + 4096);		//DA5
	ExtDA_Output(4, Iapf_a * 100 + 4096);		//DA4
	ExtDA_Output(5, iha * 1000 + 4096);			//DA3
	ExtDA_Output(6, cosVal * 1000 + 4096);		//DA2
	ExtDA_Output(7, Iload_d * 100 + 4096);		//DA1

	GPIO_WritePin(58, 0);	//查看事个控制程序的中断时间
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}
void APF_Init(void)
{
	Para_Init();
	PID_Init();
	memset_fast(&RpBuffer[0][0], 0, POINT_NUM * 2);
	memset_fast(&RpBuffer[1][0], 0, POINT_NUM * 2);
}

static void Para_Init(void)
{
	Carrier_Wave_Count = 200000000 / 2 * 0.5 * APF_SWITCH_PERIOD - 1;

	UdcRef = DC_REF_DEFAULT;
	QRef = Q_REF_DEFAULT;
	CmpsateSet = 0;
	APF_State = APF_STATE_BIT_STOP;

	RpGain = 0;
	RpKr = 0.98;
	RpLeadingBeat = 2;
	RpCutoffFreqence = 3000;
	{
		DF22 df22 = DF22_3000Hz;
		DF22_RpFilter0 = df22;
		DF22_RpFilter1 = df22;
	}
	{
		DF22 df22 = DF22_2500Hz;
		DF22_UdcA = df22;
		DF22_UdcB = df22;
		DF22_UdcC = df22;
	}
	{
		DF22 df22 = DF22_70Hz;
		DF22_Udaverage = df22;
		DF22_Ihd = df22;
		DF22_Ihq = df22;
	}
	UdFilter.sum = 0;
	memset_fast(UdFilter.data, 0, 256 * 2);
	IhdFilter.sum = 0;
	memset_fast(IhdFilter.data, 0, 256 * 2);
	IhqFilter.sum = 0;
	memset_fast(IhqFilter.data, 0, 256 * 2);

	MainTskTrigger = 0;

}

static void PID_Init(void)
{
	pid_instance_Pll.Kp = 10;
	pid_instance_Pll.Ki = 10 * APF_SWITCH_PERIOD;
	pid_instance_Pll.Kd = 0;
	pid_init(&pid_instance_Pll, ENABLE);

	pid_instance_Ialfa.Kp = 8;
	pid_instance_Ialfa.Ki = 2 * APF_SWITCH_PERIOD;
	pid_instance_Ialfa.Kd = 0;
	pid_init(&pid_instance_Ialfa, ENABLE);

	pid_instance_Ibeta.Kp = 8;
	pid_instance_Ibeta.Ki = 2 * APF_SWITCH_PERIOD;
	pid_instance_Ibeta.Kd = 0;
	pid_init(&pid_instance_Ibeta, ENABLE);

	pid_instance_Udcp.Kp = 0.5;
	pid_instance_Udcp.Ki = 1 * APF_SWITCH_PERIOD;
	pid_instance_Udcp.Kd = 0;
	pid_init(&pid_instance_Udcp, ENABLE);

	pid_instance_Udcn_d.Kp = 0.5;
	pid_instance_Udcn_d.Ki = 1 * APF_SWITCH_PERIOD;
	pid_instance_Udcn_d.Kd = 0;
	pid_init(&pid_instance_Udcn_d, ENABLE);

	pid_instance_Udcn_q.Kp = 0.5;
	pid_instance_Udcn_q.Ki = 1 * APF_SWITCH_PERIOD;
	pid_instance_Udcn_q.Kd = 0;
	pid_init(&pid_instance_Udcn_q, ENABLE);
}
static float GetPllAngle(float ua, float ub)
{
	static float theta = 0;
	float w;
	float ud, uq, ux, uy;
	float valsin, valcos;

	clarke(ua, ub, &ux, &uy);
	sincos(theta, &valsin, &valcos);
//	valsin = sin(theta);
//	valcos = cos(theta);
	park(ux, uy, &ud, &uq, valsin, valcos);

	if (ud <= 0)
	{
		ud = -ud + 0.001;
	}
	uq = uq / ud;
	w = pid_calculate(&pid_instance_Pll, uq) + 100 * PI_CONST;
	theta += w * APF_SWITCH_PERIOD;
	if (theta > 2 * PI_CONST)
	{
		theta -= 2 * PI_CONST;
	}
	else if (theta < 0)
	{
		theta += 2 * PI_CONST;
	}

	return theta;
}

char CheckPwrState(float uab, float ubc, float uca, float udc)
{
	float u;
	char ans;
	static DF22 filter =
	{ 5.41589311124356e-07, 1.08317862224871e-06, 5.41589311124356e-07,
			-1.99791739975065, 0.997919566107898, 0, 0 };

	if (uab < 0)
		uab = -uab;
	if (ubc < 0)
	{
		ubc = -ubc;
	}
	if (uca < 0)
	{
		uca = -uca;
	}
	if (uab < ubc)
	{
		uab = ubc;
	}
	if (uab < uca)
	{
		uab = uca;
	}

	UacRectifier = DCL_runDF22(&filter, uab);
	if (UacRectifier > 20 && udc >= UacRectifier)

		ans = 1;
	else
		ans = 0;

	return ans;
}

static float AverageFilter(float ui, average_filter_instance* filter)
{
	float avr;
	/*
	 * mem[0] = sum(mem[2..2+cnt];
	 * mem[1] = i;
	 */
	filter->sum += ui;
	avr = filter->sum / 256;
	filter->data[filter->Cnt] = ui;
	filter->Cnt++;
	if (filter->Cnt >= 256)
		filter->Cnt = 0;
	filter->sum -= filter->data[filter->Cnt];

	return avr;
}

static void HarmonicDetection(float ia, float ib, float *pifa, float *pifb,
		float *pihd, float *pihq)
{
	float sinval, cosval;
	float id, iq, ix, iy;

	sincos(VectorAngle, &sinval, &cosval);
	clarke(ia, ib, &ix, &iy);
	park(ix, iy, &id, &iq, sinval, cosval);

	ix = AverageFilter(id, &IhdFilter);
	iy = AverageFilter(iq, &IhqFilter);

//	ix = DCL_runDF22(&DF22_Ihd,id);
//	iy = DCL_runDF22(&DF22_Ihq,id);

	*pihd = id - ix;
	*pihq = id - iy;
	inv_park(ix, iy, &id, &iq, sinval, cosval);
	inv_clarke(id, iq, pifa, pifb);
}

