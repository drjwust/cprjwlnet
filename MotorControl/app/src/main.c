//DAC程序已根据管脚变化改动
// max7219已根据管脚变化改动
//GPIO输入输出配置已改动
//led灯已改为GPIO30
//位置检测UVW已根据管脚变化改动
//PWM接口和TZ接口的配置已根据管脚变化更改，其中死区，高有效还是低有效，哪些TZ信号触发封锁还要后续配置。
//AD采样接口已更改
#include <app_define.h>
#include <DCL.h>

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "variables.h"

#ifdef __FLASH
#pragma CODE_SECTION(CANARX_ISR, "ramfuncs");
#pragma CODE_SECTION(CANATX_ISR, "ramfuncs");
#pragma CODE_SECTION(adc_isr, "ramfuncs");
#pragma CODE_SECTION(DAC, "ramfuncs");
#pragma CODE_SECTION(max_7219, "ramfuncs");
#pragma CODE_SECTION(Temp_Cal, "ramfuncs");
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
#endif

interrupt void adc_isr(void);
interrupt void CANARX_ISR(void);
interrupt void CANATX_ISR(void);
void En1EPWM(void);
void En2EPWM(void);
void Dis1EPWM(void);
void Dis2EPWM(void);
static void GPIO_Init(void);
void Pos_Cal(void);
void Init_Pos(void);
void max_7219(Uint16 data);
Uint16 Temp_Cal(Uint16 data);
float Pos_CurAngle_Cal(float is);
float AngleEstimation(float iq);
float SpeedRef = 0.05;
float Ts = 0.001 / ISR_FREQUENCY;
float Omiga = 0;
float huang_omiga = 0;
float Omiga_abs;
float Omiga_prev = 0;
float theta_test = 0.75 * Pulse_Num;

float Time = 0;
DF22 AngEstBPF, AngEstLPF;
DF22 IdBSF, IqBSF;
PI_CONTROLLER AngEstPI;
float EstAngle = 0;
float RealAngle = 0;
float EstSpeed;
float RealSpeed;
float Imax1 = 0, Imax2 = 0;

float GraphData1[500];
float GraphData2[500];
float DispData1, DispData2;
int16 Cnt = 0;

float P1 = 0.00001;
float P2 = 0.00001;
float P3 = 0.000001;
float c = 0.000001;
float k1;
Uint16 IsrTicker = 0;
Uint16 RxTicker = 0;
Uint16 Timer_1s = 0;
Uint16 CANTX_MSGID = 0x101;
Uint16 PWM_STAT1 = 0;
Uint16 PWM_STAT2 = 0;
Uint16 counter = 0;
Uint16 U_uvw, V_uvw, W_uvw, Direction, Pos_Error;
Uint16 LEDSG1, LEDSG2;
float Is = 0;
int16 Pos_Zero = 0, Position, Pos_Prev, Omiga_sum, Omiga_count;
int16 huang1, huang_differ, huang_prev, huang_sum; //通过can传上来的位置等数据
float Cur_A = 0;
float Cur_B = 0;
float Cur_C = 0;
float Cur_U = 0;
float Cur_V = 0;
float Cur_W = 0;
float UDC = 0;
float Ud_see;
float Uq_see;
float powerfactor; //功率因数
float Uvector; //电压矢量幅值
float Ivector; //电流矢量幅值
float Ivector_old = 0;
int16 Temp_A = 0;
int16 Temp_B = 0;
int16 Temp_C = 0;

COMREG_PARA COMReg_Para1 = COMReg_Para_DEFAULT; //电机1的参数
COMREG_PARA COMReg_Para2 = COMReg_Para_DEFAULT; //电机2的参数
RAMP ramp1 = RAMP_DEFAULTS; //电机1VVVF斜坡
RAMP ramp2 = RAMP_DEFAULTS; //电机2VVVF斜坡
RAMP ramp_speed1 = RAMP_DEFAULTS; //电机1转速给定斜坡
RAMP ramp_speed2 = RAMP_DEFAULTS; //电机2转速给定斜坡
THETA theta1 = THETA_CALC_DEFAULTS;
THETA theta2 = THETA_CALC_DEFAULTS;
TEMPCUR_REG Tempcurve = TEMPCURVE_DEFAULT;
ADResult_REG ADResult = ADRESULT_DEFAULT;

// Instance a few transform objects
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;
CLARKE clarke2 = CLARKE_DEFAULTS;
PARK park2 = PARK_DEFAULTS;
IPARK ipark2 = IPARK_DEFAULTS;

// Instance PI regulators to regulate the d and q  axis currents, and speed
PI_CONTROLLER pi_Omiga1 = OMIGAPI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_id1 = CURRENTPI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iq1 = CURRENTPI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_Omiga2 = OMIGAPI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_id2 = CURRENTPI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iq2 = CURRENTPI_CONTROLLER_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs
SVGEN svgen1 = SVGEN_DEFAULTS;
SVGEN svgen2 = SVGEN_DEFAULTS;

void DAC(Uint32 DAinput)
{
	unsigned int i = 0;
	Uint32 temp = 0x800000;
	GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
	DELAY_US(0.1L);
	for (i = 0; i <= 23; i++)
	{
		GpioDataRegs.GPASET.bit.GPIO21 = 1; //时钟下降沿数据移入
		if ((DAinput & temp) == 0)
			GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;
		else
			GpioDataRegs.GPBSET.bit.GPIO33 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;
		temp = temp >> 1;
	}
	GpioDataRegs.GPBSET.bit.GPIO32 = 1;
	DELAY_US(0.1L);
}

void main(void)
{
	InitSysCtrl();
	InitGpio();
#ifdef __FLASH
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
#endif

	GPIO_Init();
	DINT;
	InitPieCtrl();

	IER = 0x0000;
	IFR = 0x0000;

	InitPieVectTable();
	InitAdc();
	InitEPwm();
	InitECan();
	InitEQep();
	Init_Pos();
	max_7219(0x0900);		//No decode for digits7-0
	max_7219(0x0A0B);		//23/32Intensity
	max_7219(0x0B01);		//Scan Display 0 1 
	max_7219(0x0C01);		//Shutdown Regsiter Normal Operation
	max_7219(0x0F00);		//Display Normal Operation
	Dis1EPWM();
	Dis2EPWM();
	EALLOW;
	// This is needed to write to EALLOW protected registers
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;			// 使能PIE总
	PieVectTable.ADCINT = &adc_isr;
	PieVectTable.ECAN0INTA = &CANARX_ISR;
	PieVectTable.ECAN1INTA = &CANATX_ISR;
	EDIS;

	// Enable PIE group 1 interrupt 6 for AD_INT
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1;			// AD
	PieCtrlRegs.PIEIER9.bit.INTx5 = 1; 			// eCAN0INT-A  接收中断
	PieCtrlRegs.PIEIER9.bit.INTx6 = 1; 			// eCAN1INT-A  发送中断

	// Enable CPU INT1 for AD_INT:

	IER |= M_INT1;
	IER |= M_INT9;
	// Enable global Interrupts and higher priority real-time debug events:
	EINT;
	// Enable Global interrupt INTM
	ERTM;
	// Enable Global realtime interrupt DBGM
	/*reset IGBT*/
	GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;
	GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
	DELAY_US(5000);
	GpioDataRegs.GPCSET.bit.GPIO65 = 1;
	GpioDataRegs.GPCSET.bit.GPIO66 = 1;

	Pos_Prev = huang1;
	for (;;)
	{
		if (COMReg_Para1.Mode == 1)
		{
			LEDSG1 = 0x0130;	//1
			LEDSG2 = 0x027E;
		}
		else if (COMReg_Para1.Mode == 2)
		{
			LEDSG1 = 0x016D;	//2
			LEDSG2 = 0x027E;
		}
		else if (COMReg_Para1.Mode == 3)
		{
			LEDSG1 = 0x0179;	//3.
			LEDSG2 = 0x027E;
		}
		else if (COMReg_Para1.Mode == 4)
		{
			LEDSG1 = 0x0133;	//4
			LEDSG2 = 0x027E;
		}
		else
		{
			LEDSG1 = 0x017E;
			LEDSG2 = 0x027E;		//显示00.
		}
		if (COMReg_Para2.Mode == 1)
		{
			LEDSG2 = 0x0230;		//显示01.
		}
		else if (COMReg_Para2.Mode == 2)
		{
			LEDSG2 = 0x026D;
		}
		else if (COMReg_Para2.Mode == 3)
		{
			LEDSG2 = 0x0279;
		}
		else if (COMReg_Para2.Mode == 4)
		{
			LEDSG2 = 0x0233;
		}
		else
		{
			LEDSG1 = 0x017E;
			LEDSG2 = 0x027E;		//显示00.
		}
		DAC(0x00100000 | ((Uint16) (3891) << 4));//5.1987--DA输出5V（4095）时对应的直流母线保护电压是788V
		DAC(0x00120000 | ((Uint16) (3600) << 4));//((Uint16)(COMReg_Para1.I_Prtct*4.55)<<4));//4.55-DA输出5V（4095）时对应的电流保护值为900A
//		DAC(0x00140000 | ((Uint16) (RealSpeed * 100 + 2048) << 4));
//		DAC(0x00160000 | ((Uint16) (EstSpeed * 100 + 2048) << 4));
		DAC(0x00140000 | ((Uint16) (RealAngle * 500) << 4)); //TODO DAC输出
		DAC(0x00160000 | ((Uint16) (EstAngle * 500) << 4)); //park1.Qs为iq标么值缌值为900A
//		DAC(0x00140000 | ((Uint16) (DispData1 * 300 + 2049) << 4)); //TODO DAC输出
//		DAC(0x00160000 | ((Uint16) (DispData2 * 300 + 2048) << 4)); //park1.Qs为iq标么值缌值为900A
		if ((COMReg_Para1.Runcode == 1) && (PWM_STAT1 == 0))
			En1EPWM();
		if (COMReg_Para1.Runcode != 1)
		{

			AngEstPI = (PI_CONTROLLER
					)PI_ANGEST_DEFAULT;
			AngEstBPF = (DF22
					)DF22_BPF_800HZ_BW320HZ;
			AngEstLPF = (DF22
					)DF22_LPF_100HZ;
			IqBSF = (DF22
					) DF22_BSF_800HZ_BW80HZ;
			IdBSF = (DF22
					) DF22_BSF_800HZ_BW80HZ;
			Time = 0;
			Imax1 = 0;
			Imax2 = 0;

//			AngEstBPF.x1 = 0;
//			AngEstBPF.x2 = 0;
//			AngEstLPF.x1 = 0;
//			AngEstLPF.x2 = 0;

			pi_Omiga1.Ref = 0;
			pi_Omiga1.Out = 0;
			pi_Omiga1.i1 = 0;
			pi_id1.Ref = 0;
			pi_id1.i1 = 0;
			pi_id1.Out = 0;
			pi_iq1.Ref = 0;
			pi_iq1.i1 = 0;
			pi_iq1.Out = 0;
			theta1.Out = 0;
			ramp1.Out = 0;
			ramp_speed1.Out = 0;
			ipark1.Ds = 0;
			ipark1.Qs = 0;
			Ivector_old = 0;

			if (PWM_STAT1 == 1)
			{
				Dis1EPWM();
			}
		}
		if ((COMReg_Para2.Runcode == 1) && (PWM_STAT2 == 0))
			En2EPWM();
		if (COMReg_Para2.Runcode != 1)
		{

			pi_Omiga2.Ref = 0;
			pi_Omiga2.Out = 0;
			pi_Omiga2.i1 = 0;
			pi_id2.Ref = 0;
			pi_id2.i1 = 0;
			pi_id2.Out = 0;
			pi_iq2.Ref = 0;
			pi_iq2.i1 = 0;
			pi_iq2.Out = 0;
			theta2.Out = 0;
			ramp2.Out = 0;
			ramp_speed2.Out = UDC;
			ipark2.Ds = 0;
			ipark2.Qs = 0;
			if (PWM_STAT2 == 1)
			{
				Dis2EPWM();
			}
		}
		/*---控制开关量---*/
		/*if(COMReg_Para1.switch_1==1)
		 {
		 GpioDataRegs.GPCCLEAR.bit.GPIO74 = 1;
		 }
		 else
		 {
		 if((Temp_B>=50)||(Temp_A>=50)||(Temp_C>=50))
		 {
		 GpioDataRegs.GPCCLEAR.bit.GPIO74 = 1;
		 }
		 if((Temp_B<40)&&(Temp_A<=40)&&(Temp_C<=40))
		 {
		 GpioDataRegs.GPCSET.bit.GPIO74 = 1;
		 }
		 }*/
		/*---过温警告---*/
		/*if((Temp_A >= COMReg_Para1.Temp_Warning) || (Temp_B >= COMReg_Para1.Temp_Warning) || (Temp_C >= COMReg_Para1.Temp_Warning))
		 {
		 LEDSG2=0x02B8;		//显示F.X.
		 COMReg_Para1.FaultCode = TEMPHI;
		 if((Temp_A >= COMReg_Para1.Temp_Prtct) || (Temp_B >= COMReg_Para1.Temp_Prtct) || (Temp_C >= COMReg_Para1.Temp_Prtct))
		 {
		 DisEPWM();
		 COMReg_Para1.FaultCode = OVERTEMP;
		 }
		 }
		 else
		 {
		 LEDSG2 = 0x029F;		//显示0X.
		 COMReg_Para1.FaultCode = NOFAULT;
		 }*/
		/*---过电流软件保护---*/
		if ((Cur_A > 20) || (Cur_B > 20) || (Cur_C > 20) || (Cur_A < -20)
				|| (Cur_B < -20) || (Cur_C < -20))
		{
			LEDSG2 = 0x0247;		//显示F.X.
			COMReg_Para1.FaultCode = OVERCUR;
			Dis1EPWM();
		}
		else
		{
			COMReg_Para1.FaultCode = NOFAULT;
		}
		if ((Cur_U > 20) || (Cur_V > 20) || (Cur_W > 20) || (Cur_U < -20)
				|| (Cur_V < -20) || (Cur_W < -20))
		{
			LEDSG2 = 0x0247;		//显示F.X.
			COMReg_Para2.FaultCode = OVERCUR;
			Dis2EPWM();
		}
		else
		{
			COMReg_Para2.FaultCode = NOFAULT;
		}
		/*---过电压软件保护---*/
		if (UDC > 650)
		{
			LEDSG2 = 0x02B8;		//显示F.X.
			COMReg_Para1.FaultCode = OVERVOL;
			Dis1EPWM();
			Dis2EPWM();
		}
		max_7219(LEDSG1);
		max_7219(LEDSG2);
		if (EPwm1Regs.TZFLG.bit.OST == 1)
		{
			GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;
			GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
			DELAY_US(1L);
			GpioDataRegs.GPCSET.bit.GPIO65 = 1;
			GpioDataRegs.GPCSET.bit.GPIO66 = 1;
		}
	}

}

// MainISR 10kHz
//
interrupt void adc_isr(void)
{

// Verifying the ISR

	IsrTicker++;
	Timer_1s++;
	Time += SAMPLE_PERIOD;
	ADResult.I_V_DSP = AdcRegs.ADCRESULT0 >> 4;
	ADResult.I_V_DSP += AdcRegs.ADCRESULT9 >> 4;
	ADResult.I_V_DSP = ADResult.I_V_DSP / 2;

	ADResult.UV_2_DSP = AdcRegs.ADCRESULT1 >> 4;
	ADResult.UV_2_DSP += AdcRegs.ADCRESULT10 >> 4;
	ADResult.UV_2_DSP = ADResult.UV_2_DSP / 3;

	ADResult.Udc_DSP = AdcRegs.ADCRESULT2 >> 4;
	ADResult.Udc_DSP += AdcRegs.ADCRESULT11 >> 4;
	ADResult.Udc_DSP = ADResult.Udc_DSP / 2;

	ADResult.IpA_DSP = AdcRegs.ADCRESULT3 >> 4;
	ADResult.IpA_DSP += AdcRegs.ADCRESULT12 >> 4;
	ADResult.IpA_DSP = ADResult.IpA_DSP / 2;

	ADResult.I_U_DSP = AdcRegs.ADCRESULT5 >> 4;
	ADResult.I_U_DSP += AdcRegs.ADCRESULT13 >> 4;
	ADResult.I_U_DSP = ADResult.I_U_DSP / 2;

	ADResult.VW_2_DSP = AdcRegs.ADCRESULT6 >> 4;
	ADResult.VW_2_DSP += AdcRegs.ADCRESULT14 >> 4;
	ADResult.VW_2_DSP = ADResult.VW_2_DSP / 2;

	ADResult.IpB_DSP = AdcRegs.ADCRESULT7 >> 4;
	ADResult.IpB_DSP += AdcRegs.ADCRESULT15 >> 4;
	ADResult.IpB_DSP = ADResult.IpB_DSP / 2;

	ADResult.Vref2V048 = AdcRegs.ADCRESULT4 >> 4;

	ADResult.AGND = AdcRegs.ADCRESULT8 >> 4;

	if (ADResult.ADtimes == 0)				//校准AD零位
	{
		ADResult.I_V_DSP0 = ADResult.I_V_DSP;
		ADResult.UV_2_DSP0 = ADResult.UV_2_DSP;
//		ADResult.Udc_DSP0 = ADResult.Udc_DSP;	//这种定标方式会在直流母线上残余电压时出现问题
		ADResult.Udc_DSP0 = 29;	//TODO
		ADResult.IpA_DSP0 = ADResult.IpA_DSP;
		ADResult.I_U_DSP0 = ADResult.I_U_DSP;
		ADResult.VW_2_DSP0 = ADResult.VW_2_DSP;
		ADResult.IpB_DSP0 = ADResult.IpB_DSP;
		ADResult.ADtimes = 1;
	}
	Cur_A = -(ADResult.IpA_DSP - ADResult.IpA_DSP0) * I_ABC_COFF;
	Cur_B = -(ADResult.IpB_DSP - ADResult.IpB_DSP0) * I_ABC_COFF;
	Cur_C = -Cur_A - Cur_B;
	Cur_U = -(ADResult.I_U_DSP - ADResult.I_U_DSP0) * I_ABC_COFF;
	Cur_V = -(ADResult.I_V_DSP - ADResult.I_V_DSP0) * I_ABC_COFF;
	Cur_W = -Cur_U - Cur_V;
	UDC = (ADResult.Udc_DSP - ADResult.Udc_DSP0) * U_UDC_COFF;
	Temp_A = 0;				//Temp_Cal(ADResult.Temp_A);
	Temp_B = 0;				//Temp_Cal(ADResult.Temp_B);
	Temp_C = 0;				//Temp_Cal(ADResult.Temp_C);
	//if(COMReg_Para1.Mode == 1||COMReg_Para1.Mode == 2||COMReg_Para1.Mode == 3||COMReg_Para1.Mode == 4)
	Pos_Cal();
	//else
//	{
	/*
	 * 下面这段代码应该没有作用
	 */
	huang_differ = huang1 - huang_prev;
	if (huang_differ < -300)
		huang_differ = huang_differ + 4096;
	else if (huang_differ > 300)
		huang_differ = huang_differ - 4096;
	huang_prev = huang1;
	huang_sum += huang_differ;
	RxTicker++;
	if (RxTicker >= 500)
	{
		huang_omiga = huang_sum * 0.048828125;
		huang_sum = 0;
		RxTicker = 0;
	}
//	}
	/*******************************/
	if (IsrTicker >= 500)			//50ms周期时钟
	{
		struct ECAN_REGS ECanaShadow;
		IsrTicker = 0;
		ECanaMboxes.MBOX1.MDL.byte.BYTE0 = ((int16) (pi_id1.Fbk) & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE1 = (int16) (pi_id1.Fbk) & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE2 = ((int16) (pi_iq1.Fbk) & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE3 = (int16) (pi_iq1.Fbk) & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = ((int16) (ipark1.Ds) & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = (int16) (ipark1.Ds) & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = ((int16) (ipark1.Qs) & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = (int16) (ipark1.Qs) & 0x00FF;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

	}
	if (Timer_1s >= 10000)
	{

		Timer_1s = 0;
		GpioDataRegs.GPATOGGLE.bit.GPIO30 = 1;
		if ((COMReg_Para1.Mode == 3 && COMReg_Para1.Runcode == 1)
				|| (COMReg_Para1.Mode == 7 && COMReg_Para1.Runcode == 1))
		{
			if (counter < 8)
			{
				counter++;
				theta_test = theta_test + 512;
				if (theta_test > Pulse_Num)
					theta_test = theta_test - Pulse_Num;
			}
			else
			{
				counter = 24;
				theta_test = 0; //转子定位在a相轴线上todo 3072
			}
		}
		else
			counter = 0;

	}

	if (COMReg_Para1.Mode == 4 && COMReg_Para1.Runcode == 1) //calculate vvvf theta
	{
//	    ramp1.TargetValue = SpeedRef;
//	    ramp1.Step = 0.000004;
//		RAMP_MACRO(ramp1)
//	    theta1.Freq = ramp1.Out;
//		theta1.StepAngleMax = BASE_FREQ*Ts;//VVVF模最终电流稳定在10Hz，200rpm。
//		THETA_CALC(theta1)
		theta1.Freq = 10;
		theta1.StepAngleMax = Ts; //VVVF模最终电流稳定在10Hz，200rpm。
		THETA_CALC(theta1)
	}

	if (COMReg_Para2.Mode == 4 && COMReg_Para2.Runcode == 1)
	{
		ramp2.TargetValue = SpeedRef;
		ramp2.Step = 0.000004;
		RAMP_MACRO(ramp2)
		theta2.Freq = ramp2.Out;
		theta2.StepAngleMax = BASE_FREQ * Ts; //VVVF模式最终电流稳定在10Hz，200rpm。
		THETA_CALC(theta2)
	}
// ------------------------------------------------------------------------------
//  Measure phase currents. 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------

	clarke1.As = Cur_A; // Phase A curr.
	clarke1.Bs = Cur_B; // Phase B curr.

	CLARKE_MACRO(clarke1)

	clarke2.As = Cur_U; // Phase A curr.
	clarke2.Bs = Cur_V; // Phase B curr.

	CLARKE_MACRO(clarke2)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------ 
	/*-----电机1-----*/
	if (COMReg_Para1.Mode == 1) //1转矩模式，2转速模式，3定位模式，4VVVF测试模式
	{
//		park1.Angle = (float) (Position + COMReg_Para1.Pos_Differ) / Pulse_Num;
		park1.Angle = EstAngle;
	}
	if (COMReg_Para1.Mode == 2) //1转矩模式，2转速模式，3定位模式，4VVVF测试模式
	{
		park1.Angle = (float) (Position + COMReg_Para1.Pos_Differ) / Pulse_Num;
	}
	if (COMReg_Para1.Mode == 3)
	{
		park1.Angle = theta_test / Pulse_Num;
	}
	if (COMReg_Para1.Mode == 4)
	{
		park1.Angle = theta1.Out;
	}
	if (COMReg_Para1.Mode == 5) //5can转矩模式
	{
		park1.Angle = (float) (huang1 + COMReg_Para1.Pos_Differ) / Pulse_Num;
	}
	if (COMReg_Para1.Mode == 6) //6can转速模式
	{
		park1.Angle = (float) (huang1 + COMReg_Para1.Pos_Differ) / Pulse_Num;
	}
	if (COMReg_Para1.Mode == 7) //7can定位模式
	{
		park1.Angle = theta_test / Pulse_Num;
	}

	if (COMReg_Para1.Mode == 1)
	{
		sincos(park1.Angle, &park1.Sine, &park1.Cosine);
	}
	else
	{
		park1.Sine = sin(park1.Angle * 2 * PI_CONSTANT);
		park1.Cosine = cos(park1.Angle * 2 * PI_CONSTANT);
	}
	park1.Alpha = clarke1.Alpha;
	park1.Beta = clarke1.Beta;

	PARK_MACRO(park1)

	ramp_speed1.TargetValue = COMReg_Para1.Omiga_Ref;
	ramp_speed1.Step = 0.08;
	if (COMReg_Para1.Runcode == 1)
	{
		RAMP_MACRO(ramp_speed1)
		pi_Omiga1.Ref = ramp_speed1.Out; //转速给定斜坡上升
	}
	if (COMReg_Para1.Mode == 5 || COMReg_Para1.Mode == 6
			|| COMReg_Para1.Mode == 7)
		pi_Omiga1.Fbk = huang_omiga;
	else
		pi_Omiga1.Fbk = Omiga;
	pi_Omiga1.Kp = (float) COMReg_Para1.Omiga_Kp * 0.001;
	pi_Omiga1.Ki = (float) COMReg_Para1.Omiga_Ki * 0.00001;
	PI_MACRO(pi_Omiga1)

	/*-----电机2-----*/
	if (COMReg_Para2.Mode == 1) //1转矩模式，2转速模式，3定位模式，4输出0.5占空比PWM
	{
		park2.Angle = (float) (Position + COMReg_Para2.Pos_Differ) / Pulse_Num;
	}
	if (COMReg_Para2.Mode == 2)
	{
		park2.Angle = (float) (Position + COMReg_Para2.Pos_Differ) / Pulse_Num;
	}
	if (COMReg_Para2.Mode == 3)
	{
		park2.Angle = theta_test / Pulse_Num;
	}
	if (COMReg_Para2.Mode == 4)
	{
		park2.Angle = theta2.Out;
	}

	park2.Sine = sin(park2.Angle * 2 * PI_CONSTANT);
	park2.Cosine = cos(park2.Angle * 2 * PI_CONSTANT);
	park2.Alpha = clarke2.Alpha;
	park2.Beta = clarke2.Beta;

	PARK_MACRO(park2)

	ramp_speed2.TargetValue = COMReg_Para2.Omiga_Ref;
	ramp_speed2.Step = 0.08;
	if (COMReg_Para2.Mode == 2 && COMReg_Para2.Runcode == 1)
	{
		RAMP_MACRO(ramp_speed2)
		pi_Omiga2.Ref = ramp_speed2.Out; //转速给定斜坡上升
	}
	pi_Omiga2.Fbk = Omiga;
	pi_Omiga2.Kp = (float) COMReg_Para2.Omiga_Kp * 0.001;
	pi_Omiga2.Ki = (float) COMReg_Para2.Omiga_Ki * 0.00001;
	PI_MACRO(pi_Omiga2)

// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI IQ controller macro
// ------------------------------------------------------------------------------  
	/*-----电机1-----*/
	if (COMReg_Para1.Mode == 1 || COMReg_Para1.Mode == 3
			|| COMReg_Para1.Mode == 5 || COMReg_Para1.Mode == 7)
	{
		pi_id1.Ref = COMReg_Para1.Id_Ref;
		pi_iq1.Ref = COMReg_Para1.Iq_Ref;
	}

	else if (COMReg_Para1.Mode == 2 || COMReg_Para1.Mode == 6)
	{
		pi_id1.Ref = COMReg_Para1.Id_Ref;
		pi_iq1.Ref = pi_Omiga1.Out;
	}

	if (COMReg_Para1.Mode == 1)		//使用转矩模型进行位置估计,消除高频注入对电流控制带来的影响
	{
		pi_iq1.Fbk = DCL_runDF22(&IqBSF, park1.Qs);	//消除高频注入带来的影响
		pi_id1.Fbk = DCL_runDF22(&IdBSF, park1.Ds);	//消除高频注入带来的影响
	}
	else
	{
		pi_iq1.Fbk = park1.Qs;
		pi_id1.Fbk = park1.Ds;
	}

	pi_iq1.Kp = (float) COMReg_Para1.I_Kp * 0.001;
	pi_iq1.Ki = (float) COMReg_Para1.I_Ki * 0.00001;
	pi_iq1.Umax = UDC * ONEbySQRT3;
	pi_iq1.Umin = -UDC * ONEbySQRT3;
	PI_MACRO(pi_iq1)

	pi_id1.Kp = (float) COMReg_Para1.I_Kp * 0.001;
	pi_id1.Ki = (float) COMReg_Para1.I_Ki * 0.00001;
	pi_id1.Umax = UDC * ONEbySQRT3;
	pi_id1.Umin = -UDC * ONEbySQRT3;
	PI_MACRO(pi_id1)

	/***************************无速度传感器的高频注入*************************/
	if (COMReg_Para1.Mode == 1)		//使用转矩模型进行位置估计
	{
		if (Time < 3)	//当时间小于5s时，电机先判断磁极位置
		{
			EstAngle = AngleEstimation(park1.Qs);

			pi_id1.Out = 0;		//初次调试时，可以先把电流调节器输出置为0
			pi_iq1.Out = 0;
			pi_id1.Out += 120 * cos(2 * 800 * PI_CONSTANT * Time);
		}
		else if (Time > 3 && Time < 5)
		{
			pi_id1.Out = 0;		//初次调试时，可以先把电流调节器输出置为0
			pi_iq1.Out = 0;
//			EstAngle = AngleEstimation(park1.Qs);
		}
		else if (Time > 5 && Time <= 5.002)
		{
			pi_id1.Out = 80;
			pi_iq1.Out = 0;
			if (Imax1 < park1.Ds)
				Imax1 = park1.Ds;
//			EstAngle = AngleEstimation(park1.Qs);
		}
		else if (Time > 5.002 && Time <= 5.1)
		{
			pi_id1.Out = 0;		//初次调试时，可以先把电流调节器输出置为0
			pi_iq1.Out = 0;
//			EstAngle = AngleEstimation(park1.Qs);
		}
		else if (Time > 5.1 && Time <= 5.102)
		{
			pi_id1.Out = -80;	//注入一个极性相反的电压
			pi_iq1.Out = 0;
			if (Imax2 < fabs(park1.Ds))
				Imax2 = fabs(park1.Ds);
//			EstAngle = AngleEstimation(park1.Qs);
		}
		else if (Time > 5.102 && Time <= 5.2)
		{
			pi_id1.Out = 0;		//初次调试时，可以先把电流调节器输出置为0
			pi_iq1.Out = 0;
//			EstAngle = AngleEstimation(park1.Qs);
		}
		else if (Time > 5.2)
		{
			if (Imax1 > Imax2)	//磁极极性判断
			{
				EstAngle = AngleEstimation(park1.Qs);
			}
			else
			{
				float theta;
				theta = AngleEstimation(park1.Qs) + PI_CONSTANT;
				if (theta > 2 * PI_CONSTANT)
					theta -= 2 * PI_CONSTANT;
				else if (theta < 0)
					theta += 2 * PI_CONSTANT;
				EstAngle = theta;
			}

//		pi_id1.Out = 0;		//初次调试时，可以先把电流调节器输出置为0
//		pi_iq1.Out = 0;
			pi_id1.Out += 120 * cos(2 * 800 * PI_CONSTANT * Time);
		}
		DispData1 = park1.Ds;
		DispData2 = park1.Qs;
	}
	//TODO 高频注入

	/*-----电机2-----*/
	if (COMReg_Para2.Mode == 1 || COMReg_Para2.Mode == 3)
	{
		pi_id2.Ref = COMReg_Para2.Id_Ref;
		pi_iq2.Ref = COMReg_Para2.Iq_Ref;
	}

	else if (COMReg_Para2.Mode == 2)
	{
		pi_id2.Ref = COMReg_Para2.Id_Ref;
		pi_iq2.Ref = pi_Omiga2.Out;
	}
	pi_iq2.Fbk = park2.Qs;
	pi_iq2.Kp = (float) COMReg_Para2.I_Kp * 0.001;
	pi_iq2.Ki = (float) COMReg_Para2.I_Ki * 0.00001;
	pi_iq2.Umax = UDC * ONEbySQRT3;
	pi_iq2.Umin = -UDC * ONEbySQRT3;
	PI_MACRO(pi_iq2)

	pi_id2.Fbk = park2.Ds;
	pi_id2.Kp = (float) COMReg_Para2.I_Kp * 0.001;
	pi_id2.Ki = (float) COMReg_Para2.I_Ki * 0.00001;
	pi_id2.Umax = UDC * ONEbySQRT3;
	pi_id2.Umin = -UDC * ONEbySQRT3;
	PI_MACRO(pi_id2)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
	/*-----电机1-----*/
	if (COMReg_Para1.Runcode == 1)
	{
		if (COMReg_Para1.Mode == 1 || COMReg_Para1.Mode == 2
				|| COMReg_Para1.Mode == 3 || COMReg_Para1.Mode == 5
				|| COMReg_Para1.Mode == 6 || COMReg_Para1.Mode == 7)
		{
			ipark1.Ds = pi_id1.Out;  //Ud
			ipark1.Qs = pi_iq1.Out; //Uq
		}
		else if (COMReg_Para1.Mode == 4)
		{
			ipark1.Ds = 0;
			ipark1.Qs = 10 * ramp1.Out * 60; //最大为600*0.05=30
		}

		else
		{
			ipark1.Ds = 0;
			ipark1.Qs = 0;
		}
	}

	if (COMReg_Para1.Mode == 1)
	{
		ipark1.Angle = EstAngle;
		sincos(ipark1.Angle, &ipark1.Sine, &ipark1.Cosine);
	}
	else
	{
		ipark1.Sine = park1.Sine;
		ipark1.Cosine = park1.Cosine;
	}
	IPARK_MACRO(ipark1)
	Ivector = sqrt(park1.Ds * park1.Ds + park1.Qs * park1.Qs);
	if (Ivector > Ivector_old)
		Ivector_old = Ivector;

	/*-----电机2-----*/
	if (COMReg_Para2.Runcode == 1)
	{
		if (COMReg_Para2.Mode == 1 || COMReg_Para2.Mode == 2
				|| COMReg_Para2.Mode == 3)
		{
			ipark2.Ds = pi_id2.Out;  //Ud
			ipark2.Qs = pi_iq2.Out; //Uq
		}
		else if (COMReg_Para2.Mode == 4)
		{
			ipark2.Ds = 0;
			ipark2.Qs = 10 * ramp2.Out * 60; //最大为600*0.05=30
		}
		else
		{
			ipark2.Ds = 0;
			ipark2.Qs = 0;
		}
	}
	ipark2.Sine = park2.Sine;
	ipark2.Cosine = park2.Cosine;
	IPARK_MACRO(ipark2)

	GraphData1[Cnt] = DispData1;
	GraphData2[Cnt] = DispData2;
	Cnt++;
	if (Cnt >= 500)
		Cnt = 0;
// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------

	/*-----电机1-----*/
	svgen1.Ualpha = ipark1.Alpha / (UDC * ONEbySQRT3);
	svgen1.Ubeta = ipark1.Beta / (UDC * ONEbySQRT3);
	SVGENDQ_MACRO(svgen1)

	/*-----电机2-----*/
	svgen2.Ualpha = ipark2.Alpha / (UDC * ONEbySQRT3);
	svgen2.Ubeta = ipark2.Beta / (UDC * ONEbySQRT3);
	SVGENDQ_MACRO(svgen2)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
	EPwm3Regs.CMPA.half.CMPA = 3750 * svgen2.Ta + 3750;
	EPwm2Regs.CMPA.half.CMPA = 3750 * svgen2.Tb + 3750;
	EPwm1Regs.CMPA.half.CMPA = 3750 * svgen2.Tc + 3750;	// Calculate the new PWM compare values

	EPwm6Regs.CMPA.half.CMPA = 3750 * svgen1.Ta + 3750;	//因为4、5、6管脚对应的IGBT控制主动电机
	EPwm5Regs.CMPA.half.CMPA = 3750 * svgen1.Tb + 3750;	//所以用svgen1对应（svgen1是之前写过的）
	EPwm4Regs.CMPA.half.CMPA = 3750 * svgen1.Tc + 3750;	// Calculate the new PWM compare values

	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

}   // MainISR Ends Here

//-----------CAN通讯接收中断-----------//
interrupt void CANARX_ISR(void)
{
	struct ECAN_REGS ECanaShadow;
	Uint16 CANRX_MSGID;

	ECanaShadow.CANRMP.all = ECanaRegs.CANRMP.all;
	ECanaShadow.CANRMP.bit.RMP0 = 1;   //clear  RMLIF0(接收邮箱溢出) by set RMP0
	ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;

	CANRX_MSGID = ECanaMboxes.MBOX0.MSGID.bit.STDMSGID;

	switch (CANRX_MSGID)
	{
	case 0x200:
	{
		COMReg_Para1.I_Prtct = ECanaMboxes.MBOX0.MDL.byte.BYTE0;
		COMReg_Para1.UDC_Prtct = ECanaMboxes.MBOX0.MDL.byte.BYTE1;
		COMReg_Para1.Omiga_Prtct = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
		COMReg_Para1.Temp_Warning = ECanaMboxes.MBOX0.MDL.byte.BYTE3;
		COMReg_Para1.Temp_Prtct = ECanaMboxes.MBOX0.MDH.byte.BYTE4;

		break;
	}
	case 0x201:
	{
		COMReg_Para1.Runcode = ECanaMboxes.MBOX0.MDL.byte.BYTE0;
		COMReg_Para1.Mode = ECanaMboxes.MBOX0.MDL.byte.BYTE1;
		COMReg_Para1.FreRef = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
		COMReg_Para1.switch_1 = ECanaMboxes.MBOX0.MDH.byte.BYTE4;
		COMReg_Para1.switch_2 = ECanaMboxes.MBOX0.MDH.byte.BYTE5;
		COMReg_Para1.switch_3 = ECanaMboxes.MBOX0.MDH.byte.BYTE6;
		COMReg_Para1.switch_4 = ECanaMboxes.MBOX0.MDH.byte.BYTE7;
		break;
	}
	case 0x202:
	{
		COMReg_Para1.I_Kp = (ECanaMboxes.MBOX0.MDL.byte.BYTE0 << 8)
				| ECanaMboxes.MBOX0.MDL.byte.BYTE1;
		COMReg_Para1.I_Ki = (ECanaMboxes.MBOX0.MDL.byte.BYTE2 << 8)
				| ECanaMboxes.MBOX0.MDL.byte.BYTE3;
		COMReg_Para1.Omiga_Kp = (ECanaMboxes.MBOX0.MDH.byte.BYTE4 << 8)
				| ECanaMboxes.MBOX0.MDH.byte.BYTE5;
		COMReg_Para1.Omiga_Ki = (ECanaMboxes.MBOX0.MDH.byte.BYTE6 << 8)
				| ECanaMboxes.MBOX0.MDH.byte.BYTE7;

		break;
	}
	case 0x203:
	{
		COMReg_Para1.Iq_Ref = (ECanaMboxes.MBOX0.MDL.byte.BYTE0 << 8)
				| ECanaMboxes.MBOX0.MDL.byte.BYTE1;
		COMReg_Para1.Id_Ref = (ECanaMboxes.MBOX0.MDL.byte.BYTE2 << 8)
				| ECanaMboxes.MBOX0.MDL.byte.BYTE3;
		COMReg_Para1.Omiga_Ref = (ECanaMboxes.MBOX0.MDH.byte.BYTE4 << 8)
				| ECanaMboxes.MBOX0.MDH.byte.BYTE5;
		COMReg_Para1.Pos_Differ = (ECanaMboxes.MBOX0.MDH.byte.BYTE6 << 8)
				| ECanaMboxes.MBOX0.MDH.byte.BYTE7;

		break;
	}
	case 0x204:
	{
		COMReg_Para2.Runcode = ECanaMboxes.MBOX0.MDL.byte.BYTE0;
		COMReg_Para2.Mode = ECanaMboxes.MBOX0.MDL.byte.BYTE1;
		COMReg_Para2.FreRef = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
		COMReg_Para2.switch_1 = ECanaMboxes.MBOX0.MDH.byte.BYTE4;
		COMReg_Para2.switch_2 = ECanaMboxes.MBOX0.MDH.byte.BYTE5;
		COMReg_Para2.switch_3 = ECanaMboxes.MBOX0.MDH.byte.BYTE6;
		COMReg_Para2.switch_4 = ECanaMboxes.MBOX0.MDH.byte.BYTE7;
		break;
	}
	case 0x205:
	{
		COMReg_Para2.I_Kp = (ECanaMboxes.MBOX0.MDL.byte.BYTE0 << 8)
				| ECanaMboxes.MBOX0.MDL.byte.BYTE1;
		COMReg_Para2.I_Ki = (ECanaMboxes.MBOX0.MDL.byte.BYTE2 << 8)
				| ECanaMboxes.MBOX0.MDL.byte.BYTE3;
		COMReg_Para2.Omiga_Kp = (ECanaMboxes.MBOX0.MDH.byte.BYTE4 << 8)
				| ECanaMboxes.MBOX0.MDH.byte.BYTE5;
		COMReg_Para2.Omiga_Ki = (ECanaMboxes.MBOX0.MDH.byte.BYTE6 << 8)
				| ECanaMboxes.MBOX0.MDH.byte.BYTE7;

		break;
	}
	case 0x206:
	{
		COMReg_Para2.Iq_Ref = (ECanaMboxes.MBOX0.MDL.byte.BYTE0 << 8)
				| ECanaMboxes.MBOX0.MDL.byte.BYTE1;
		COMReg_Para2.Id_Ref = (ECanaMboxes.MBOX0.MDL.byte.BYTE2 << 8)
				| ECanaMboxes.MBOX0.MDL.byte.BYTE3;
		COMReg_Para2.Omiga_Ref = (ECanaMboxes.MBOX0.MDH.byte.BYTE4 << 8)
				| ECanaMboxes.MBOX0.MDH.byte.BYTE5;
		COMReg_Para2.Pos_Differ = (ECanaMboxes.MBOX0.MDH.byte.BYTE6 << 8)
				| ECanaMboxes.MBOX0.MDH.byte.BYTE7;

		break;
	}
	case 100:
	{
		huang1 = (ECanaMboxes.MBOX0.MDL.byte.BYTE1 << 8)
				| ECanaMboxes.MBOX0.MDL.byte.BYTE0;
		huang1 = 4096 - huang1;

		break;
	}
	default:
	{
		break;
	}
	}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;   // 第九组中断响应
	IER |= M_INT9;
	EINT;
}

//-----------CAN通讯发送中断-----------//
interrupt void CANATX_ISR(void)
{
	struct ECAN_REGS ECanaShadow;

	ECanaShadow.CANTA.all = 0;
	ECanaShadow.CANTA.bit.TA1 = 1;     	         // Clear TA1
	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME1 = 0;      //禁止1号邮箱
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	CANTX_MSGID++;
	if (CANTX_MSGID > 0x10A)
		CANTX_MSGID = 0x101;

	ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = CANTX_MSGID;

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME1 = 1;      //使能1号邮箱
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	switch (CANTX_MSGID)
	{
	case 0x101:
	{
		break;
	}
	case 0x102:
	{

		if ((COMReg_Para1.Mode == 5) || (COMReg_Para1.Mode == 6)
				|| (COMReg_Para1.Mode == 7))
		{
			ECanaMboxes.MBOX1.MDL.byte.BYTE0 = ((int16) huang_omiga & 0xFF00)
					>> 8;
			ECanaMboxes.MBOX1.MDL.byte.BYTE1 = (int16) huang_omiga & 0x00FF;
			ECanaMboxes.MBOX1.MDL.byte.BYTE2 = (huang1 & 0xFF00) >> 8; //(position_elec & 0xFF00)>>8;
			ECanaMboxes.MBOX1.MDL.byte.BYTE3 = huang1 & 0x00FF;
		}
		else
		{
			ECanaMboxes.MBOX1.MDL.byte.BYTE0 = ((int16) Omiga & 0xFF00) >> 8;
			ECanaMboxes.MBOX1.MDL.byte.BYTE1 = (int16) Omiga & 0x00FF;
			ECanaMboxes.MBOX1.MDL.byte.BYTE2 = (Position & 0xFF00) >> 8; //(position_elec & 0xFF00)>>8;
			ECanaMboxes.MBOX1.MDL.byte.BYTE3 = Position & 0x00FF;
		}
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = ((Uint16) UDC & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = (Uint16) UDC & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = (Uint16) Temp_B & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = 0x0000;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

		break;
	}
	case 0x103:
	{
		ECanaMboxes.MBOX1.MDL.byte.BYTE0 = (COMReg_Para1.I_Kp & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE1 = COMReg_Para1.I_Kp & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE2 = (COMReg_Para1.I_Ki & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE3 = COMReg_Para1.I_Ki & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = (COMReg_Para1.Omiga_Kp & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = COMReg_Para1.Omiga_Kp & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = (COMReg_Para1.Omiga_Ki & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = COMReg_Para1.Omiga_Ki & 0x00FF;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

		break;
	}
	case 0x104:
	{
		ECanaMboxes.MBOX1.MDL.byte.BYTE0 = (COMReg_Para1.Iq_Ref & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE1 = COMReg_Para1.Iq_Ref & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE2 = (COMReg_Para1.Id_Ref & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE3 = COMReg_Para1.Id_Ref & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = (COMReg_Para1.Omiga_Ref & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = COMReg_Para1.Omiga_Ref & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = (COMReg_Para1.Pos_Differ & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = COMReg_Para1.Pos_Differ & 0x00FF;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

		break;
	}
	case 0x105:
	{
		ECanaMboxes.MBOX1.MDL.byte.BYTE0 = COMReg_Para1.Runcode & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE1 = COMReg_Para1.Mode & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE2 = COMReg_Para1.FreRef & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE3 = 0x01; //COMReg_Stat.StatCode;
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = COMReg_Para1.FaultCode;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = 0x0000;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = 0x0000;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = 0x0000;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

		break;
	}
	case 0x106:
	{
		ECanaMboxes.MBOX1.MDL.byte.BYTE0 = ((int) (pi_iq1.Ref) & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE1 = (int) (pi_iq1.Ref) & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE2 = ((int) (pi_id1.Ref) & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE3 = (int) (pi_id1.Ref) & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = 0;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = 0;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = 0;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = 0;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

		break;
	}
	case 0x107:
	{
		ECanaMboxes.MBOX1.MDL.byte.BYTE0 = (Position & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE1 = Position & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE2 = ((int16) (Ivector_old) & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE3 = (int16) (Ivector_old) & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = ((int16) (Ivector) & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = (int16) (Ivector) & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = (Omiga_sum & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = Omiga_sum & 0x00FF;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

		break;
	}
	case 0x108:
	{
		ECanaMboxes.MBOX1.MDL.byte.BYTE0 = ((huang1 - Position) & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE1 = (huang1 - Position) & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE2 = ((int16) (huang_omiga) & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE3 = (int16) (huang_omiga) & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = (huang1 & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = huang1 & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = ((int16) (ipark2.Qs) & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = (int16) (ipark2.Qs) & 0x00FF;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
	}

	case 0x109:
	{
		ECanaMboxes.MBOX1.MDL.byte.BYTE0 = (COMReg_Para2.I_Kp & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE1 = COMReg_Para2.I_Kp & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE2 = (COMReg_Para2.I_Ki & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE3 = COMReg_Para2.I_Ki & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = (COMReg_Para2.Omiga_Kp & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = COMReg_Para2.Omiga_Kp & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = (COMReg_Para2.Omiga_Ki & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = COMReg_Para2.Omiga_Ki & 0x00FF;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

		break;
	}
	case 0x10A:
	{
		ECanaMboxes.MBOX1.MDL.byte.BYTE0 = (COMReg_Para2.Iq_Ref & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE1 = COMReg_Para2.Iq_Ref & 0x00FF;
		ECanaMboxes.MBOX1.MDL.byte.BYTE2 = (COMReg_Para2.Id_Ref & 0xFF00) >> 8;
		ECanaMboxes.MBOX1.MDL.byte.BYTE3 = COMReg_Para2.Id_Ref & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE4 = (COMReg_Para2.Omiga_Ref & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE5 = COMReg_Para2.Omiga_Ref & 0x00FF;
		ECanaMboxes.MBOX1.MDH.byte.BYTE6 = (COMReg_Para2.Pos_Differ & 0xFF00)
				>> 8;
		ECanaMboxes.MBOX1.MDH.byte.BYTE7 = COMReg_Para2.Pos_Differ & 0x00FF;

		ECanaShadow.CANTRS.all = 0;
		ECanaShadow.CANTRS.bit.TRS1 = 1; // Set TRS for mailbox under test
		ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

		break;
	}
	default:
	{
		break;
	}
	}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;	// 第九组中断
	IER |= M_INT9;
	EINT;
}

void En1EPWM()
{
	EALLOW;

//	EPwm1Regs.TZCLR.bit.OST = 1;
//	EPwm2Regs.TZCLR.bit.OST = 1;
//	EPwm3Regs.TZCLR.bit.OST = 1;
	EPwm4Regs.TZCLR.bit.OST = 1;
	EPwm5Regs.TZCLR.bit.OST = 1;
	EPwm6Regs.TZCLR.bit.OST = 1;

	PWM_STAT1 = 1;

	EDIS;

	return;
}

void En2EPWM()
{
	EALLOW;

	EPwm1Regs.TZCLR.bit.OST = 1;				//Clear trip event
	EPwm2Regs.TZCLR.bit.OST = 1;
	EPwm3Regs.TZCLR.bit.OST = 1;

	PWM_STAT2 = 1;

	EDIS;

	return;
}

void Dis1EPWM()
{
	EALLOW;

//	EPwm1Regs.TZFRC.bit.OST = 1;
//	EPwm2Regs.TZFRC.bit.OST = 1;
//	EPwm3Regs.TZFRC.bit.OST = 1;
	EPwm4Regs.TZFRC.bit.OST = 1;
	EPwm5Regs.TZFRC.bit.OST = 1;
	EPwm6Regs.TZFRC.bit.OST = 1;

	PWM_STAT1 = 0;

	EDIS;

	return;
}

void Dis2EPWM()
{
	EALLOW;

	EPwm1Regs.TZFRC.bit.OST = 1;				//Force trip event
	EPwm2Regs.TZFRC.bit.OST = 1;
	EPwm3Regs.TZFRC.bit.OST = 1;

	PWM_STAT2 = 0;

	EDIS;

	return;
}

static void GPIO_Init(void)
{
	EALLOW;
	/*----输入输出数据口配置----*/
	GpioCtrlRegs.GPCPUD.bit.GPIO79 = 0;		// Enable pullup on GPIO79
	GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 0;		// GPIO79 = GPIO79
	GpioCtrlRegs.GPCDIR.bit.GPIO79 = 0;		// GPIO79 = input

	GpioCtrlRegs.GPCPUD.bit.GPIO78 = 0;		// Enable pullup on GPIO78
	GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 0;		// GPIO78 = GPIO78
	GpioCtrlRegs.GPCDIR.bit.GPIO78 = 0;		// GPIO78 = input

	GpioCtrlRegs.GPCPUD.bit.GPIO77 = 0;		// Enable pullup on GPIO77
	GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 0;		// GPIO77 = GPIO77
	GpioCtrlRegs.GPCDIR.bit.GPIO77 = 0;		// GPIO77 = input

	GpioCtrlRegs.GPCPUD.bit.GPIO76 = 0;		// Enable pullup on GPIO76
	GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 0;		// GPIO76 = GPIO76
	GpioCtrlRegs.GPCDIR.bit.GPIO76 = 0;		// GPIO76 = input

	GpioCtrlRegs.GPCPUD.bit.GPIO75 = 0;		// Enable pullup on GPIO75
	GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 0;		// GPIO75 = GPIO75
	GpioCtrlRegs.GPCDIR.bit.GPIO75 = 0;		// GPIO75 = input

	GpioCtrlRegs.GPCPUD.bit.GPIO74 = 0;		// Enable pullup on GPIO74
	GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 0;		// GPIO74 = GPIO74
	GpioCtrlRegs.GPCDIR.bit.GPIO74 = 0;		// GPIO74 = input

	GpioCtrlRegs.GPCPUD.bit.GPIO73 = 0;		// Enable pullup on GPIO73
	GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 0;		// GPIO73 = GPIO73
	GpioCtrlRegs.GPCDIR.bit.GPIO73 = 1;		// GPIO73 = output
	GpioDataRegs.GPCSET.bit.GPIO73 = 1;

	GpioCtrlRegs.GPCPUD.bit.GPIO72 = 0;		// Enable pullup on GPIO72
	GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 0;		// GPIO72 = GPIO72
	GpioCtrlRegs.GPCDIR.bit.GPIO72 = 1;		// GPIO72 = output
	GpioDataRegs.GPCSET.bit.GPIO72 = 1;

	GpioCtrlRegs.GPCPUD.bit.GPIO71 = 0;		// Enable pullup on GPIO71
	GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 0;		// GPIO71 = GPIO71
	GpioCtrlRegs.GPCDIR.bit.GPIO71 = 1;		// GPIO71 = output
	GpioDataRegs.GPCSET.bit.GPIO71 = 1;

	GpioCtrlRegs.GPCPUD.bit.GPIO70 = 0;		// Enable pullup on GPIO70
	GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 0;		// GPIO70 = GPIO70
	GpioCtrlRegs.GPCDIR.bit.GPIO70 = 1;		// GPIO70 = output
	GpioDataRegs.GPCSET.bit.GPIO70 = 1;

	GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;		// Enable pullup on GPIO66 outDSP5
	GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;		// GPIO66 = GPIO66
	GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;		// GPIO66 = output
	GpioDataRegs.GPCSET.bit.GPIO66 = 1;

	GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;		// Enable pullup on GPIO65 outDSP6
	GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;		// GPIO65 = GPIO65
	GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;		// GPIO65 = output
	GpioDataRegs.GPCSET.bit.GPIO65 = 1;
	/*----DAC数据口配置----*/
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;		//
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;
	/*----max7219数据口配置----*/
	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;		//
	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;
	GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 0;
	GpioCtrlRegs.GPCDIR.bit.GPIO68 = 1;
	GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 0;
	GpioCtrlRegs.GPCDIR.bit.GPIO69 = 1;
	/*----LED灯配置----*/
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;    //配置LED1的GPIO30为IO功能
	GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;     //配置GPIO30为输出

	/*----UVW信号GPIO口配置----*/
	GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;		// Enable pullup on GPIO58
	GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;		// GPIO58 = GPIO58
	GpioCtrlRegs.GPBDIR.bit.GPIO58 = 0;		// GPIO58 = input

	GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;		// Enable pullup on GPIO59
	GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;		// GPIO59 = GPIO59
	GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0;		// GPIO59 = input

	GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;		// Enable pullup on GPIO60
	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;		// GPIO60 = GPIO60
	GpioCtrlRegs.GPBDIR.bit.GPIO60 = 0;		// GPIO60 = input
	EDIS;
}

void Pos_Cal(void)                             // 计算转速、转子位置 
{
	int16 Pos_differ;

	Direction = EQep1Regs.QEPSTS.bit.QDF;		//正转为1，反转为0；
	Position = EQep1Regs.QPOSCNT;

	if (Position < 0)
		Position += Pulse_Num;
	else if (Position > Max_Pulse)
		Position -= Pulse_Num;

	//正转时每次Position为正；反转为负。即正转转速为正，反转为负
	if (Direction == 1)                         // A超前B ,正转
	{
		if ((Position >= Pos_Prev))              // 没过零位
			Pos_differ = Position - Pos_Prev;
		else
			Pos_differ = Position - Pos_Prev + Pulse_Num;      // 过了零位
	}

	if (Direction == 0)                          // A滞后B ,反转
	{
		if ((Position <= Pos_Prev))              // 没过零位
			Pos_differ = Position - Pos_Prev;
		else
			Pos_differ = Position - Pos_Prev - Pulse_Num;      // 过了零位
	}

	Pos_Prev = Position;
	Omiga_sum += Pos_differ;
	Omiga_count++;

	RealAngle = 2 * PI_CONSTANT * (Position + COMReg_Para1.Pos_Differ)
			* 1.0/ Pulse_Num;
	if (RealAngle > 2 * PI_CONSTANT)
		RealAngle -= 2 * PI_CONSTANT;
	else if (RealAngle < 0)
		RealAngle += 2 * PI_CONSTANT;

	if (Omiga_count >= 500)
	{
		//Omiga_prev=Omiga;
		RealSpeed = Omiga_sum / Pulse_Num * 2 * PI_CONSTANT / 0.05;
		Omiga = Omiga_sum * Omiga_COFF;
		Omiga_count = 0;
		Omiga_sum = 0;

		/*if(abs(Omiga-Omiga_prev)>1000)
		 Omiga=Omiga_prev;*/
	}
	return;
}

void Init_Pos(void)                            // 初始位置计算
{
	Uint16 Zone = 0;

	U_uvw = GpioDataRegs.GPBDAT.bit.GPIO58;
	V_uvw = GpioDataRegs.GPBDAT.bit.GPIO59;
	W_uvw = GpioDataRegs.GPBDAT.bit.GPIO60;

	Zone = U_uvw * 4 + V_uvw * 2 + W_uvw;

	switch (Zone)
	{
	case 1:
	{
		Pos_Zero = Pulse_Num * 5 / 6;			// UVW=001
		Pos_Error = 0;

	}
	case 2:
	{
		Pos_Zero = Pulse_Num / 2;				// UVW=010
		Pos_Error = 0;
		break;
	}
	case 3:
	{
		Pos_Zero = Pulse_Num * 2 / 3;			// UVW=011
		Pos_Error = 0;
		break;
	}
	case 4:
	{
		Pos_Zero = Pulse_Num / 6;				// UVW=100
		Pos_Error = 0;
		break;
	}
	case 5:
	{
		Pos_Zero = 0;                       // UVW=101
		Pos_Error = 0;
		break;
	}
	case 6:
	{
		Pos_Zero = Pulse_Num / 3;				// UVW=110
		Pos_Error = 0;
		break;
	}
	default:
	{
		Pos_Error = 1;
		break;
	}
	}

	if (!Pos_Error)
	{
		Pos_Zero += Pulse_Num / 12;
		if (Pos_Zero >= Pulse_Num)
			Pos_Zero -= Pulse_Num;
		if (Pos_Zero < 0)
			Pos_Zero += Pulse_Num;
		//Pos_Zero=Pos_Zero*0.333333;
		EQep1Regs.QPOSCNT = Pos_Zero;
		Pos_Prev = EQep1Regs.QPOSCNT;
	}
	return;
}

void max_7219(Uint16 data)
{
	Uint16 i, temp = 0x8000;
	GpioDataRegs.GPCCLEAR.bit.GPIO68 = 1;
	for (i = 0; i < 16; i++)
	{
		GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
		if ((temp & data) == 0)
			GpioDataRegs.GPCCLEAR.bit.GPIO69 = 1;
		else
			GpioDataRegs.GPCSET.bit.GPIO69 = 1;
		GpioDataRegs.GPCSET.bit.GPIO67 = 1;
		temp = temp >> 1;
	}
	GpioDataRegs.GPCSET.bit.GPIO68 = 1;
}

//TODO 角度估计
float AngleEstimation(float iq)
{
	float i1, i2, i3, w;
	static float theta = 0;
	i1 = DCL_runDF22(&AngEstBPF, iq);
	DispData1 = i1;
	i2 = -i1 * sin(800 * 2 * PI_CONSTANT * Time);
	i3 = DCL_runDF22(&AngEstLPF, i2);
	DispData2 = i3;
	AngEstPI.Ref = i3 * COMReg_Para1.Omiga_Ref * 0.01;
	AngEstPI.Fbk = 0;
	PI_MACRO(AngEstPI);
	w = AngEstPI.Out;
	EstSpeed = w;
	theta += w * SAMPLE_PERIOD;
	if (theta > 2 * PI_CONSTANT)
		theta -= 2 * PI_CONSTANT;
	else if (theta < 0)
		theta += 2 * PI_CONSTANT;

	return theta;
}

/*Uint16 Temp_Cal(Uint16 data)
 {
 Uint16 Temp;

 if(data <= Tempcurve.ZERO)
 {
 Temp = 0;
 }
 else if((data > Tempcurve.ZERO) && (data <= Tempcurve.PO5))
 {
 Temp = (5 * (data - Tempcurve.ZERO)) / (Tempcurve.PO5 - Tempcurve.ZERO) + 0;
 }
 else if((data > Tempcurve.PO5) && (data <= Tempcurve.PO10))
 {
 Temp = (5 * (data - Tempcurve.PO5)) / (Tempcurve.PO10 - Tempcurve.PO5) + 5;
 }
 else if((data > Tempcurve.PO10) && (data <= Tempcurve.PO15))
 {
 Temp = (5 * (data - Tempcurve.PO10)) / (Tempcurve.PO15 - Tempcurve.PO10) + 10;
 }
 else if((data > Tempcurve.PO15) && (data <= Tempcurve.PO20))
 {
 Temp = (5 * (data - Tempcurve.PO15)) / (Tempcurve.PO20 - Tempcurve.PO15) + 15;
 }
 else if((data > Tempcurve.PO20) && (data <= Tempcurve.PO25))
 {
 Temp = (5 * (data - Tempcurve.PO20)) / (Tempcurve.PO25 - Tempcurve.PO20) + 20;
 }
 else if((data > Tempcurve.PO25) && (data <= Tempcurve.PO30))
 {
 Temp = (5 * (data - Tempcurve.PO25)) / (Tempcurve.PO30 - Tempcurve.PO25) + 25;
 }
 else if((data > Tempcurve.PO30) && (data <= Tempcurve.PO35))
 {
 Temp = (5 * (data - Tempcurve.PO30)) / (Tempcurve.PO35 - Tempcurve.PO30) + 30;
 }
 else if((data > Tempcurve.PO35) && (data <= Tempcurve.PO40))
 {
 Temp = (5 * (data - Tempcurve.PO35)) / (Tempcurve.PO40 - Tempcurve.PO35) + 35;
 }
 else if((data > Tempcurve.PO40) && (data <= Tempcurve.PO45))
 {
 Temp = (5 * (data - Tempcurve.PO40)) / (Tempcurve.PO45 - Tempcurve.PO40) + 40;
 }
 else if((data > Tempcurve.PO45) && (data <= Tempcurve.PO50))
 {
 Temp = (5 * (data - Tempcurve.PO45)) / (Tempcurve.PO50 - Tempcurve.PO45) + 45;
 }
 else if((data > Tempcurve.PO50) && (data <= Tempcurve.PO55))
 {
 Temp = (5 * (data - Tempcurve.PO50)) / (Tempcurve.PO55 - Tempcurve.PO50) + 50;
 }
 else if((data > Tempcurve.PO55) && (data <= Tempcurve.PO60))
 {
 Temp = (5 * (data - Tempcurve.PO55)) / (Tempcurve.PO60 - Tempcurve.PO55) + 55;
 }
 else if((data > Tempcurve.PO60) && (data <= Tempcurve.PO65))
 {
 Temp = (5 * (data - Tempcurve.PO60)) / (Tempcurve.PO65 - Tempcurve.PO60) + 60;
 }
 else if((data > Tempcurve.PO65) && (data <= Tempcurve.PO70))
 {
 Temp = (5 * (data - Tempcurve.PO65)) / (Tempcurve.PO70 - Tempcurve.PO65) + 65;
 }
 else if((data > Tempcurve.PO70) && (data <= Tempcurve.PO75))
 {
 Temp = (5 * (data - Tempcurve.PO70)) / (Tempcurve.PO75 - Tempcurve.PO70) + 70;
 }
 else if((data > Tempcurve.PO75) && (data <= Tempcurve.PO80))
 {
 Temp = (5 * (data - Tempcurve.PO75)) / (Tempcurve.PO80 - Tempcurve.PO75) + 75;
 }
 else if((data > Tempcurve.PO80) && (data <= Tempcurve.PO85))
 {
 Temp = (5 * (data - Tempcurve.PO80)) / (Tempcurve.PO85 - Tempcurve.PO80) + 80;
 }
 else if((data > Tempcurve.PO85) && (data <= Tempcurve.PO90))
 {
 Temp = (5 * (data - Tempcurve.PO85)) / (Tempcurve.PO90 - Tempcurve.PO85) + 85;
 }
 else if((data > Tempcurve.PO90) && (data <= Tempcurve.PO95))
 {
 Temp = (5 * (data - Tempcurve.PO90)) / (Tempcurve.PO95 - Tempcurve.PO90) + 90;
 }
 else if((data > Tempcurve.PO95) && (data <= Tempcurve.PO100))
 {
 Temp = (5 * (data - Tempcurve.PO95)) / (Tempcurve.PO100 - Tempcurve.PO95) + 95;
 }
 else if((data > Tempcurve.PO100) && (data <= Tempcurve.PO105))
 {
 Temp = (5 * (data - Tempcurve.PO100)) / (Tempcurve.PO105 - Tempcurve.PO100) + 100;
 }
 else if((data > Tempcurve.PO105) && (data <= Tempcurve.PO110))
 {
 Temp = (5 * (data - Tempcurve.PO105)) / (Tempcurve.PO110 - Tempcurve.PO105) + 105;
 }
 else if((data > Tempcurve.PO110) && (data <= Tempcurve.PO115))
 {
 Temp = (5 * (data - Tempcurve.PO110)) / (Tempcurve.PO115 - Tempcurve.PO110) + 110;
 }
 else if((data > Tempcurve.PO115) && (data <= Tempcurve.PO120))
 {
 Temp = (5 * (data - Tempcurve.PO115)) / (Tempcurve.PO120 - Tempcurve.PO115) + 115;
 }
 else if((data > Tempcurve.PO120) && (data <= Tempcurve.PO125))
 {
 Temp = (5 * (data - Tempcurve.PO120)) / (Tempcurve.PO125 - Tempcurve.PO120) + 120;
 }
 else if((data > Tempcurve.PO125) && (data <= Tempcurve.PO130))
 {
 Temp = (5 * (data - Tempcurve.PO125)) / (Tempcurve.PO130 - Tempcurve.PO125) + 125;
 }
 else if((data > Tempcurve.PO130) && (data <= Tempcurve.PO135))
 {
 Temp = (5 * (data - Tempcurve.PO130)) / (Tempcurve.PO135 - Tempcurve.PO130) + 130;
 }
 else if((data > Tempcurve.PO135) && (data <= Tempcurve.PO140))
 {
 Temp = (5 * (data - Tempcurve.PO135)) / (Tempcurve.PO140 - Tempcurve.PO135) + 135;
 }
 else if((data > Tempcurve.PO140) && (data <= Tempcurve.PO145))
 {
 Temp = (5 * (data - Tempcurve.PO140)) / (Tempcurve.PO145 - Tempcurve.PO140) + 140;
 }
 else if((data > Tempcurve.PO145) && (data <= Tempcurve.PO150))
 {
 Temp = (5 * (data - Tempcurve.PO145)) / (Tempcurve.PO150 - Tempcurve.PO145) + 145;
 }
 else if(data > Tempcurve.PO150)
 {
 Temp = 150;
 }
 else
 Temp = 999;

 return Temp;

 }*/

/*float Pos_CurAngle_Cal(float is)
 {
 float CurAngle;
 float Pos_CurAngle;

 if((is >= 0) && (is < 302))
 {
 CurAngle = 68.92 - is * 0.04042553;
 }
 else if((is>=302) && (is < 486))
 {
 CurAngle = 56.71 - (is - 302) * 0.05820652;
 }
 else if((is>=486) && (is < 590))
 {
 CurAngle = 46 - (is - 486) * 0.03846154;
 }
 else if((is>=590) && (is < 648))
 {
 CurAngle = 42 - (is - 590) * 0.038;
 }
 else if((is>=648) && (is < 850))
 {
 CurAngle = 41- ( is - 648) * 0.04455;
 }
 else if(is == 850)
 {
 CurAngle = 32;
 }

 Pos_CurAngle = CurAngle*0.01745329;

 return Pos_CurAngle;

 }
 */
