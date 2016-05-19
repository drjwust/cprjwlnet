/*
 * bsp.c
 *
 *  Created on: 2015-9-26
 *  author：	 佃仁俊
 */

#include <includes.h>

static void ExtIO_Init(void);

void BSP_Init(void)
{
	InitSysCtrl();

	InitGpio();
	ExtIO_Init();
//	I2cAGpioConfig(I2C_A_GPIO0_GPIO1);
//	I2cBGpioConfig(I2C_B_GPIO134_GPIO35);
	DINT;

	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	InitPieVectTable();

	EALLOW;
	PieVectTable.ADCA1_INT = &APF_Main;	//ADC A 结束完一次序列转换，进入一次中断
	PieVectTable.EPWM2_TZ_INT = &FaultProcess;	//TZ interrupt
	PieVectTable.CANA_1_INT = CANIntHandler;
//	PieVectTable.I2CA_INT = &i2c_int1a_isr;
//	PieVectTable.I2CB_INT = &i2c_int1b_isr;
	EDIS;
	InitADC();
//	I2CA_Init();
//	I2CB_Init();

	//Power up the PWM and ADC
	EALLOW;
	CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;

	CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
	CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
	CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
	CpuSysRegs.PCLKCR13.bit.ADC_D = 1;

	//Use VREFHI as the reference for DAC
	DacaRegs.DACCTL.bit.DACREFSEL = 1;
	DacbRegs.DACCTL.bit.DACREFSEL = 1;
	DaccRegs.DACCTL.bit.DACREFSEL = 1;

	//Enable DAC output
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
	DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
	DaccRegs.DACOUTEN.bit.DACOUTEN = 1;
	EDIS;

	SetOC1Value(50);
	SetOC2Value(200);
	SetOVValue(445);

	// Configure EPWM
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

	EPwm2Regs.ETSEL.bit.SOCAEN = 1;	        // Disable SOC on A group
	EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;	  // Select SOC on up-count
	EPwm2Regs.ETPS.bit.SOCAPRD = 1;		        // Generate pulse on 1st event

	EDIS;

	InitEpwmAll();
//	InitADC();
	CAN_Init();

	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	// Enable CPU INT8 which is connected to PIE group 8
//	IER |= M_INT1;
	IER |= (M_INT1 | M_INT9);
	// Enable global Interrupts and higher priority real-time debug events:
	EINT;
	// Enable Global interrupt INTM
	ERTM;
	// Enable Global realtime interrupt DBGM

	//enable PIE interrupt
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
	// Enable I2C __interrupt 1 in the PIE: Group 8 __interrupt 1
//	PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
	PieCtrlRegs.PIEIER9.bit.INTx5 = 1;

	return;
}

void ExtIO_Init(void)
{
	//Output setup
	GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);	//D33
	GPIO_SetupPinOptions(41, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(41, 1);
	GPIO_SetupPinMux(48, GPIO_MUX_CPU1, 0);	//D34
	GPIO_SetupPinOptions(48, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(48, 1);
	GPIO_SetupPinMux(51, GPIO_MUX_CPU1, 0);	//DO3->LED1
	GPIO_SetupPinOptions(51, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(51, 0);
	GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);	//DO4->LED2
	GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(52, 0);
	GPIO_SetupPinMux(58, GPIO_MUX_CPU1, 0);	//TP50
	GPIO_SetupPinOptions(58, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(59, GPIO_MUX_CPU1, 0);	//TP51
	GPIO_SetupPinOptions(59, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(86, GPIO_MUX_CPU1, 0);	//DO1->继电器1
	GPIO_SetupPinOptions(86, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(86, 0);
	GPIO_SetupPinMux(87, GPIO_MUX_CPU1, 0);	//DO2->继电器2
	GPIO_SetupPinOptions(87, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(87, 0);

	GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);	//ExtDAC CS
	GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(32, 0);

	//Input Setup
	GPIO_SetupPinMux(53, GPIO_MUX_CPU1, 0);	//DIN1
	GPIO_SetupPinOptions(53, GPIO_INPUT, GPIO_PUSHPULL); //TODO 待定
	GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 0);	//DIN2
	GPIO_SetupPinOptions(54, GPIO_INPUT, GPIO_PUSHPULL); //TODO 待定
}

/*
 * 设置逆变器#1的过流值，单位为A
 */
void SetOC1Value(float i)
{
	DacaRegs.DACVALS.bit.DACVALS = i * 37;
}
/*
 * 设置逆变器#2的过流值，单位为A
 */
void SetOC2Value(float i)
{
	/*
	 * 传感器为CHB-200SF，变比为：200A ~ 100mA
	 * DAC REFH=3.3V，12Bit DAC
	 * 测量电阻为59欧，K= 0.1 / 200 * 59 / 3.3 * 4096 = 36.6
	 */
	DacbRegs.DACVALS.bit.DACVALS = i * 37;
}
/*
 * 设置直流母线的过压值，单位为V
 */
void SetOVValue(float u)
{
	/*
	 * 传感器为CHV-25P，变比为10mA ~ 25mA,原边电阻 60K欧;
	 * DAC REFH = 3.3V，12Bit DAC
	 * 测量电阻 59欧, K = 1 / 0.109
	 */
	DaccRegs.DACVALS.bit.DACVALS = u * 9.17;
}

void ADValueConvert(void)
{
	/*
	 * 以下电流定标适用于，电流传感器变比为2000：1，采样电阻为59欧
	 */
//	static float cali_i1ap = 0.04622, cali_i1an = 0.04584, cali_i1bp = 0.04775,
//			cali_i1bn = 0.04341;
	static float cali_i1a = 1, cali_i1b = 1;
//	static float cali_i2a = 1, cali_i2b = 1;
	static float cali_i2a = 0.271, cali_i2b = 0.269;
	static float cali_u1a = 0.0253, cali_u1b = 0.0255;
//	static float cali_u1a = 0.437, cali_u1b = 0.437;
	static float cali_u2a = 0.0253, cali_u2b = 0.0256;
//	static float cali_u2a = 1, cali_u2b = 1;

//	static float cali_udca = 1, cali_udcb = 1, cali_udcc = 01, cali_udcd = 1; //直流母线电压增益校正系数
	static float cali_udca = 0.109, cali_udcb = 0.109, cali_udcc = 0.109,
			cali_udcd = 1; //直流母线电压增益校正系数

	static uint16_t i = 0;
	static float u1a0, u2a0, u1b0, u2b0;
	static float i1a0, i2a0, i1b0, i2b0;
	float i1ap,i1an,i1bp,i1bn,i2ap,i2an,i2bp,i2bn;
	float i1a, i1b, i1c, i2a, i2b, i2c;
	float u1a, u1b, u1c, u2a, u2b, u2c;
	float udca, udcb, udcc, udcd;

	/*
	 * 必须要注意电路板的I2BP和I2BN接错了，U2A和U2B接错了，
	 * 此处已按照XXX_Adc.c中配置更正，i1a,i1b,i2a,i2b,u2a,u2b等变量与接口一致。
	 */
	i1ap = AdcaResultRegs.ADCRESULT0;
//	i1ap *= cali_i1ap;
	i1an = AdcaResultRegs.ADCRESULT1;
//	i1an *= cali_i1an; //I1A
	i1bp = AdcaResultRegs.ADCRESULT2;
//	i1bp *= cali_i1bp;
	i1bn = AdcaResultRegs.ADCRESULT3;
//	i1bn *= cali_i1bn; //I1B

	i2ap = AdcdResultRegs.ADCRESULT0;
//	i2ap *= cali_i2ap;
	i2an = AdcdResultRegs.ADCRESULT1;
//	i2an *= cali_i2an; //I2A
	i2bp = AdcdResultRegs.ADCRESULT3;
//	i2bp *= cali_i2bp;
	i2bn = AdcdResultRegs.ADCRESULT2;
//	i2bn *= cali_i2bp; //I2B

	u1a = (AdcbResultRegs.ADCRESULT3);	//U1A
	u1b = (AdccResultRegs.ADCRESULT3);	//U1B
	u2a = (AdcbResultRegs.ADCRESULT1);	//U2A
	u2b = (AdcbResultRegs.ADCRESULT0);	//U2B

	udca = (AdccResultRegs.ADCRESULT2);	//UDCA
	udcb = (AdccResultRegs.ADCRESULT1);	//UDCB
	udcc = (AdccResultRegs.ADCRESULT0);	//UDCC
	udcd = (AdcbResultRegs.ADCRESULT2);	//UDCD

	i1a = -i1ap + i1an;
	i1b = -i1bp + i1bn;
	i1c = -i1a - i1b;

	i2a = -i2ap + i2an; 	//必须要注意 电路图里面的PN画反了
	i2b = -i2bp + i2bn;
	i2c = -i2a - i2b;

	if (i < APF_SAMPLE_FREQ)
	{
		i1a0 += i1a * APF_SAMPLE_PERIOD;
		i1b0 += i1b * APF_SAMPLE_PERIOD;
		i2a0 += i2a * APF_SAMPLE_PERIOD;
		i2b0 += i2b * APF_SAMPLE_PERIOD;

		u1a0 += u1a * APF_SAMPLE_PERIOD;
		u1b0 += u1b * APF_SAMPLE_PERIOD;
		u2a0 += u2a * APF_SAMPLE_PERIOD;
		u2b0 += u2b * APF_SAMPLE_PERIOD;
		i++;
	}
	i1a = (i1a - i1a0) * cali_i1a;
	i1b = (i1b - i1b0) * cali_i1b;

	i2a = (i2a - i2a0) * cali_i2a;
	i2b = (i2b - i2b0) * cali_i2b;

//	i1b = i1bp - i1bn;
//	i1c = -i1a - i1b;
////
//	i2a = i2ap - i2an -30;
//	i2b = i2bp - i2bn;
//	i2c = -i2a - i2b;

	u1a = (u1a - u1a0) * cali_u1a;
	u1b = (u1b - u1b0) * cali_u1b;
//	u1c = -u1a - u1b;

	u2a = (u2a - u2a0) * cali_u2a;
	u2b = (u2b - u2b0) * cali_u2b;
//	u2c = -u2a - u2b;

	udca = udca * cali_udca;
	udcb = udcb * cali_udcb;
	udcc = udcc * cali_udcc;
	udcd = udcd * cali_udcd;

	/*
	 * 以上i1a,i1b等变量，与接口标号符一致。
	 */
	Upcc_ab = i2a;
	Upcc_bc = i2b;
	Upcc_ca = -Upcc_ab - Upcc_bc;

	Upcc_a = (Upcc_ab - Upcc_ca) * 0.3333333333f;
	Upcc_b = (Upcc_bc - Upcc_ab) * 0.3333333333f;
	Upcc_c = -Upcc_a - Upcc_b;

	Iapf_a = -u2a;	//AB两相的电流传感器方向不一样，逆变器内部是流向IGBT
	Iapf_b = u2b;
//	Iapf_c = i2c;

	Iload_a = u1a;
	Iload_b = u1b;
//	Iload_c = i1c;

//	UdcA = udca;
//	UdcB = udcb;
//	UdcC = udcc;

	//直线母线电压经过了一个2500HZ的二阶低通滤波器
	UdcA = DCL_runDF22(&DF22_UdcA, udca);
	UdcB = DCL_runDF22(&DF22_UdcB, udcb);
	UdcC = DCL_runDF22(&DF22_UdcC, udcc);
}

