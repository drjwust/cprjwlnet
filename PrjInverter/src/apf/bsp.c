/*
 * bsp.c
 *
 *  Created on: 2015-9-26
 *  author：	 佃仁俊
 */

#include <includes.h>

void BSP_Init(void) {
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F2837xD_SysCtrl.c file.
	InitSysCtrl();

	// Step 2. Initialize GPIO:
	// This example function is found in the F2837xD_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
	InitGpio();
//	I2cAGpioConfig(I2C_A_GPIO0_GPIO1);
//	I2cBGpioConfig(I2C_B_GPIO134_GPIO35);

	// Step 3. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	DINT;

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the F2837xD_PieCtrl.c file.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F2837xD_DefaultIsr.c.
	// This function is found in F2837xD_PieVect.c.
	InitPieVectTable();

	EALLOW;
	PieVectTable.ADCA1_INT = &APF_Main;	//ADC A 结束完一次序列转换，进入一次中断
	PieVectTable.EPWM2_TZ_INT = &FaultProcess;	//TZ interrupt
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

	// Configure EPWM
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

	EPwm2Regs.ETSEL.bit.SOCAEN = 1;	        // Disable SOC on A group
	EPwm2Regs.ETSEL.bit.SOCASEL = 2;	        // Select SOC on up-count
	EPwm2Regs.ETPS.bit.SOCAPRD = 1;		        // Generate pulse on 1st event

	EDIS;

	InitEpwmAll();

	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	// Enable CPU INT8 which is connected to PIE group 8
	IER |= M_INT1;
	// Enable global Interrupts and higher priority real-time debug events:
	EINT;
	// Enable Global interrupt INTM
	ERTM;
	// Enable Global realtime interrupt DBGM

	//enable PIE interrupt
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
	// Enable I2C __interrupt 1 in the PIE: Group 8 __interrupt 1
	PieCtrlRegs.PIEIER8.bit.INTx1 = 1;

	return;
}

/*
 * 设置逆变器#1的过流值，单位为A
 */
void SetOC1Value(float i) {
	DacaRegs.DACVALS.bit.DACVALS = i * 4096 / 100;
}
/*
 * 设置逆变器#2的过流值，单位为A
 */
void SetOC2Value(float i) {
	/*
	 * 传感器为CHB-200SF，变比为：200A ~ 100mA
	 * DAC REFH=3.3V，12Bit DAC
	 * 测量电阻为29.5欧
	 */
	DacbRegs.DACVALS.bit.DACVALS = i * 18.3078;
}
/*
 * 设置直流母线的过压值，单位为V
 */
void SetOVValue(float u) {
	/*
	 * 传感器为CHV-25P，变比为10mA ~ 25mA,原边电阻 120K欧;
	 * DAC REFH = 3.3V，12Bit DAC
	 * 测量电阻 59欧
	 */
	DaccRegs.DACVALS.bit.DACVALS = u * 1.5515;
}

void ADValueConvert(void) {
	static float cali_i1a = 0.5, cali_i1b = 0.5;
	static float cali_i2a = 0.05462, cali_i2b = 0.05462;
	static float cali_u1a = 1, cali_u1b = 1;
	static float cali_u2a = 0.437, cali_u2b = 0.437;
	static float cali_udca = 0.6447, cali_udcb = 0.6447, cali_udcc = 0.6447, cali_udcd = 1;//直流母线电压增益校正系数

	static uint16_t i;
	static float u1a0,u2a0,u1b0,u2b0;
	float i1a, i1b,i1c, i2a, i2b, i2c;
	float u1a, u1b, u1c, u2a, u2b, u2c;
	float udca, udcb, udcc, udcd;

	i1a = (int)AdcaResultRegs.ADCRESULT0 - (int)AdcaResultRegs.ADCRESULT1; //I1A
	i1b = (int)AdcaResultRegs.ADCRESULT2 - (int)AdcaResultRegs.ADCRESULT3; //I1B

	i2a = (int)AdcdResultRegs.ADCRESULT0 - (int)AdcdResultRegs.ADCRESULT1; //I2A
	i2b = (int)AdcdResultRegs.ADCRESULT2 - (int)AdcdResultRegs.ADCRESULT3; //I2B

	u1a = (AdcbResultRegs.ADCRESULT3);	//U1A
	u1b = (AdccResultRegs.ADCRESULT3);	//U1B
	u2a = (AdcbResultRegs.ADCRESULT0);	//U2A
	u2b = (AdcbResultRegs.ADCRESULT1);	//U2B

	udca = (AdcbResultRegs.ADCRESULT2);	//UDCA
	udcb = (AdccResultRegs.ADCRESULT2);	//UDCB
	udcc = (AdccResultRegs.ADCRESULT1);	//UDCC
	udcd = (AdccResultRegs.ADCRESULT0);	//UDCD

	if (i <= APF_SWITCH_FREQ)
	{
		u1a0 += u1a * APF_SWITCH_PERIOD;
		u1b0 += u1b * APF_SWITCH_PERIOD;
		u2a0 += u2a * APF_SWITCH_PERIOD;
		u2b0 += u2b * APF_SWITCH_PERIOD;
		i++;
	}
//	i1a = (i1a - offset[0]) * cali_i1a;
//	i1b = (i1b - offset[1]) * cali_i1b;
	i1c = -i1a - i1b;
//
//	i2a = (i2a - offset[2]) * cali_i2a;
//	i2b = (i2b - offset[3]) * cali_i2b;
	i2c = -i2a - i2b;

	u1a = (u1a - u1a0) * cali_u1a;
	u1b = (u1b - u1b0) * cali_u1b;
	u1c = -u1a - u1b;

	u2a = (u2a - u2a0) * cali_u2a;
	u2b = (u2b - u2b0) * cali_u2b;
	u2c = -u2a - u2b;

	udca = udca * cali_udca;
	udcb = udcb * cali_udcb;
	udcc = udcc * cali_udcc;
	udcd = udcd * cali_udcd;


	Upcc_ab = u2a;
	Upcc_bc = u2b;
	Upcc_ca = u2c;

	Upcc_a = (Upcc_ab - Upcc_ca) * 0.3333333333f;
	Upcc_b = (Upcc_bc - Upcc_ab) * 0.3333333333f;
	Upcc_c = -Upcc_a - Upcc_b;

	Iapf_a = i2a;
	Iapf_b = i2b;
	Iapf_c = i2c;

	Iload_a = i1a;
	Iload_b = i1b;
	Iload_c = i1c;

	UdcA = udca;
	UdcB = udcb;
	UdcC = udcc;

}

