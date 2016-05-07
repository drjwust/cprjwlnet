//###########################################################################
//
// FILE:   F2837xD_EPwm.c
//
// TITLE:  F2837xD EPwm Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v160 $
// $Release Date: Mon Jun 15 13:36:23 CDT 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//#include "F2837xD_device.h"     // F2837xD Headerfile Include File
//#include "F2837xD_Examples.h"   // F2837xD Examples Include File
//#include <APF_Define.h>
#include <includes.h>
void InitEPwmGpio(void)
{
//	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	InitEPwm4Gpio();
//	InitEPwm5Gpio();
	InitEPwm6Gpio();
	InitEPwm7Gpio();
	InitEPwm8Gpio();
//	InitEPwm9Gpio();
//	InitEPwm10Gpio();
//	InitEPwm11Gpio();
//	InitEPwm12Gpio();

}

void InitEPwm1Gpio(void)
{
	EALLOW;

	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO145 = 1;    // Disable pull-up on GPIO145 (EPWM1A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO146 = 1;    // Disable pull-up on GPIO146 (EPWM1B)

	/* Configure EPWM-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM1 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
//	GpioCtrlRegs.GPEMUX2.bit.GPIO145 = 1;   // Configure GPIO145 as EPWM1A
//	GpioCtrlRegs.GPEMUX2.bit.GPIO146 = 1;   // Configure GPIO0146 as EPWM1B

	EDIS;
}

void InitEPwm2Gpio(void)
{
	EALLOW;

	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO147 = 1;    // Disable pull-up on GPIO147 (EPWM2A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO148 = 1;    // Disable pull-up on GPIO148 (EPWM2B)

	/* Configure EPwm-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM2 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
//	GpioCtrlRegs.GPEMUX2.bit.GPIO147 = 1;   // Configure GPIO147 as EPWM2A
//	GpioCtrlRegs.GPEMUX2.bit.GPIO148 = 1;   // Configure GPIO148 as EPWM2B

	EDIS;
}

void InitEPwm3Gpio(void)
{
	EALLOW;

	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO149 = 1;    // Disable pull-up on GPIO149 (EPWM3A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO150 = 1;    // Disable pull-up on GPIO150 (EPWM3B)

	/* Configure EPwm-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM3 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
//	GpioCtrlRegs.GPEMUX2.bit.GPIO149 = 1;   // Configure GPIO149 as EPWM3A
//	GpioCtrlRegs.GPEMUX2.bit.GPIO150 = 1;   // Configure GPIO150 as EPWM3B

	EDIS;
}

void InitEPwm4Gpio(void)
{
	EALLOW;
	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO7 (EPWM4B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO151 = 1;    // Disable pull-up on GPIO151 (EPWM4A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO152 = 1;    // Disable pull-up on GPIO152 (EPWM4B)

	/* Configure EPWM-4 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM4 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B
//	GpioCtrlRegs.GPEMUX2.bit.GPIO151 = 1;   // Configure GPIO151 as EPWM4A
//	GpioCtrlRegs.GPEMUX2.bit.GPIO152 = 1;   // Configure GPIO152 as EPWM4B

	EDIS;
}

void InitEPwm5Gpio(void)
{
	EALLOW;
	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up on GPIO8 (EPWM5A)
	GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable pull-up on GPIO9 (EPWM5B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO153 = 1;    // Disable pull-up on GPIO153 (EPWM5A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO154 = 1;    // Disable pull-up on GPIO154 (EPWM5B)

	/* Configure EPWM-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM5 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B
//	GpioCtrlRegs.GPEMUX2.bit.GPIO153 = 1;   // Configure GPIO153 as EPWM5A
//	GpioCtrlRegs.GPEMUX2.bit.GPIO154 = 1;   // Configure GPIO0154 as EPWM5B

	EDIS;
}

void InitEPwm6Gpio(void)
{
	EALLOW;
	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // Disable pull-up on GPIO10 (EPWM6A)
	GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // Disable pull-up on GPIO11 (EPWM6B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO155 = 1;    // Disable pull-up on GPIO155 (EPWM6A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO156 = 1;    // Disable pull-up on GPIO156 (EPWM6B)

	/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B
	//	GpioCtrlRegs.GPEMUX2.bit.GPIO155 = 1;   // Configure GPIO155 as EPWM6A
	//	GpioCtrlRegs.GPEMUX2.bit.GPIO156 = 1;   // Configure GPIO156 as EPWM6B

	EDIS;
}

void InitEPwm7Gpio(void)
{
	EALLOW;
	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;    // Disable pull-up on GPIO12 (EPWM7A)
	GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;    // Disable pull-up on GPIO13 (EPWM7B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO157 = 1;    // Disable pull-up on GPIO157 (EPWM7A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO158 = 1;    // Disable pull-up on GPIO158 (EPWM7B)

	/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;   // Configure GPIO12 as EPWM7A
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;   // Configure GPIO13 as EPWM7B
	//	GpioCtrlRegs.GPEMUX2.bit.GPIO157 = 1;   // Configure GPIO157 as EPWM7A
	//	GpioCtrlRegs.GPEMUX2.bit.GPIO158 = 1;   // Configure GPIO158 as EPWM7B

	EDIS;
}

void InitEPwm8Gpio(void)
{
	EALLOW;
	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;    // Disable pull-up on GPIO14 (EPWM8A)
	GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;    // Disable pull-up on GPIO15 (EPWM8B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO159 = 1;    // Disable pull-up on GPIO159 (EPWM8A)
//  GpioCtrlRegs.GPFPUD.bit.GPIO160 = 1;    // Disable pull-up on GPIO160 (EPWM8B)

	/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;   // Configure GPIO14 as EPWM8A
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;   // Configure GPIO15 as EPWM8B
	//	GpioCtrlRegs.GPEMUX2.bit.GPIO159 = 1;   // Configure GPIO159 as EPWM8A
	//	GpioCtrlRegs.GPFMUX1.bit.GPIO160 = 1;   // Configure GPIO160 as EPWM8B

	EDIS;
}

void InitEPwm9Gpio(void)
{
	EALLOW;
	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPFPUD.bit.GPIO161 = 1;  // Disable pull-up on GPIO161 (EPWM9A)
	GpioCtrlRegs.GPFPUD.bit.GPIO162 = 1;  // Disable pull-up on GPIO162 (EPWM9B)

	/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPFMUX1.bit.GPIO161 = 1;   // Configure GPIO161 as EPWM9A
	GpioCtrlRegs.GPFMUX1.bit.GPIO162 = 1;   // Configure GPIO162 as EPWM9B

	EDIS;
}

void InitEPwm10Gpio(void)
{
	EALLOW;
	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPFPUD.bit.GPIO163 = 1; // Disable pull-up on GPIO163 (EPWM10A)
	GpioCtrlRegs.GPFPUD.bit.GPIO164 = 1; // Disable pull-up on GPIO164 (EPWM10B)

	/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPFMUX1.bit.GPIO163 = 1;   // Configure GPIO163 as EPWM10A
	GpioCtrlRegs.GPFMUX1.bit.GPIO164 = 1;   // Configure GPIO164 as EPWM10B

	EDIS;
}

void InitEPwm11Gpio(void)
{
	EALLOW;
	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPFPUD.bit.GPIO165 = 1; // Disable pull-up on GPIO165 (EPWM11A)
	GpioCtrlRegs.GPFPUD.bit.GPIO166 = 1; // Disable pull-up on GPIO166 (EPWM11B)

	/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPFMUX1.bit.GPIO165 = 1;   // Configure GPIO165 as EPWM11A
	GpioCtrlRegs.GPFMUX1.bit.GPIO166 = 1;   // Configure GPIO166 as EPWM11B

	EDIS;
}

void InitEPwm12Gpio(void)
{
	EALLOW;
	/* Disable internal pull-up for the selected output pins
	 for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPFPUD.bit.GPIO167 = 1; // Disable pull-up on GPIO167 (EPWM12A)
	GpioCtrlRegs.GPFPUD.bit.GPIO168 = 1; // Disable pull-up on GPIO168 (EPWM12B)

	/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPFMUX1.bit.GPIO167 = 1;   // Configure GPIO167 as EPWM12A
	GpioCtrlRegs.GPFMUX1.bit.GPIO168 = 1;   // Configure GPIO168 as EPWM12B

	EDIS;
}

void InitEPwm2()
{

	// Enable TZ1 as one shot trip sources
	EALLOW;
	EPwm2Regs.TZSEL.bit.OSHT1 = 1;
	// What do we want the TZ1 to do?
	EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
	// Enable TZ interrupt
	EPwm2Regs.TZEINT.bit.CBC = 1;
	EDIS;

	EPwm2Regs.TBPRD = Carrier_Wave_Count;       // Set timer period
	EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
	EPwm2Regs.TBCTR = 0x0000;                  // Clear counter

	// Setup TBCLK
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

//	// Setup shadow register load on ZERO
//	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
//	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
//	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
//	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set Compare values
	EPwm2Regs.CMPA.bit.CMPA = 1000;     // Set compare A value
//	EPwm2Regs.CMPB.bit.CMPB = 1000;     // Set compare B value

	// Set actions
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

//	EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM2B on Zero
//	EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;

	EPwm2Regs.AQCTLB.bit.CAU = AQ_SET;          // Set PWM2B on Zero
	EPwm2Regs.AQCTLB.bit.CAD = AQ_CLEAR;

	// Active Low PWMs - Setup Deadband
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
	EPwm2Regs.DBRED = 4000 / 5;	//4us的死区时间
	EPwm2Regs.DBFED = 4000 / 5;

	// Interrupt where we will change the Compare Values
	EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	EPwm2Regs.ETSEL.bit.INTEN = 0;                // Enable INT
	EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

}

void InitEPwmx(volatile struct EPWM_REGS *EPwmxRegs)
{
	EPwmxRegs->TBPRD = Carrier_Wave_Count;       // Set timer period
	EPwmxRegs->TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
	EPwmxRegs->TBCTR = 0x0000;                  // Clear counter

	// Setup TBCLK
	EPwmxRegs->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwmxRegs->TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	EPwmxRegs->TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
	EPwmxRegs->TBCTL.bit.CLKDIV = TB_DIV1;

	// Setup shadow register load on ZERO
	EPwmxRegs->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwmxRegs->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwmxRegs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwmxRegs->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set Compare values
	EPwmxRegs->CMPA.bit.CMPA = 3000;     // Set compare A value
//	EPwmxRegs->CMPB.bit.CMPB = 3000;     // Set Compare B value

	// Set actions
//	EPwmxRegs->AQCTLA.bit.CAD = AQ_SET;            // Set PWM1A on Zero
//	EPwmxRegs->AQCTLA.bit.CAU = AQ_CLEAR;    // Clear PWM1A on event A, up count
	EPwmxRegs->AQCTLA.bit.CAD = AQ_CLEAR;            // Set PWM1A on Zero
	EPwmxRegs->AQCTLA.bit.CAU = AQ_SET;    // Clear PWM1A on event A, up count

//	EPwmxRegs->AQCTLB.bit.CAD = AQ_CLEAR;            // Set PWM1A on Zero
//	EPwmxRegs->AQCTLB.bit.CAU = AQ_SET;    // Clear PWM1A on event A, up count
	EPwmxRegs->AQCTLB.bit.CAD = AQ_SET;            // Set PWM1A on Zero
	EPwmxRegs->AQCTLB.bit.CAU = AQ_CLEAR;    // Clear PWM1A on event A, up count

	// Active Low PWMs - Setup Deadband
	EPwmxRegs->DBCTL.bit.IN_MODE = DBA_ALL;
	EPwmxRegs->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwmxRegs->DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwmxRegs->DBCTL.bit.IN_MODE = DBA_ALL;
	EPwmxRegs->DBRED = 2000 / 5;	//4us的死区时间
	EPwmxRegs->DBFED = 2000 / 5;

	EALLOW;
//	EPwmxRegs->TZSEL.bit.OSHT1 = TZ_ENABLE;			//TZ1 to EPWM1
//	EPwmxRegs->TZSEL.bit.OSHT2 = TZ_ENABLE;         //TZ2 to EPWM1
//	EPwm1Regs.TZSEL.bit.OSHT3 = TZ_ENABLE;          //TZ3 to EPWM1
//	EPwm1Regs.TZSEL.bit.OSHT5 = TZ_ENABLE;
	EPwmxRegs->TZCTL.bit.TZA = TZ_FORCE_LO;//Force A,B Output low or high使PWM封锁后输出为低。
	EPwmxRegs->TZCTL.bit.TZB = TZ_FORCE_LO;
//	EPwmxRegs->TZCTL2.bit.ETZE = 1;
	EDIS;

	// Interrupt where we will change the Compare Values
	EPwmxRegs->ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	EPwmxRegs->ETSEL.bit.INTEN = 0;                // Enable INT
	EPwmxRegs->ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

}

void InitTzGpio(void)
{
	// For External Trigger, GPIO12 as the trigger for TripZone
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)

	GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;  // Asynch input GPIO12 (TZ1)

	EALLOW;
	InputXbarRegs.INPUT1SELECT = 12;
	EDIS;
}

void InitEpwmAll(void)
{
//	InitTzGpio();
	InitEPwmGpio();
	InitEPwmx(&EPwm2Regs);
	InitEPwmx(&EPwm3Regs);
	InitEPwmx(&EPwm4Regs);
	InitEPwmx(&EPwm6Regs);
	InitEPwmx(&EPwm7Regs);
	InitEPwmx(&EPwm8Regs);
}
