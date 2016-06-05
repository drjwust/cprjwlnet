// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:19 $
//###########################################################################
//
// FILE:   DSP2833x_EPwm.c
//
// TITLE:  DSP2833x ePWM Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//---------------------------------------------------------------------------
// InitEPwm: 
//---------------------------------------------------------------------------
// This function initializes the ePWM(s) to a known state.
//
void InitEPwm(void)
{  	
   InitEPwmGpio();
   // EPwm1
   // Setup TBCLK

   EPwm1Regs.TBPRD = 7500;						//  10k frequency
   EPwm1Regs.TBPHS.half.TBPHS = 0x0;           // Phase shift is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter
      
   // Setup counter mode
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count mode     
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading, master module
   EPwm1Regs.TBCTL.bit.PRDLD=TB_SHADOW;           // use shadow register
   EPwm1Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;      // synchronization output select,output on zero
   EPwm1Regs.TBCTL.bit.HSPCLKDIV=0x0;

   // Setup shadowing
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;	  //=1直接更新，=0则根据loadmode位决定什么时候更新占空比
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero记数到0时更新占空比即比较值cmpa的值
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Set actions
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM1A on event A, up count
   EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;           // Clear PWM1A on event A, down count
   
   EPwm1Regs.DBCTL.bit.IN_MODE=0x0;                // dead band enable
   EPwm1Regs.DBCTL.bit.OUT_MODE=0x3;
   EPwm1Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;//        // polarity select
   EPwm1Regs.DBFED=450;	// 150对应1us 死区
   EPwm1Regs.DBRED=450;	// 150对应1us 死区
   EPwm1Regs.CMPA.half.CMPA = 3750;

   EPwm1Regs.ETSEL.bit.SOCAEN = 0x1;        // Enable SOC on A group
   EPwm1Regs.ETSEL.bit.SOCASEL = 0x2;       // 2--period,4--Select SOC from from CPMA on upcount
   EPwm1Regs.ETPS.bit.SOCAPRD = 0x1;        // Generate pulse on 1st event

   EALLOW;
   EPwm1Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			//TZ1 to EPWM1
   EPwm1Regs.TZSEL.bit.OSHT2 = TZ_ENABLE;          //TZ2 to EPWM1
   EPwm1Regs.TZSEL.bit.OSHT3 = TZ_ENABLE;          //TZ3 to EPWM1
   EPwm1Regs.TZSEL.bit.OSHT5 = TZ_ENABLE;
   EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			//Force A,B Output low or high使PWM封锁后输出为低。
   EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
   EPwm1Regs.TZEINT.bit.OST = 0x1;					//
   EDIS;	

  
 //  EPwm1Regs.ETSEL.bit.INTEN=1;                   // enable EPwm interrupt
 //  EPwm1Regs.ETSEL.bit.INTSEL=001;                // enable event on zero
 //  EPwm1Regs.ETPS.bit.INTPRD=01;                  // generate interrupt on the first event

   // EPwm2
   // Setup TBCLK
   EPwm2Regs.TBPRD = 7500;	// Set timer period 1600 TBCLKs
   EPwm2Regs.TBPHS.half.TBPHS = 0x0;           // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter
      
   // Setup counter mode
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up     
   EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;         // SLAVE module
   EPwm2Regs.TBCTL.bit.PRDLD=TB_SHADOW;
   EPwm2Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;
   EPwm2Regs.TBCTL.bit.HSPCLKDIV=0x0;

   // Setup shadowing
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Set actions
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM1A on event A, up count
   EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;           // Clear PWM1A on event A, down count
  
   EPwm2Regs.DBCTL.bit.IN_MODE=0x0;
   EPwm2Regs.DBCTL.bit.OUT_MODE=0x3;
   EPwm2Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;//
   EPwm2Regs.DBFED=450; // 
   EPwm2Regs.DBRED=450; // 
   EPwm2Regs.CMPA.half.CMPA = 3750;

   EALLOW;
   EPwm2Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			//TZ1 to EPWM2
   EPwm2Regs.TZSEL.bit.OSHT2 = TZ_ENABLE;          //TZ2 to EPWM2
   EPwm2Regs.TZSEL.bit.OSHT3 = TZ_ENABLE;          //TZ3 to EPWM2
   EPwm2Regs.TZSEL.bit.OSHT5 = TZ_ENABLE;
   EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			//Force A,B Output low or high
   EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
   EPwm2Regs.TZEINT.bit.OST = 0x1;					//
   EDIS;

   // EPwm3
   // Setup TBCLK
   EPwm3Regs.TBPRD = 7500;//7812;//CounterPRD;    // Set timer period 1600 TBCLKs
   EPwm3Regs.TBPHS.half.TBPHS = 0x0;           		// Phase is 0
   EPwm3Regs.TBCTR = 0x0000;                        // Clear counter
      
   // Setup counter mode
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up     
   EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // SLAVE module
   EPwm3Regs.TBCTL.bit.PRDLD=TB_SHADOW;
   EPwm3Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;
   EPwm3Regs.TBCTL.bit.HSPCLKDIV=0x0;

   // Setup shadowing
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Set actions
   EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Set PWM1A on event A, up count
   EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;            // Clear PWM1A on event A, down count
  
   EPwm3Regs.DBCTL.bit.IN_MODE=0x0;
   EPwm3Regs.DBCTL.bit.OUT_MODE=0x3;
   EPwm3Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;//
   EPwm3Regs.DBFED=450; // 
   EPwm3Regs.DBRED=450; // 
   EPwm3Regs.CMPA.half.CMPA = 3750;

   EALLOW;
   EPwm3Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			//TZ1 to EPWM3
   EPwm3Regs.TZSEL.bit.OSHT2 = TZ_ENABLE;          //TZ2 to EPWM3
   EPwm3Regs.TZSEL.bit.OSHT3 = TZ_ENABLE;          //TZ3 to EPWM3
   EPwm3Regs.TZSEL.bit.OSHT5 = TZ_ENABLE;
   EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			//Force A,B Output low or high
   EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
   EPwm3Regs.TZEINT.bit.OST = 0x1;
   EDIS;

   // EPwm4
   // Setup TBCLK
   EPwm4Regs.TBPRD = 7500;//7812;//CounterPRD;    // Set timer period 1600 TBCLKs
   EPwm4Regs.TBPHS.half.TBPHS = 0x0;           		// Phase is 0
   EPwm4Regs.TBCTR = 0x0000;                        // Clear counter
      
   // Setup counter mode
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up     
   EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // SLAVE module
   EPwm4Regs.TBCTL.bit.PRDLD=TB_SHADOW;
   EPwm4Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;
   EPwm4Regs.TBCTL.bit.HSPCLKDIV=0x0;

   // Setup shadowing
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Set actions
   EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Set PWM1A on event A, up count
   EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;            // Clear PWM1A on event A, down count
  
   EPwm4Regs.DBCTL.bit.IN_MODE=0x0;
   EPwm4Regs.DBCTL.bit.OUT_MODE=0x3;
   EPwm4Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;//
   EPwm4Regs.DBFED=450; // 3us 死区
   EPwm4Regs.DBRED=450; // 3us 死区
   EPwm4Regs.CMPA.half.CMPA = 3750;

   EPwm4Regs.ETSEL.bit.SOCAEN = 0x1;        // Enable SOC on A group
   EPwm4Regs.ETSEL.bit.SOCASEL = 0x2;       // 2--period,4--Select SOC from from CPMA on upcount
   EPwm4Regs.ETPS.bit.SOCAPRD = 0x1;        // Generate pulse on 1st event

   EALLOW;
   EPwm4Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			//TZ1 to EPWM4
   EPwm4Regs.TZSEL.bit.OSHT2 = TZ_ENABLE;          //TZ2 to EPWM4
   EPwm4Regs.TZSEL.bit.OSHT3 = TZ_ENABLE;          //TZ3 to EPWM4
   EPwm4Regs.TZSEL.bit.OSHT5 = TZ_ENABLE;
   EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			//Force A,B Output low or high
   EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
   EPwm4Regs.TZEINT.bit.OST = 0x1;
   EDIS;

  // EPwm5
   // Setup TBCLK
   EPwm5Regs.TBPRD = 7500;//7812;//CounterPRD;    // Set timer period 1600 TBCLKs
   EPwm5Regs.TBPHS.half.TBPHS = 0x0;           		// Phase is 0
   EPwm5Regs.TBCTR = 0x0000;                        // Clear counter
      
   // Setup counter mode
   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up     
   EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // SLAVE module
   EPwm5Regs.TBCTL.bit.PRDLD=TB_SHADOW;
   EPwm5Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;
   EPwm5Regs.TBCTL.bit.HSPCLKDIV=0x0;

   // Setup shadowing
   EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Set actions
   EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Set PWM1A on event A, up count
   EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;            // Clear PWM1A on event A, down count
  
   EPwm5Regs.DBCTL.bit.IN_MODE=0x0;
   EPwm5Regs.DBCTL.bit.OUT_MODE=0x3;
   EPwm5Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;//
   EPwm5Regs.DBFED=450; // 3us 死区
   EPwm5Regs.DBRED=450; // 3us 死区
   EPwm5Regs.CMPA.half.CMPA = 3750;

   EALLOW;
   EPwm5Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			//TZ1 to EPWM5
   EPwm5Regs.TZSEL.bit.OSHT2 = TZ_ENABLE;          //TZ2 to EPWM5
   EPwm5Regs.TZSEL.bit.OSHT3 = TZ_ENABLE;          //TZ3 to EPWM5
   EPwm5Regs.TZSEL.bit.OSHT5 = TZ_ENABLE;
   EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			//Force A,B Output low or high
   EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
   EPwm5Regs.TZEINT.bit.OST = 0x1;
   EDIS;

  // EPwm6
   // Setup TBCLK
   EPwm6Regs.TBPRD = 7500;//7812;//CounterPRD;    // Set timer period 1600 TBCLKs
   EPwm6Regs.TBPHS.half.TBPHS = 0x0;           		// Phase is 0
   EPwm6Regs.TBCTR = 0x0000;                        // Clear counter
      
   // Setup counter mode
   EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up     
   EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // SLAVE module
   EPwm6Regs.TBCTL.bit.PRDLD=TB_SHADOW;
   EPwm6Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;
   EPwm6Regs.TBCTL.bit.HSPCLKDIV=0x0;

   // Setup shadowing
   EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Set actions
   EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Set PWM1A on event A, up count
   EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;            // Clear PWM1A on event A, down count
  
   EPwm6Regs.DBCTL.bit.IN_MODE=0x0;
   EPwm6Regs.DBCTL.bit.OUT_MODE=0x3;
   EPwm6Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;//
   EPwm6Regs.DBFED=450; // 3us 死区
   EPwm6Regs.DBRED=450; // 3us 死区
   EPwm6Regs.CMPA.half.CMPA = 3750;

   EALLOW;
   EPwm6Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			//TZ1 to EPWM6
   EPwm6Regs.TZSEL.bit.OSHT2 = TZ_ENABLE;          //TZ2 to EPWM6
   EPwm6Regs.TZSEL.bit.OSHT3 = TZ_ENABLE;          //TZ3 to EPWM6
   EPwm6Regs.TZSEL.bit.OSHT5 = TZ_ENABLE;
   EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			//Force A,B Output low or high
   EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
   EPwm6Regs.TZEINT.bit.OST = 0x1;
   EDIS;

   return;
}
/*void InitEPwm(void)
{
   // Initialize ePWM1/2/3/4/5/6

   //tbd...
 
}
*/
//---------------------------------------------------------------------------
// Example: InitEPwmGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ePWM pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 

void InitEPwmGpio(void)
{
	InitEPwm1Gpio();
	InitEPwm2Gpio();
   	InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
	InitTzGpio();
}

void InitEPwm1Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)   
   
/* Configure ePWM-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
   
    EDIS;
}

void InitEPwm2Gpio(void)
{
   EALLOW;
	
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM3B)

/* Configure ePWM-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
   
    EDIS;
}

void InitEPwm3Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
       
/* Configure ePWM-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
	
    EDIS;
}

void InitEPwm4Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
       
/* Configure ePWM-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO5 as EPWM3B
	
    EDIS;
}

void InitEPwm5Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
       
/* Configure ePWM-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO5 as EPWM3B
	
    EDIS;
}

void InitEPwm6Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
       
/* Configure ePWM-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO5 as EPWM3B
	
    EDIS;
}

void InitTzGpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
   GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)
   GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;    // Enable pull-up on GPIO13 (TZ2)
   GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up on GPIO14 (TZ3)
   GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (TZ5)
   
/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;  // Asynch input GPIO12 (TZ1)
   GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;  // Asynch input GPIO13 (TZ2)
   GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;  // Asynch input GPIO14 (TZ3)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;  // Asynch input GPIO16 (TZ5)
  
/* Configure TZ pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be TZ functional pins.
// Comment out other unwanted lines.   
   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
   GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2 电流
   GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // Configure GPIO14 as TZ3
   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;  // Configure GPIO16 as TZ5
   EDIS;
}


//===========================================================================
// End of file.
//===========================================================================

