// TI File $Revision: /main/5 $
// Checkin $Date: October 23, 2007   13:34:09 $
//###########################################################################
//
// FILE:	DSP2833x_Adc.c
//
// TITLE:	DSP2833x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include <DSP2833x_Device.h>     // DSP2833x Headerfile Include File
#include <DSP2833x_Examples.h>   // DSP2833x Examples Include File

//#define ADC_usDELAY  5000L

//---------------------------------------------------------------------------
// InitAdc:
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.

void InitAdc(void)
{
	extern void DSP28x_usDelay(Uint32 Count);
	//Configure ADC
	AdcRegs.ADCREFSEL.bit.REF_SEL = 0x1;			//外部参考2.048V电压
	AdcRegs.ADCTRL3.all = 0x00E0;
	DELAY_US(5000L);								//在转换之前需要延迟5ms
	AdcRegs.ADCTRL1.bit.CPS=1;						// 两分频
	AdcRegs.ADCTRL1.bit.ACQ_PS = 4;					// 采样窗Sequential mode: Sample rate   = 1/[(1+ACQ_PS)*ADC clock in ns]
													//= 12.5/(4+1) =2.5MHz
													// If Simultaneous mode enabled: Sample rate = 1/[(3+ACQ_PS)*ADC clock in ns]
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;
	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
	AdcRegs.ADCTRL3.bit.SMODE_SEL=0;				// 顺序采样模式
	AdcRegs.ADCTRL1.bit.SEQ_CASC=1;					// 级联模式
	AdcRegs.ADCMAXCONV.all = 0x0F;					// Setup 16 conv's
	//AdcRegs.ADCOFFTRIM.all = 0x1EE;				//设定偏置寄存器

	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;			//ADCINA0,I_V_DSP
	AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x2;			//ADCINA2,UV_2_DSP 
	AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x4;			//ADCINA4,Udc_DSP 
	AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x5; 			//ADCINA5,IpA_DSP
	AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x7;			//ADCINA7,VREF_2.048V 
	AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x8;			//ADCINB0,I_U_DSP 
	AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0xA;			//ADCINB2,VW_2_DSP 
	AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0xD;			//ADCINB5,IpB_DSP
	AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0xF;			//ADCINB7  AGND 
	AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x0; 
	AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0x2;			
	AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0x4; 
	AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0x5; 			
	AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0x8; 
	AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0xA; 			
	AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0xD;                                          
  
/* 
	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;			// Enable SOCA from ePWM to start SEQ1
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  			// Enable SEQ1 interrupt (every EOS)
*/
	return;
}


