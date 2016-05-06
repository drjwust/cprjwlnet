//###########################################################################
// FILE:   F2837xD_Adc.c
// TITLE:  F2837xD Adc Support Functions.
//###########################################################################
// $TI Release: F2837xD Support Library v160 $
// $Release Date: Mon Jun 15 13:36:23 CDT 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File

/* 
* Set the resolution and signalmode for a given ADC. This will ensure that
* the correct trim is loaded. 
*/
void AdcSetMode(Uint16 adc, Uint16 resolution, Uint16 signalmode)
{
	Uint16 adcOffsetTrimOTPIndex; //index into OTP table of ADC offset trims
	Uint16 adcOffsetTrim; //temporary ADC offset trim
	
	//re-populate INL trim
	CalAdcINL(adc);
	
	if(0xFFFF != *((Uint16*)GetAdcOffsetTrimOTP)){
		//offset trim function is programmed into OTP, so call it

		//calculate the index into OTP table of offset trims and call
		//function to return the correct offset trim
		adcOffsetTrimOTPIndex = 4*adc + 2*resolution + 1*signalmode;
		adcOffsetTrim = (*GetAdcOffsetTrimOTP)(adcOffsetTrimOTPIndex);
	}
	else {
		//offset trim function is not populated, so set offset trim to 0
		adcOffsetTrim = 0;
	}

	//Apply the resolution and signalmode to the specified ADC.
	//Also apply the offset trim and, if needed, linearity trim correction.
	switch(adc){
		case ADC_ADCA:
			AdcaRegs.ADCCTL2.bit.RESOLUTION = resolution;
			AdcaRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
			AdcaRegs.ADCOFFTRIM.all = adcOffsetTrim;
			if(ADC_RESOLUTION_12BIT == resolution){

				//12-bit linearity trim workaround
				AdcaRegs.ADCINLTRIM1 &= 0xFFFF0000;
				AdcaRegs.ADCINLTRIM2 &= 0xFFFF0000;
				AdcaRegs.ADCINLTRIM4 &= 0xFFFF0000;
				AdcaRegs.ADCINLTRIM5 &= 0xFFFF0000;
			}
		break;
		case ADC_ADCB:
			AdcbRegs.ADCCTL2.bit.RESOLUTION = resolution;
			AdcbRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
			AdcbRegs.ADCOFFTRIM.all = adcOffsetTrim;
			if(ADC_RESOLUTION_12BIT == resolution){

				//12-bit linearity trim workaround
				AdcbRegs.ADCINLTRIM1 &= 0xFFFF0000;
				AdcbRegs.ADCINLTRIM2 &= 0xFFFF0000;
				AdcbRegs.ADCINLTRIM4 &= 0xFFFF0000;
				AdcbRegs.ADCINLTRIM5 &= 0xFFFF0000;
			}
		break;
		case ADC_ADCC:
			AdccRegs.ADCCTL2.bit.RESOLUTION = resolution;
			AdccRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
			AdccRegs.ADCOFFTRIM.all = adcOffsetTrim;
			if(ADC_RESOLUTION_12BIT == resolution){

				//12-bit linearity trim workaround
				AdccRegs.ADCINLTRIM1 &= 0xFFFF0000;
				AdccRegs.ADCINLTRIM2 &= 0xFFFF0000;
				AdccRegs.ADCINLTRIM4 &= 0xFFFF0000;
				AdccRegs.ADCINLTRIM5 &= 0xFFFF0000;
			}
		break;
		case ADC_ADCD:
			AdcdRegs.ADCCTL2.bit.RESOLUTION = resolution;
			AdcdRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
			AdcdRegs.ADCOFFTRIM.all = adcOffsetTrim;
			if(ADC_RESOLUTION_12BIT == resolution){

				//12-bit linearity trim workaround
				AdcdRegs.ADCINLTRIM1 &= 0xFFFF0000;
				AdcdRegs.ADCINLTRIM2 &= 0xFFFF0000;
				AdcdRegs.ADCINLTRIM4 &= 0xFFFF0000;
				AdcdRegs.ADCINLTRIM5 &= 0xFFFF0000;
			}
		break;
	}
}

/* 
* Loads INL trim values from OTP into the trim registers of the specified ADC.
* Use only as part of AdcSetMode function, since linearity trim correction
* is needed for some modes.
*/
void CalAdcINL(Uint16 adc)
{
	switch(adc){
		case ADC_ADCA:
			if(0xFFFF != *((Uint16*)CalAdcaINL)){
				//trim function is programmed into OTP, so call it
				(*CalAdcaINL)();
			}
			else {
				//do nothing, no INL trim function populated
			}
			break;
		case ADC_ADCB:
			if(0xFFFF != *((Uint16*)CalAdcbINL)){
				//trim function is programmed into OTP, so call it
				(*CalAdcbINL)();
			}
			else {
				//do nothing, no INL trim function populated
			}
			break;
		case ADC_ADCC:
			if(0xFFFF != *((Uint16*)CalAdccINL)){
				//trim function is programmed into OTP, so call it
				(*CalAdccINL)();
			}
			else {
				//do nothing, no INL trim function populated
			}
			break;
		case ADC_ADCD:
			if(0xFFFF != *((Uint16*)CalAdcdINL)){
				//trim function is programmed into OTP, so call it
				(*CalAdcdINL)();
			}
			else {
				//do nothing, no INL trim function populated
			}
			break;
	}
}

void SetupADC(void)
{
	Uint16 acqps;

	//determine minimum acquisition window (in SYSCLKS) based on resolution
	if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
		acqps = 300; //75ns-14
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

	//Select the channels to convert and end of conversion flag
    //ADCA
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;  //SOC0 will convert pin A4
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 5;  //SOC1 will convert pin A5
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA

    AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 1;	//enable continuous mode
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //end of SOC3 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 2;  //SOC1 will convert pin B2
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 3;  //SOC2 will convert pin B3
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 14;  //SOC3 will convert pin ADCIN14
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA

//    AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 0;	//enable continuous mode
//    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
//    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0;   //enable INT1 flag
//    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCC
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin C2
    AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin C3
    AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4;  //SOC0 will convert pin C4
    AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 15;  //SOC1 will convert pin ADCIN15
    AdccRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles

    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA


//    AdccRegs.ADCINTSEL1N2.bit.INT2CONT = 0;	//enable continuous mode
//    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
//    AdccRegs.ADCINTSEL1N2.bit.INT1E = 0;   //enable INT1 flag
//    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCD
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin D1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC0 will convert pin D2
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 3;  //SOC1 will convert pin D3
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles

    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 7; //trigger on ePWM2 SOCA

//    AdcdRegs.ADCINTSEL1N2.bit.INT2CONT = 0;	//enable continuous mode
//    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
//    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 0;   //enable INT1 flag
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

}

void InitADC(void)
{
	EALLOW;

	//write configurations
	AdcaRegs.ADCCTL2.bit.PRESCALE = 0X2; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.PRESCALE = 0X2; //set ADCCLK divider to /4
	AdccRegs.ADCCTL2.bit.PRESCALE = 0X2; //set ADCCLK divider to /4
	AdcdRegs.ADCCTL2.bit.PRESCALE = 0X2; //set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//delay for 1ms to allow ADC time to power up
	DELAY_US(1000);

	EDIS;

	SetupADC();
}
