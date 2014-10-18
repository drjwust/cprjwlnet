// TI File $Revision: /main/10 $
// Checkin $Date: August 24, 2007   11:14:01 $
//###########################################################################
//
// FILE:   DSP2833x_GlobalPrototypes.h
//
// TITLE:  Global prototypes for DSP2833x Examples
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#ifndef DSP2833x_GLOBALPROTOTYPES_H
#define DSP2833x_GLOBALPROTOTYPES_H


#ifdef __cplusplus
extern "C" {
#endif

/*---- shared global function prototypes -----------------------------------*/
extern void InitAdc(void);
extern void delay_t(unsigned);

extern void DMAInitialize(void);
// DMA Channel 1
extern void DMACH1AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source);
extern void DMACH1BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep);
extern void DMACH1TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep);
extern void DMACH1WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize, int16 deswstep);
extern void DMACH1ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot, Uint16 cont, Uint16 synce, Uint16 syncsel, Uint16 ovrinte, Uint16 datasize, Uint16 chintmode, Uint16 chinte);
extern void StartDMACH1(void);
// DMA Channel 2
extern void DMACH2AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source);
extern void DMACH2BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep);
extern void DMACH2TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep);
extern void DMACH2WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize, int16 deswstep);
extern void DMACH2ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot, Uint16 cont, Uint16 synce, Uint16 syncsel, Uint16 ovrinte, Uint16 datasize, Uint16 chintmode, Uint16 chinte);
extern void StartDMACH2(void);
// DMA Channel 3
extern void DMACH3AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source);
extern void DMACH3BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep);
extern void DMACH3TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep);
extern void DMACH3WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize, int16 deswstep);
extern void DMACH3ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot, Uint16 cont, Uint16 synce, Uint16 syncsel, Uint16 ovrinte, Uint16 datasize, Uint16 chintmode, Uint16 chinte);
extern void StartDMACH3(void);
// DMA Channel 4
extern void DMACH4AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source);
extern void DMACH4BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep);
extern void DMACH4TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep);
extern void DMACH4WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize, int16 deswstep);
extern void DMACH4ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot, Uint16 cont, Uint16 synce, Uint16 syncsel, Uint16 ovrinte, Uint16 datasize, Uint16 chintmode, Uint16 chinte);
extern void StartDMACH4(void);
// DMA Channel 5
extern void DMACH5AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source);
extern void DMACH5BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep);
extern void DMACH5TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep);
extern void DMACH5WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize, int16 deswstep);
extern void DMACH5ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot, Uint16 cont, Uint16 synce, Uint16 syncsel, Uint16 ovrinte, Uint16 datasize, Uint16 chintmode, Uint16 chinte);
extern void StartDMACH5(void);
// DMA Channel 6
extern void DMACH6AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source);
extern void DMACH6BurstConfig(Uint16 bsize,Uint16 srcbstep, int16 desbstep);
extern void DMACH6TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep);
extern void DMACH6WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize, int16 deswstep);
extern void DMACH6ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot, Uint16 cont, Uint16 synce, Uint16 syncsel, Uint16 ovrinte, Uint16 datasize, Uint16 chintmode, Uint16 chinte);
extern void StartDMACH6(void);

extern void InitPeripherals(void);
#if DSP28_ECANA
extern void InitECan(void);
extern void InitECana(void);
extern void InitECanGpio(void);
extern void InitECanaGpio(void);
#endif // endif DSP28_ECANA
#if DSP28_ECANB
extern void InitECanb(void);
extern void InitECanbGpio(void);
#endif // endif DSP28_ECANB
extern void InitECap(void);
extern void InitECapGpio(void);
extern void InitECap1Gpio(void);
extern void InitECap2Gpio(void);
#if DSP28_ECAP3
extern void InitECap3Gpio(void);
#endif // endif DSP28_ECAP3
#if DSP28_ECAP4
extern void InitECap4Gpio(void);
#endif // endif DSP28_ECAP4
#if DSP28_ECAP5
extern void InitECap5Gpio(void);
#endif // endif DSP28_ECAP5
#if DSP28_ECAP6
extern void InitECap6Gpio(void);
#endif // endif DSP28_ECAP6
extern void InitEPwm(void);
extern void InitEPwmGpio(void);
extern void InitEPwm1Gpio(void);
extern void InitEPwm2Gpio(void);
extern void InitEPwm3Gpio(void);
#if DSP28_EPWM4
extern void InitEPwm4Gpio(void);
#endif // endif DSP28_EPWM4
#if DSP28_EPWM5
extern void InitEPwm5Gpio(void);
#endif // endif DSP28_EPWM5
#if DSP28_EPWM6
extern void InitEPwm6Gpio(void);
#endif // endif DSP28_EPWM6
#if DSP28_EQEP1
extern void InitEQep(void);
extern void InitEQepGpio(void);
extern void InitEQep1Gpio(void);
#endif // if DSP28_EQEP1
#if DSP28_EQEP2
extern void InitEQep2Gpio(void);
#endif // endif DSP28_EQEP2
extern void InitGpio(void);
extern void InitI2CGpio(void);

extern void InitMcbsp(void);
extern void InitMcbspa(void);
extern void InitMcbspaGpio(void);
extern void InitMcbspa8bit(void);
extern void InitMcbspa12bit(void);
extern void InitMcbspa16bit(void);
extern void InitMcbspa20bit(void);
extern void InitMcbspa24bit(void);
extern void InitMcbspa32bit(void);
#if DSP28_MCBSPB
extern void InitMcbspb(void);
extern void InitMcbspbGpio(void);
extern void InitMcbspb8bit(void);
extern void InitMcbspb12bit(void);
extern void InitMcbspb16bit(void);
extern void InitMcbspb20bit(void);
extern void InitMcbspb24bit(void);
extern void InitMcbspb32bit(void);
#endif // endif DSP28_MCBSPB

extern void InitPieCtrl(void);
extern void InitPieVectTable(void);

extern void InitSci(void);
extern void InitSciGpio(void);
extern void InitSciaGpio(void);
#if DSP28_SCIB
extern void InitScibGpio(void);
#endif // endif DSP28_SCIB
#if DSP28_SCIC
extern void InitScicGpio(void);
#endif
extern void InitSpi(void);
extern void InitSpiGpio(void);
extern void InitSpiaGpio(void);
extern void InitSysCtrl(void);
extern void InitTzGpio(void);
extern void InitXIntrupt(void);
extern void XintfInit(void);
extern void InitXintf16Gpio();
extern void InitXintf32Gpio();
extern void InitPll(Uint16 pllcr, Uint16 clkindiv);
extern void InitPeripheralClocks(void);
extern void EnableInterrupts(void);
extern void DSP28x_usDelay(Uint32 Count);
extern void ADC_cal (void);
#define KickDog ServiceDog     // For compatiblity with previous versions
extern void ServiceDog(void);
extern void DisableDog(void);
extern Uint16 CsmUnlock(void);

// DSP28_DBGIER.asm
extern void SetDBGIER(Uint16 dbgier);

//                 CAUTION
// This function MUST be executed out of RAM. Executing it
// out of OTP/Flash will yield unpredictable results
extern void InitFlash(void);
/*
extern void InitSysCtrl(void);     //系统时钟及外设时钟初始化
extern void ServiceDog(void);      //开门狗使能
extern void DisableDog(void);      //看门狗禁止
extern void InitPll(Uint16, Uint16);  		//PLL初始化 时钟配置为30*10/2=150M
extern void InitPeripheralClocks(void);  	//外设时钟使能

extern void InitGPIO(void);        //用到的GPIO引脚初始化

extern void InitSCIC(void);	    //SCIc初始化	 		 		

extern void InitTIMER1(void);      //TIMER1初始化
extern void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period);  	//TIMER1中断周期配置

//通信相关


extern void Run(void);           //起控制函数
extern void Stop(void);          //停控制函数
extern void Display1(void);      //显示函数1
extern void Display2(void);      //显示函数2
extern void Parameter_Setting(void);       //参数配置函数
//控制相关
interrupt void cpu_timer1_int(void);

extern void UDC_PI(float Kp,float Ki);    	//PI调节函数
extern float Check_Phase(float Ua,float Ub);	//相位检测函数			

extern void CON3S2S(float,float,float);	//三相静止到两相静止(3S/2s)坐标变换,测相位
extern void ACON3S2S(float,float);		//两相静止到三相静止(2r/2s)坐标反变换
extern void Sin_Value(float );			//求正/余弦值
extern void CON2S2R_W_n(float,float,float,int); 	//两相静止到两相选转坐标变换
extern void ACON2S2R_W_n(float,float, float,int); 	//两相静止到两相选转坐标反变换
extern void Check_I_Hn(float ,float ,float,int);	//n次谐波检测

extern void DA_OUT(void);  		//DA输出函数

extern void delay_t(int a);*/


void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);


//---------------------------------------------------------------------------
// External symbols created by the linker cmd file
// DSP28 examples will use these to relocate code from one LOAD location
// in either Flash or XINTF to a different RUN location in internal
// RAM
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

extern Uint16 XintffuncsLoadStart;
extern Uint16 XintffuncsLoadEnd;
extern Uint16 XintffuncsRunStart;


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of DSP2833x_GLOBALPROTOTYPES_H

//===========================================================================
// End of file.
//===========================================================================
