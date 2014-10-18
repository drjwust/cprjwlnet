/*
 Author: Bofoster Han
 2011.04.18 三相四线制APF程序开始调试过程。在实验室的工作下，已经做了基本的定标：1V对应程序中的电压值1，1A对应程序中电流值100(Rated:25A)
 2011.04.23 将程序的电流闭环修改为alpha-beta-0坐标系下。
 2011.04.25 更换零线电抗器后，效果增加，问题不大。
 2011.04.26 增加故障判断和保护功能，以及增加限幅控制的功能。
 2011.04.29 验证了电流限幅控制和自重启控制算法，并简单修改了FFT分析算法。需要考虑故障显示和便于武新方面修改的内容。
 2011.05.06 完成了无功补偿的初步设计，在全补偿情况下同时补偿无功内容。是基于ABC相电压定向（注意电流环参数的问题）
 */

#include "APF_Variable.h"
#include "APF_Constant.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "math.h"
//080528 14：03 现在的文件需要.h 文件的支持 及控制变量和常量等函数

#define AD_register (unsigned int *)0x4000     //AD存放结果的起始地址,将0x4000强制转换成地址指针并用宏定义
#define DA_register (unsigned int *)0x4008     //DA存放结果的起始地址
#define DRAM_data   (int *) 0x100000 
//截止频率700Hz,采样频率9.375kHz，离散化方法为双线性变换法的二阶低通滤波器常数
#define F_2nd_Cnst1  1.352 
#define F_2nd_Cnst2  0.5155 
#define F_2nd_Cnst3  0.04094 
//*********************************定义掉电自重启参数**************

#define Power_Fail      ( Fault_flag < 516 ) //定义掉电重启故障信息~40%  //1540 * 0.4
#define Power_Low       ( Fault_flag >= 516 && Fault_flag < 955 )//欠电压40%~75%
#define HystisRing_Low  ( Fault_flag >= 955 && Fault_flag < 1109 )//	欠电压滞环75%~85%
#define Power_Normal    ( Fault_flag >= 1109 && Fault_flag < 1571 ) //定义供电正常85%~115%
#define HystisRing_Over ( Fault_flag >= 1571 && Fault_flag < 1720 )//过电压滞环115%~125%
#define Power_Over      ( Fault_flag >= 1720 ) //过电压125%~
#define L_Power_Fail      ( L_Fault_flag < 516 ) //定义掉电重启故障信息
#define L_Power_Low       ( L_Fault_flag >= 516 && L_Fault_flag < 955 )//欠电压
#define L_Power_Over      ( L_Fault_flag >= 1720 ) //过电压
//*********************************END******************************

#define UA_Cal   0.405275//0.426453//0.41031   //0.44//1.4028//1.184
#define UB_Cal   0.409048//0.426571//0.44//1.4028//1.184
#define UC_Cal   0.397728//0.44//1.4028//1.184
#define Current_Cal  0.1//1.526624;//25A对应4V，对应数字量1637.6。
//化为实际值十倍后，乘以系数1/0.65504	//1.66666666;
#define InterCycleFeedGain  1//2.5//2.5   //内环反馈通道增益
#define VolComdGain   0.5//1  //   稳压指令增益
#define VolErrorGain  0.5 //直流侧电压误差增益
#define SeptCompAhead      3     //分次补偿的超前点数
#define ModuWaveLimit_Low  200//430//调制波限幅（低）,该值的确定与死区时间、PWM开关周期和三角波的幅值有关。
#define ModuWaveLimit_High 5800//调制波限幅（高）,该值的确定与死区时间、PWM开关周期和三角波的幅值有关。
#define Tri_wave_amp       3000//4000 //三角载波幅值
#define VolReguPI_Limit    5000 //稳压PI调节器输出限幅
#define HarmonicComdLimit  3500 //谐波指令限幅
#define UDC_PI_Restrict    1000 //电压环PI调节器限幅
#define ID_MAXOUT          500  //电压环积分项限幅
#define AngleCountGain     1//0.711111//0~360°
//划分为256个点，不同角度点的指令可以存储在不同的数组中，增益为256/360
#define AllAngleCount      360// 360  
//*************************超前相关******************** ***********
#define AllSampleCount     250//187  //一个周期的采样点数
const float RprocalValue = 0.004;					//0.005333333; //采样点的总数的倒数
#define AddSampleCount     6    //同时扩展一个周期的采样点数和超前点数？？
//该一阶带阻滤波器对应的截止频率40和2500，采样频率12.5k.离散化方法为双线性变换。
const float FilterCnstA_1st_A = 1.27600000;
const float FilterCnstB_1st_A = 0.2903000;
const float FilterCnstC_1st_A = 1.00000000;
const float FilterCnstD_1st_A = -1.98300000;
const float FilterCnstE_1st_A = 0.9971000;

//该二阶低通滤波器对应截止频率20Hz.采样频率12.5K。离散化方法为时延。
const float FilterCnstA_2nd_A = 1.986;							//0.6203;
const float FilterCnstB_2nd_A = 0.9859;							//0.0;
const float FilterCnstC_2nd_A = 0.0;						////1627;// 0.3797;
const float FilterCnstD_2nd_A = 0.00005029;							//;
const float FilterCnstE_2nd_A = 0.00005006;							//0.0;

//********************结束*****************************
//初始化配置
#define DSP28_PLLCR    10
#define DSP28_DIVSEL   2

struct CPUTIMER_VARS CpuTimer1;
void InitSysCtrl(void);     //系统时钟及外设时钟初始化
void ServiceDog(void);      //开门狗使能
void DisableDog(void);      //看门狗禁止
void InitPll(Uint16, Uint16);  		//PLL初始化 时钟配置为30*10/2=150M
void InitPeripheralClocks(void);  	//外设时钟使能
void InitGPIO(void);        //用到的GPIO引脚初始化
void InitSCIC(void);	    //SCIC初始化
void InitEPWM(void);		//PWM初始化
void InitTIMER1(void);      //TIMER1初始化
void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period); //TIMER1中断周期配置

void InitFlash(void);           	//FLASH使能

#pragma CODE_SECTION(InitFlash, "ramfuncs");
#pragma CODE_SECTION(epwm1_timer_isr, "ramfuncs");
#pragma CODE_SECTION(UDC_PI, "ramfuncs");
#pragma CODE_SECTION(Current_PI,"ramfuncs");
#pragma CODE_SECTION(Check_Phase, "ramfuncs");
#pragma CODE_SECTION(Check_Theta, "ramfuncs");
#pragma CODE_SECTION(CON3S2S, "ramfuncs");
#pragma CODE_SECTION(ACON3S2S, "ramfuncs");
#pragma CODE_SECTION(Sin_Value, "ramfuncs");
#pragma CODE_SECTION(ACON2S2R_W, "ramfuncs");
#pragma CODE_SECTION(CON2S2R_W, "ramfuncs");
#pragma CODE_SECTION(ACON2S2R_W_3, "ramfuncs");
#pragma CODE_SECTION(CON2S2R_W_3, "ramfuncs");
//#pragma CODE_SECTION(Check_I_H1, "ramfuncs");
//#pragma CODE_SECTION(Check_I_H2, "ramfuncs");
//#pragma CODE_SECTION(Check_I_H3, "ramfuncs");
//#pragma CODE_SECTION(Check_I_H4, "ramfuncs");
#pragma CODE_SECTION(Check_Harmonic_N, "ramfuncs");
#pragma CODE_SECTION(Check_Harmonic_N_S, "ramfuncs");
#pragma CODE_SECTION(CON2S2R_W_n, "ramfuncs");
#pragma CODE_SECTION(ACON2S2R_W_n, "ramfuncs");
#pragma CODE_SECTION(EN_PWM, "ramfuncs");
#pragma CODE_SECTION(DIS_PWM, "ramfuncs");
//#pragma CODE_SECTION(DA_OUT, "ramfuncs");
#pragma CODE_SECTION(COM_main, "ramfuncs");
#pragma CODE_SECTION(Read_date, "ramfuncs");
#pragma CODE_SECTION(Display, "ramfuncs"); 
//#pragma CODE_SECTION(LP_Filter_2nd, "ramfuncs"); 
//#pragma CODE_SECTION(LP_Filter_2nd_A, "ramfuncs"); 
//#pragma CODE_SECTION(BP_Filter_1st_A, "ramfuncs");
#pragma CODE_SECTION(Rectifier, "ramfuncs");
#pragma CODE_SECTION(Display_Run_Stop, "ramfuncs");

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
void MemCopy(Uint16 *SourceAddr, Uint16 *SourceEndAddr, Uint16 *DestAddr);

//通信相关
void COM_main(void);      //显示主程序
void Read_date(int ad, int n); //读取数据，起始地址ad,数据量n
void Para_set1(void);
void Para_set2(void);
void Para_set3(void);
void Parameter_Setting(void);       //参数配置函数
void Display(void);
void Display_Para(void);
void Display_Send_Ready(int, int);
void Para_set_end(int ad, int n);
//控制相关
interrupt void epwm1_timer_isr(void);

void UDC_PI(float Kp, float Ki);    	//电压PI调节函数
void Current_PI(float, float, float, float);    	//电流PI调节函数
float Check_Phase(float Ua, float Ub);	//相位检测函数
float Check_Theta(float, float);	//仅查询角度

void Compensate_shift(void);	//自动零漂补偿(没有加入实际动作)
void Clear_Comd(void);	//程序停止运行时清除补偿指令

void CON3S2S(float, float, float);	//三相静止到两相静止(3S/2s)坐标变换,测相位
void ACON3S2S(float, float);		//两相静止到三相静止(2r/2s)坐标反变换
void CON3S3S(float, float, float);	//三相到三相的变换
void ACON3S3S(float, float, float);	//三相到三相的反变换

void Sin_Value(float);			//求正/余弦值
void ACON2S2R_W(float Um, float Ut, float Theta);         //两相静止到两相选转坐标反变换
void CON2S2R_W(float U_alpha, float U_beta); //两相静止到两相选转坐标变换
void Display_CON3S2S(float, float, float); //计算电网电压显示时用到的3S/2S变换
void Check_I_Hn(float, float, float, int);	//n次谐波检测(没有用到)
//void Check_I_H1_Passive(float temp1,float temp2,float theta_t,int n); //检测基波无功，n=1
void Check_I(float Ia, float Ib);		//谐波检测
//void Check_I_H1(float, float, int n);
//void Check_I_H2(float, float, int n);
//void Check_I_H3(float, float, int n);
//void Check_I_H4(float, float, int n);
void Check_Harmonic_N(int, int, float *);		//n次谐波检测
void Check_Harmonic_N_S(int, float, float *);		//电网n次谐波检测
//float BP_Filter_1st_A(float *, float *);
void CON2S2R_W_n(float, float, int);
void ACON2S2R_W_n(float, float, int);
void CON2S2R_W_3(float, float);
void ACON2S2R_W_3(float, float);
float Rectifier(float, float, float);		//电压整流值
float LP_Filter_2nd(float *y, float *x);
void EN_PWM(void);
void DIS_PWM(void);
//void DA_OUT(void);

//通用函数

void delay_t(int); 	 //100大约是8个us/频150M
void delay_t1(int);

//*******************************eeprom*******************************
const unsigned Version_No = 0x0001; 	 ///程序版本号,占用一个字节
//**************EEPROM操作时产生的各种错误标记**************
//unsigned RD_EEPROM_Fail_Sign = 0;//读EEPROM错标志
//unsigned WR_EEPROM_Fail_Sign = 0;//读EEPROM错标志
unsigned EEPROM_No_WRed_Sign = 0; 	 //EEPROM未被写标志
unsigned Version_Erro_Sign = 0; //版本错标志

int MsgBuffer_Send[15] =
{ 0 }; //将参数值（一帧）写入EEPROM时被用到
int MsgBuffer_Receive[16] =
{ 0 }; //从EEPROM中接收到的一帧有效数据被到这里
void RD_DATA_PROCESS(unsigned Byte_Sum);
void WR_DATA_PROCESS(unsigned Word_Sum);
void RD_EEPROM(unsigned Byte_Sum, unsigned Offset_Addr, unsigned Page_Addr);
void WR_EEPROM(unsigned Byte_Sum, unsigned Offset_Addr, unsigned Page_Addr);
void Restore_Factory_Set(void);
void InitI2C(void);

void Display_Run_Stop(int);
//*********************end********************************

//float r;

void main(void)
{
//	Uint16 SendChar;

	DINT;
	//系统时钟初始化
	InitSysCtrl();
// Copy time critical code and Flash setup code to RAM
// This includes the following ISR functions: epwm1_timer_isr(), epwm2_timer_isr()
// epwm3_timer_isr and and InitFlash();
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the F28335.cmd file.
//	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
//	InitFlash();
	//初始化中断向量表
	InitPieVectTable();      //PIE 已经ENABLE
	//初始化外部中断
	EnableInterrupts();          //PIE直接采用例程中的文件 ENable the int12.xint7
	//重新定义程序中用到的中断
	EALLOW;
	PieVectTable.EPWM1_INT = &epwm1_timer_isr;
	EDIS;
	IER |= M_INT3;
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;
	EALLOW;
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD; // Enable event time-base counter equal to period
	EPwm1Regs.ETSEL.bit.INTEN = 1;  				// Enable INT
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           	// Generate INT on 1st event
	EDIS;
	InitEPWM();           	//初始化PWM
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;
	//InitTIMER1();//初始化TIMER1
	//初始化GPIO
	InitGPIO();
	//初始化I2CGPIO
	InitI2CGpio();
	//初始化I2C
	InitI2C();

	// Initalize SCI 
	InitSCIC();
	GpioDataRegs.GPBSET.bit.GPIO57 = 1;    //保护信号复位
	delay_t(1);    //   1/12 us
	GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;    //禁止保护信号复位
	//AD中8通道使能
	//GpioDataRegs.GPACLEAR.bit.GPIO24=1;  
	delay_t(1);
	*AD_register = 0xFF;
	//GpioDataRegs.GPASET.bit.GPIO24=1;
	//DisPWM;此处需要设置相关的DIS信号
	GpioDataRegs.GPBSET.bit.GPIO56 = 1;

	EINT;
	IFR = 0x0000;
	//参数初始化现在的参数直接从constant文件读取，之后要从E2PROM中读取
	/*UDCREF=UDCREF_initial;
	 UDC_Max=UDC_Max_initial;
	 Ih_Max=Ih_Max_initial;
	 C_way=C_way_initial;
	 UDC_Kp=UDC_Kp_initial;
	 UDC_Ki=UDC_Ki_initial;
	 U_FtoI=U_FtoI_initial;
	 UDC_Rf1=UDC_Rf2=Udc_slip=UABC_initial;*/
	//*****************************eeprom************************
	//读EEPROM中数据，校验EEPROM是否为空，版本号是否正确
//	Restore_Factory_Set();//首先恢复一下出场设置，以后要在触摸屏上做
	RD_EEPROM(4, 0x00, 0x4);
	RD_DATA_PROCESS(4);	//处理4个字节
	if (MsgBuffer_Receive[0] != 0x55AA)	//EEPROM为空
	{
		EEPROM_No_WRed_Sign = 1;
		Restore_Factory_Set();
	}
	if (MsgBuffer_Receive[1] != Version_No)	//程序版本不正确
	{
		Version_Erro_Sign = 1;
		Restore_Factory_Set();
	}
//	读出EEROM 0x0页，页内偏移地址为0x10~0x1F和0x20~0x23中的所用内容,并做相应的限幅处理
	RD_EEPROM(16, 0x10, 0x4);	//读0x0页偏移地址0x10
	//RD_EEPROM(16,UDCREF_Ref.Rom_O_Addr,UDCREF_Ref.Rom_P_Addr);//读0x0页偏移地址0x10
	RD_DATA_PROCESS(16);
	if (MsgBuffer_Receive[0] >= UDCREF_initial) //直流侧电压限幅
	{
		UDCREF = UDCREF_initial;
	}
	else
	{
		UDCREF = MsgBuffer_Receive[0];
	}

	if (MsgBuffer_Receive[1] >= UDC_Max_initial) //直流侧过压限幅
	{
		UDC_Max = UDC_Max_initial;
	}
	else
	{
		UDC_Max = MsgBuffer_Receive[1];

	}

	if (MsgBuffer_Receive[2] >= Ih_Max_initial) //谐波电流限幅
	{
		Ih_Max = Ih_Max_initial;
	}
	else
	{
		Ih_Max = MsgBuffer_Receive[2];
	}

	C_way = MsgBuffer_Receive[3];
	UDC_Kp = MsgBuffer_Receive[4];
	UDC_Ki = MsgBuffer_Receive[5];
	U_FtoI = MsgBuffer_Receive[6]; //抵消电网电压
	//UDC_Rf1 = MsgBuffer_Receive[7];

	/*	RD_EEPROM(2,0x1E,0x4);
	 RD_DATA_PROCESS(2);
	 C_Way1 = MsgBuffer_Receive[0];

	 RD_EEPROM(2,0x2E,0x4);
	 RD_DATA_PROCESS(2);
	 C_Way2 = MsgBuffer_Receive[0];

	 RD_EEPROM(2,0x3E,0x4);
	 RD_DATA_PROCESS(2);
	 C_Way3 = MsgBuffer_Receive[0];*/

	RD_EEPROM(6, 0x20, 0x4);	//读0x0页偏移地址0x20
	RD_DATA_PROCESS(6);	// 0x0-0x22
	/*	if(MsgBuffer_Receive[0] >= UABC_initial)
	 {
	 UDC_Rf2 = UABC_initial;
	 }
	 else
	 {
	 UDC_Rf2 = MsgBuffer_Receive[0];
	 } */
//	Udc_slip = MsgBuffer_Receive[1];
	PowerFailRecord = L_PowerFailRecord = MsgBuffer_Receive[0];	//读出上次电网故障信息
	Reboot_Mode_Set = L_Reboot_Mode_Set = MsgBuffer_Receive[1]; // 读出用户是否设置重启信息
//	L_ERROR_flag = MsgBuffer_Receive[2];//从EEPROM中读出故障信号
//	Ltime_Lack_Phase = (L_ERROR_flag & 0x10);//提取上次缺相信息

	//***************2009.5.31日添加************
	RD_EEPROM(16, 0x40, 0x4); //读0x0页偏移地址0x40为首地址的16个字节->8个16位字
	RD_DATA_PROCESS(16);
	COM_ALL_rate = MsgBuffer_Receive[0];
	COM_EXT_count = MsgBuffer_Receive[1];
	COM_IHB_RATE = MsgBuffer_Receive[2];
	COM_IH1 = MsgBuffer_Receive[3];
	COM_IH2 = MsgBuffer_Receive[4];
	COM_IH3 = MsgBuffer_Receive[5];
	COM_Ua = MsgBuffer_Receive[6];
	//COM_Tu = MsgBuffer_Receive[7]; 
	COM_IH4 = MsgBuffer_Receive[7];

	RD_EEPROM(14, 0x50, 0x4); //读0x0页偏移地址0x50
	RD_DATA_PROCESS(14);
//  n_cycle不必从EEPROM中读出，可以由n_cycle=250-COM_EXT_count得到
	COM_Ti = MsgBuffer_Receive[0];
	COM_IH4_RATE = MsgBuffer_Receive[1];
	COM_Uz = MsgBuffer_Receive[2];
	COM_IH1_RATE = MsgBuffer_Receive[4];
	COM_IH2_RATE = MsgBuffer_Receive[5];
	COM_IH3_RATE = MsgBuffer_Receive[6];

	UDC_Rf1 = UDC_Rf2 = Udc_slip = COM_Ua;
	//*****************END***********************
//*******************************end******************************

	//通信主循环
	COM_main();

}

/****************************通信相关************************************/

void COM_main(void)
{
//注：与触摸屏通信过程中，所有发送和接收的数据的个数都是按16位字的个
//数计算的，发送方式是先发送高字节再发送低字节，发送完成后计一个数。

	while (1)
	{

		//************电网电压显示计算************************
		/*Display_CON3S2S( UA_Display, UB_Display, UC_Display );
		 alpha = DisplayLS1 * DisplayLS1;
		 belta = DisplayLS2 * DisplayLS2;
		 LineVolSquare = ( alpha+belta ) * 0.3333;
		 for ( BiCount = 10; BiCount > 0; BiCount-- )
		 {
		 BiVari = ( BiMinInitial + BiMaxInitial ) * 0.5;
		 if ( ( BiVari * BiVari - LineVolSquare ) < 0 )
		 {
		 BiMinInitial = BiVari;
		 }
		 if ( ( BiVari * BiVari - LineVolSquare ) > 0 )
		 {
		 BiMaxInitial = BiVari;
		 }
		 }
		 COM_UAB = (int) ( BiVari );
		 COM_UBC = (int) ( BiVari );
		 COM_UCA = COM_UAB;
		 BiMinInitial = 0;
		 BiMaxInitial = 1000;*/
		//***************显示部分结束**************************
		Read_date(0xA8, 1);
		Reactive_Power = Com_date[1];
		Reactive_Power &= 0x03;

		if ((Run_Stop_Flag == 0) || (ERROR_flag != 0))  //补偿指令清零
		{
			Clear_Comd(); //补偿及其稳压指令清零
		}
		//读取启、停控制位
		//Display();
		// ERROR_flag=0;
		Read_date(0x64, 1);
		Run_Stop_Flag = Com_date[1];
		Run_Stop_Flag &= 0x01;

		if (Init_para_flag == 0)
		{
			//参数初始化
			Display_Para();	   //程序初始在触摸屏上显示从EEPROM读出来的设置参数，且仅仅显示一次

			GpioDataRegs.GPBSET.bit.GPIO57 = 1;	   //保护信号复位
			delay_t(12);
			GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;	   //禁止保护信号复位
			Wrong_Flag = 0;
			Init_para_flag = 1;
		}
//**********************************加入自重启等新应用程序************************************//

		Read_date(0xD0, 1);	   //208
		Reboot_Mode_Set = Com_date[1];
		Reboot_Mode_Set &= 0x01;	   //208.0
		if (Reboot_Mode_Set != L_Reboot_Mode_Set)
		{
			L_Reboot_Mode_Set = Reboot_Mode_Set;
			MsgBuffer_Send[0] = Reboot_Mode_Set;
			WR_DATA_PROCESS(1);
			WR_EEPROM(2, 0x22, 0x4);
		}

		if (PowerFailRecord != L_PowerFailRecord)
		{
			L_PowerFailRecord = PowerFailRecord;
			MsgBuffer_Send[0] = PowerFailRecord;
			WR_DATA_PROCESS(1);
			WR_EEPROM(2, 0x20, 0x4);
		}

		if (Reboot_Mode_Set == 0)
		{
			//Reboot_Run=0;
			//Reboot_Run_Lamp=0;
			//重启计时数据复位
			//Delay_Rebt_Add=0;
			//Delay_Rebt_Add1=0;
			Delay_Rebt_Add2 = 0;
			Delay_Rebt_Add3 = 0;
		}

		if (Run_Stop_Flag == 0)		  //控制重启倒计时复位
		{
			Reboot_Count_Down = 6;
		}

		//&&&&&&&&&&&&&&&&&&&&&&&&&判断电网电压故障并据此控制APF动作&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		FaultCtrlVari = 1;		  //FaultCtrlVari用于作为控制变量，目的是方便在主循环中判断电网故障
		if (PowerFailRecord == 0)		  //没有故障信号时将所有重启故障计数清零
		{
			Delay_Rebt_Add2 = 0;
			Delay_Rebt_Add3 = 0;
		}

		if (Power_Normal) //( Fault_flag >= 1309 && Fault_flag < 1771 )
		{
			L_Fault_flag = Fault_flag;
		}
		else if (Power_Fail) //( Fault_flag < 616 ) //有故障信号就停止运行，待清故障信号时程序再重新运行。
		{
			//这样 做在出现故障信号时，程序就停止运行，运行指示灯灭，稳压和谐波指令电流的计算也停止，调试可能不便利。
			//为调试便利，可以在调试阶段屏蔽到此else()段。
			if (Run_Stop_Flag == 1)		//系统开启指令
			{
				Display_Run_Stop(0);	//控制停止
				Run_Stop_Flag = 0;
				PowerFailRecord = 1;//记录出现的电网故障，掉电时同时需要复位一次以确保不是大幅度的波动造成的硬件保护
			}
			Delay_Rebt_Add2 = 0;
			Delay_Rebt_Add3 = 0;
			L_Fault_flag = Fault_flag;
		}
		else if (Power_Low) //( Fault_flag >= 616 && Fault_flag < 1155 )
		{
			if (Run_Stop_Flag == 1)
			{
				Display_Run_Stop(0);	//控制停止
				Run_Stop_Flag = 0;
				PowerFailRecord = 1;	//记录出现的电网故障，由于没有硬件保护不需要复位
			}
			//以下两个变量为0用于控制APF处于待机状态
			Delay_Rebt_Add2 = 0;
			Delay_Rebt_Add3 = 0;
			L_Fault_flag = Fault_flag;
		}
		//正常运行过程中进入滞环区域不当作故障记录。
		else if (HystisRing_Low) //( Fault_flag >= 1155 && Fault_flag < 1309 ) //如果电压处于滞环区域通过判断上次的电压状态判断是否为电压故障
		{
			if (L_Power_Low || L_Power_Fail) //( Fault_flag < 616 )前一次电网电压状态为低电压或电网大幅波动
			{
				if (Run_Stop_Flag == 1)
				{
					Display_Run_Stop(0); //控制停止
					Run_Stop_Flag = 0;
					PowerFailRecord = 1; //记录出现的电网故障，由于没有硬件保护恍枰次?
				}
				//以下两个变量为0用于控制APF处于待机状态
				Delay_Rebt_Add2 = 0;
				Delay_Rebt_Add3 = 0;
			}
			L_Fault_flag = Fault_flag;
		}
		else if (HystisRing_Over) //( Fault_flag >= 1771 && Fault_flag < 1920 ) //正常运行过程中进入滞环区域不当作故障记录。
		{
			if (L_Power_Over) //( L_Fault_flag >= 1920 )
			{
				if (Run_Stop_Flag == 1)
				{
					Display_Run_Stop(0);	//控制停止
					Run_Stop_Flag = 0;
					PowerFailRecord = 1;	//记录出现的电网故障，由于没有硬件保护不需要复位
				}
				//以下两个变量为0用于控制APF处于待机状态
				Delay_Rebt_Add2 = 0;
				Delay_Rebt_Add3 = 0;
			}
			L_Fault_flag = Fault_flag;
		}
		else if (Power_Over) //( Fault_flag >= 1920 )
		{
			if (Run_Stop_Flag == 1)
			{
				Display_Run_Stop(0);	//控制停止
				Run_Stop_Flag = 0;
				PowerFailRecord = 1;	//记录出现的电网故障，由于没有硬件保护不需要复位
			}
			//以下两个变量为0用于控制APF处于待机刺?
			Delay_Rebt_Add2 = 0;
			Delay_Rebt_Add3 = 0;
			L_Fault_flag = Fault_flag;
		}
		FaultCtrlVari = 0;	//FaultCtrlVari用于作为控制变量，目的是方便在主循环中判断电网故障
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&结束&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

		//*****************************end**************************************
		if (Init_para_flag == 1)      //触摸屏通信断是否动重启
		{
			//用于判断完全掉电后上电是否自动簦门卸进行一次
			//EEPROM中读到的重启信号优先级要低于触摸屏上当前设置的重启信息,上次重启设置可以
			//在重启前人为的从触摸屏上清除掉。
			if (Initial_Reset == 0)
			{
				GpioDataRegs.GPBSET.bit.GPIO57 = 1;		//保护信号复位
				delay_t(12);		//   20 us
				GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;		//禁止保护信号复位
				Wrong_Flag = 0;
				Initial_Reset = 1;
			}
		}
		//*********************************perfect********************************
		//重启条件
		if (PowerFailRecord == 1 && Power_Normal && Reboot_Mode_Set == 1)/* ( Fault_flag >= 1309 && Fault_flag < 1771 ) && (Fault_Count<=FluctuateCount)*/
		{
			Delay_Rebt_Add2 += 1;
			//if(Delay_Rebt_Add2>100)
			if (Delay_Rebt_Add2 > 100)
			{
				Delay_Rebt_Add3 += 1;
				Delay_Rebt_Add2 = 0;
			}
			Reboot_Count_Down = 6 - Delay_Rebt_Add3;
			//复位2分钟后重启,通过修改Delay_Rebt_Add1>1的值可以控制完全掉电后再次重启的时间
			//暂时设定为2分钟左右
			if (Delay_Rebt_Add3 > 5)
			{
				GpioDataRegs.GPBSET.bit.GPIO57 = 1;		      //保护信号复位
				delay_t(240);		      //   20 us
				GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;		      //禁止保护信号复位
				Wrong_Flag = 0;
				//Reboot_Run_Lamp=1;// 启动自动重启
				//Reboot_Run=1;
				Display_Run_Stop(1);
				Run_Stop_Flag = 1;      //控制启动
				Delay_Rebt_Add3 = 0;
				//Fault_Count++;
				// if(Fault_Count>FluctuateCount)
				//{
				//Fault_Count=10000;
				//}
				//如果已经重启，则说明电网故障清除，将电网故障记录清零并写入E2PROM
				L_PowerFailRecord = PowerFailRecord = 0;
				MsgBuffer_Send[0] = PowerFailRecord;
				WR_DATA_PROCESS(1);
				WR_EEPROM(2, 0x20, 0x4);
				Reboot_Count_Down = 6;
			}
		}
		//**********************计算基波功率因数*******************************

		I_fundmental = Check_Theta(I_lade_d, I_lade_q);
		if (I_fundmental >= 360)
			I_fundmental -= 360;
		Sin_Value(I_fundmental);
		if (cosVal < 1 && cosVal > 0)
			Factor = cosVal * 100;
		if (cosVal < 0 && cosVal > -1)
			Factor = -cosVal * 100;
//**********************************end*****************************************************// 

//******************************************************************************************//
		Para_set_end(0xAC, 0x00); //0xAC为触摸屏"等待参数设置"的地址；"等待参数设置"为指示灯//参数设置置位
		Para_set_end(0xAE, 0x00); //0XAE为触摸屏"出厂设置x完成"指示灯地址
		if (Run_Stop_Flag == 0x01)
		{
			if (display_flag == 5)
			{
				Display(); //用于显示需要动态显示的参数，动态显示其实是一个不断刷新的过程
				display_flag = 0;
			}
			else
				display_flag += 1;
		}
		else
		{
			Read_date(0xAB, 1); //按了某一个"确定"键，相应的位为1  ？？？？
			Para_SET = Com_date[1];
			Para_SET &= 0x07;

			//Display_Para();
			if (Para_SET == 0x02) //这里的Para_SET对应触摸屏"用户参数设置"中的"确定"。
			{
				//Para_set3();
				Para_set1(); //读触摸屏参数赋给相应的变量，并将与原来不同的参数写隕EPROM
				//修改触摸屏参数且按了"用户参数设置"中的"?之后，将设置的参数显示一下
				//由于触摸屏设置参数后只要不对设置的参数刷新，不下电，则设置的参数不会改变
				//因此此处是否对设置的参数进行一次显示都无所谓，此处为便于理解在按确定键之后
				//在触摸屏上显示一次，另外此处还可以用于校验Para_set1()从触摸屏读出的参欠裾?
				Display_Para();
			}
			//Para3_Crl = 0;
			else if (Para_SET == 0x04)		//这里的Para_SET对应触摸屏"调试参数设?中的"确定"？？？
			{
				Para_set2();				//读触摸屏参数赋给相应的变量，并将与原来不同的参数写入EEPROM
				//修改触摸屏参数且按了"调试参数设置"中的""之后，将设置的参数显示一下
				//由于触摸屏设置参数后只要不对设置的参数刷新，不下电，则设置的参数不会改变
				//因此此处是否陨柚玫牟问幸淮蜗允径嘉匏剑舜ξ阌诶斫庠诎慈范?
				//在触摸屏上显示一次，另外此处还可以用于校验Para_set2()从触摸屏读出的参数是否正确
				Display_Para();
			}
			//Display();

			if (display_flag == 5)
			{
				Display();				//用于显示需要动态显示的参数，动态显示其一个不断刷新的过程
				display_flag = 0;
			}
			else
				display_flag += 1;

			//*********************出厂设置**************************
			Read_date(0xAD, 1);
			Factory_Flag = Com_date[1];
			Factory_Flag &= 0xFF;
			//暂时无论按那个出厂设置键，都恢复一个出厂设置，以后要区分用于恢复不同的出厂参数
			if (Factory_Flag == 0x02)
			{
				Restore_Factory_Set();
				Para_set_end(0xAE, 0x02);	//0xAE:触摸屏上等待出厂1设置地址；0x02：向该地址写入的值
				Display_Para();
			}
			else if (Factory_Flag == 0x04)
			{
				Restore_Factory_Set();
				Para_set_end(0xAE, 0x04);
				Display_Para();
			}
			else if (Factory_Flag == 0x08)
			{
				Restore_Factory_Set();
				Para_set_end(0xAE, 0x08);
				Display_Para();
			}
			else if (Factory_Flag == 0x10)
			{
				Restore_Factory_Set();
				Para_set_end(0xAE, 0x10);
				Display_Para();
			}

			//******************出厂设置结束*************************
		}

	}
}

void Read_date(int ad, int n)
{

	int n_date;
	loop: com1 = 0;
	GpioDataRegs.GPBSET.bit.GPIO61 = 1;		//置发送模式
	ScicRegs.SCITXBUF = 0x1B;		//ESC一ASCII码的内容
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x52;		//R-ASCII码的内容
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x00;		//NULL-ACSII码内容
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = ad;		//此位为地址位
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x00;		//NULL-ACSII码内容
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = n;		//此位为传输数据的个数
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0xD3;		//此为是结束效验位
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	GpioDataRegs.GPASET.bit.GPIO26 = 1;
	GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;		//置为接受模式
	ScicRegs.SCICTL1.bit.SWRESET = 0;			//清标志位
	ScicRegs.SCICTL1.bit.SWRESET = 1;
	while (ScicRegs.SCIRXST.bit.RXRDY == 0)
	{
		com1++;
		delay_t1(100);
		if (com1 > 2000)
		{
			goto loop;
		}
	}
	com1 = 0;
	GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
	ReceivedChar = ScicRegs.SCIRXBUF.all;
	ReceivedChar &= 0x00FF;				//接受？1B-ASCII码中为'ESC'
	if (ReceivedChar != 0x001B)
		goto loop;

	while (ScicRegs.SCIRXST.bit.RXRDY == 0)
	{
		com1++;
		delay_t1(100);
		if (com1 > 1000)
		{
			goto loop;
		}
	}
	ReceivedChar = ScicRegs.SCIRXBUF.all;
	ReceivedChar &= 0x00FF;				//接受？41-ASCII码中为‘A’
	if (ReceivedChar != 0x0041)
		goto loop;
	for (n_date = 0; n_date < (2 * n); n_date++)
	{
		com1 = 0;
		while (ScicRegs.SCIRXST.bit.RXRDY == 0)
		{
			com1++;
			delay_t1(100);
			if (com1 > 1000)
			{
				goto loop;
			}
		}
		ReceivedChar = ScicRegs.SCIRXBUF.all;
		ReceivedChar &= 0x00FF;
		Com_date[n_date] = ReceivedChar;
	}
}
void Display(void)
{
	//用数组的方式发送数据经常会发送错误，因此主要的数据、尤其是指令数据直接发送立即数；
	GpioDataRegs.GPBSET.bit.GPIO61 = 1;		//置发送模式
	//数据1
	display_date_one[1] = COM_UAB % 256;    //COM_U为INT型
	display_date_one[0] = COM_UAB >> 8;     //电网电压平衡，用一相表示
	display_date_one[3] = COM_UBC % 256;
	display_date_one[2] = COM_UBC >> 8;
	display_date_one[5] = COM_UCA % 256;
	display_date_one[4] = COM_UCA >> 8;
	display_date_one[6] = COM_IA >> 8;     //0x00;
	display_date_one[7] = COM_IA % 256;     //;COM_IA;
	display_date_one[8] = COM_IB >> 8;     //0x00;
	display_date_one[9] = COM_IB % 256;     //COM_IB;
	display_date_one[10] = COM_IC >> 8;     //0x00;
	display_date_one[11] = COM_IC % 256;     //COM_IC;
	display_date_one[12] = COM_I0 >> 8;     //0x00;
	display_date_one[13] = COM_I0 % 256;     //COM_I0;

	ScicRegs.SCITXBUF = 0x1B;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}		//ESC-ASCII码内容
	ScicRegs.SCITXBUF = 0x57;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}		//触摸屏接收数据命令字 'W'-ASCII码内容
	ScicRegs.SCITXBUF = 0x00;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x32;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}		//接收数据存放地址（触摸屏）
	ScicRegs.SCITXBUF = 0x00;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x07;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}		//接收数据个数（16位字个数=1/2字节个数）
	for (com = 0; com < 14; com++)
	{
		ScicRegs.SCITXBUF = display_date_one[com];
		while (ScicRegs.SCICTL2.bit.TXRDY == 0)
		{
			;
		}
	}
	ScicRegs.SCITXBUF = 0x8D;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}

//数据2
	display_date_two[0] = THD_IA >> 8;		//0x00;
	display_date_two[1] = THD_IA % 256;		//THD_IA;     //电网电压平衡，用一相表示
	display_date_two[2] = THD_IB >> 8;		//0x00;
	display_date_two[3] = THD_IB % 256;		//THD_IB;
	display_date_two[4] = THD_IB >> 8;		//0x00;
	display_date_two[5] = THD_IC % 256;		//THD_IC;
	display_date_two[7] = COM_Udc % 256;
	display_date_two[6] = COM_Udc >> 8;
	display_date_two[8] = 0x00;
	display_date_two[9] = 0x32;
	display_date_two[10] = ERROR_flag >> 8;		//0x00;
	display_date_two[11] = ERROR_flag % 256;		//ERROR_flag;
	//*****************reboot***********************
	display_date_two[12] = Reboot_Count_Down >> 8; //63
	display_date_two[13] = Reboot_Count_Down % 256;
	display_date_two[14] = ((int) Power_Factor) >> 8; //0;//Power_Factor>>8;      //64
	display_date_two[15] = ((int) Power_Factor) % 256; //99;//Power_Factor%256;
	display_date_two[16] = THD_IA3 >> 8;           //65
	display_date_two[17] = THD_IA3 % 256;
	display_date_two[18] = THD_IA5 >> 8;           //66
	display_date_two[19] = THD_IA5 % 256;
	display_date_two[20] = THD_IA7 >> 8;           //67
	display_date_two[21] = THD_IA7 % 256;
	display_date_two[22] = THD_IA9 >> 8;           //68
	display_date_two[23] = THD_IA9 % 256;
	display_date_two[24] = THD_IA11 >> 8;          //69
	display_date_two[25] = THD_IA11 % 256;
	display_date_two[26] = CompensateA >> 8;          //70
	display_date_two[27] = CompensateA % 256;
	display_date_two[28] = CompensateB >> 8;          //71
	display_date_two[29] = CompensateB % 256;
	display_date_two[30] = CompensateC >> 8;          //72
	display_date_two[31] = CompensateC % 256;
	//**************************end************************

	ScicRegs.SCITXBUF = 0x1B;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x57;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}          //以上为写触摸屏命令
	ScicRegs.SCITXBUF = 0x00;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x39;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}          //以上为触摸屏的首地址
	ScicRegs.SCITXBUF = 0x00;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x10;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}          //以上为要向触摸屏写的16位字的个数
	for (com = 0; com < 32; com++)
	{
		ScicRegs.SCITXBUF = display_date_two[com]; //此处用数组的方式，编程简单，但是如何使用，将数组中的数和另外一个数对应还是问题
		while (ScicRegs.SCICTL2.bit.TXRDY == 0)
		{
			;
		}
	}	                                   //以上为向触摸屏写所有8位字节，即实际要写的内容。高字节在前低字节在后
	ScicRegs.SCITXBUF = 0x8D;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}	                                   //以上为写触摸屏结束命令？？？
}
void Display_Para(void)
{
	GpioDataRegs.GPBSET.bit.GPIO61 = 1;	              //置发送模式,非常重要，没有这一句不能进行正常发送
	//数据3
//************************eeprom******************************
	Para1_Intdate[1] = ((int) UDCREF % 256);	//直流电压稳压值低字节
	Para1_Intdate[0] = ((int) UDCREF >> 8);	//直流电压稳压值高字节,首先发送     //150
	Para1_Intdate[3] = (int) Ih_Max;
	Para1_Intdate[2] = 0x00;	//过流保护值                              //151
	Para1_Intdate[5] = C_way;
	Para1_Intdate[4] = 00;	//补偿方式                                 //152
	Para1_Intdate[7] = (COM_ALL_rate % 256);	// 2009.5.31
	Para1_Intdate[6] = (COM_ALL_rate >> 8);                            //153
	Para1_Intdate[9] = (COM_EXT_count % 256);
	Para1_Intdate[8] = (COM_EXT_count >> 8);                            // 154
	Para1_Intdate[11] = (COM_IHB_RATE % 256);
	Para1_Intdate[10] = (COM_IHB_RATE >> 8);                            //155
	Para1_Intdate[13] = ((int) UDC_Max % 256);
	Para1_Intdate[12] = ((int) UDC_Max >> 8);                            //156
	Para1_Intdate[14] = (COM_IH1 >> 8);
	Para1_Intdate[15] = (COM_IH1 % 256);	//157
	Para1_Intdate[16] = (COM_IH1_RATE >> 8);
	Para1_Intdate[17] = (COM_IH1_RATE % 256);	//158
	Para1_Intdate[18] = (COM_IH2 >> 8);
	Para1_Intdate[19] = (COM_IH2 % 256);	//159
	Para1_Intdate[20] = (COM_IH2_RATE >> 8);
	Para1_Intdate[21] = (COM_IH2_RATE % 256);	//160
	Para1_Intdate[22] = (COM_IH3 >> 8);
	Para1_Intdate[23] = (COM_IH3 % 256);	//161
	Para1_Intdate[24] = (COM_IH3_RATE >> 8);
	Para1_Intdate[25] = (COM_IH3_RATE % 256);	//162
	Para1_Intdate[26] = (COM_IH4 >> 8);
	Para1_Intdate[27] = (COM_IH4 % 256);	//163
	Para1_Intdate[28] = (COM_IH4_RATE >> 8);
	Para1_Intdate[29] = (COM_IH4_RATE % 256);	//164
	Display_Send_Ready(0x96, 15);	//准备发送显示数据；触摸屏起始地址150（0x96),连续显示14/2=7个数据
	for (com = 0; com < 30; com++)
	{
		ScicRegs.SCITXBUF = Para1_Intdate[com];	//此处用数组姆绞剑喑碳虻ィ侨绾问褂茫橹械氖土硗庖桓鍪杂故俏侍?
		while (ScicRegs.SCICTL2.bit.TXRDY == 0)
		{
			;
		}
	}
	ScicRegs.SCITXBUF = 0x8D;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	//Para3_Crl = 0;
//数据4
	Para1_Intdate[1] = ((int) UDC_Kp % 256);
	Para1_Intdate[0] = ((int) UDC_Kp >> 8);	//200
	Para1_Intdate[3] = ((int) UDC_Ki % 256);
	Para1_Intdate[2] = ((int) UDC_Ki >> 8);	//201
	Para1_Intdate[5] = ((int) U_FtoI % 256);
	Para1_Intdate[4] = ((int) U_FtoI >> 8);	//202
	Para1_Intdate[7] = (COM_Ua % 256);
	Para1_Intdate[6] = (COM_Ua >> 8);	//203
	Para1_Intdate[9] = 0;	//(COM_Tu%256);
	Para1_Intdate[8] = 0;	//(COM_Tu>>8);//204
	Para1_Intdate[11] = 0;	//(COM_Ti%256);
	Para1_Intdate[10] = 0;	//(COM_Ti>>8);//205
	Para1_Intdate[13] = 0;	//(COM_Iz%256);
	Para1_Intdate[12] = 0;	//(COM_Iz>>8);//206
	Para1_Intdate[15] = 0;	//(COM_Uz%256);
	Para1_Intdate[14] = 0;	//(COM_Uz>>8);//207
	//*************************
	Para1_Intdate[17] = (Reboot_Mode_Set % 256);
	Para1_Intdate[16] = (Reboot_Mode_Set >> 8);		//208

	Display_Send_Ready(0xC8, 8);	//准备发送显示数据；触摸屏起始地址150（0x96),连续显示16/2=8个数据
	for (com = 0; com < 18; com++)
	{
		ScicRegs.SCITXBUF = Para1_Intdate[com];	//此处用数组的方式，编程简单，但是如何使用，将数组中的数和另外一个数对应还是问题
		while (ScicRegs.SCICTL2.bit.TXRDY == 0)
		{
			;
		}
	}
	ScicRegs.SCITXBUF = 0x8D;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
//数据5
	/*		Para1_Intdate[1] = ((int)C_Way1%256);
	 Para1_Intdate[0] = ((int)C_Way1>>8);//169
	 Para1_Intdate[3] = ((int)C_Way2%256);
	 Para1_Intdate[2] = ((int)C_Way2>>8);//170
	 Para1_Intdate[5] = ((int)C_Way3%256);
	 Para1_Intdate[4] = ((int)C_Way3>>8);//171

	 Display_Send_Ready(0xA9,3);//准备发送显示数据；触摸屏起始地址169（0xA9),连续显示3个数据
	 for(com=0;com<6;com++)
	 {
	 ScicRegs.SCITXBUF=Para1_Intdate[com];//此处用数组的方式，编程简单，但是如何使用，将数组中的数和另外一个数对应还是问题
	 while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	 }
	 ScicRegs.SCITXBUF=0x8D;
	 while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	 */
}
//******************************自重启用参数***********************************
void Display_Run_Stop(int Run_Stop)
{
	int Run_Stop_Ctr[2] =
	{ 0, 0 };

	GpioDataRegs.GPBSET.bit.GPIO61 = 1;		//置发送模式,非常重要，没有这一句不能进行正常发送
	Run_Stop_Ctr[1] = Run_Stop;
	Run_Stop_Ctr[0] = 0x00;
	Display_Send_Ready(0x64, 1);	//准备发送显示数据；触摸屏起始地址150（0x96),连续显示16/2=8个数据
	for (com = 0; com < 2; com++)
	{
		ScicRegs.SCITXBUF = Run_Stop_Ctr[com];// 用数组的方式，编程简单，但是如何使用，将数组中的数和另外一个数对应还是问题
		while (ScicRegs.SCICTL2.bit.TXRDY == 0)
		{
			;
		}
	}
	ScicRegs.SCITXBUF = 0x8D;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
}
//****************************end*****************************************

void Para_set1(void)
{
	Read_date(0x96, 15);
	EEPROM_Compare = (Com_date[0] << 8) | Com_date[1]; //150
	if (EEPROM_Compare != (int) UDCREF) //判断是否将数据写入EEPROM
	{
		UDCREF = EEPROM_Compare;
		MsgBuffer_Send[0] = (int) UDCREF;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x10, 0x4);
	}
	//UDCREF=(Com_date[0]<<8)|Com_date[1]; //150
	//Ih_Max=(Com_date[2]<<8)|Com_date[3];	//151
	EEPROM_Compare = (Com_date[2] << 8) | Com_date[3];	//151
	if (EEPROM_Compare != (int) Ih_Max)
	{
		Ih_Max = EEPROM_Compare;
		MsgBuffer_Send[0] = (int) Ih_Max;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x14, 0x4);
	}
	//C_way=Com_date[5];	//152  .1=1全补。1=0分次补  。3=1 补偿基波无功
	EEPROM_Compare = Com_date[5];	//152  .1=1全补。1=0分次补  。3=1 补偿基波无功
	if (EEPROM_Compare != C_way)
	{
		C_way = EEPROM_Compare;
		MsgBuffer_Send[0] = C_way;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x16, 0x4);
	}
//	COM_ALL_rate=(Com_date[6]<<8)|Com_date[7];	//153
	EEPROM_Compare = (Com_date[6] << 8) | Com_date[7];	//153
	if (EEPROM_Compare != COM_ALL_rate)
	{
		COM_ALL_rate = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_ALL_rate;	//  EEPROM->0x40
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x40, 0x4);
	}
//	COM_EXT_count=Com_date[9]; //154
	EEPROM_Compare = Com_date[9]; //154
	if (EEPROM_Compare != COM_EXT_count)
	{
		COM_EXT_count = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_EXT_count; // EEPROM->0X42
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x42, 0x4);
	}
	n_cycle = 250 - COM_EXT_count;
// 	COM_IHB_RATE=(Com_date[10]<<8)|Com_date[11]; //155
	EEPROM_Compare = (Com_date[10] << 8) | Com_date[11]; //155
	if (EEPROM_Compare != COM_IHB_RATE)
	{
		COM_IHB_RATE = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_IHB_RATE; // EEPROM->0X44
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x44, 0x4);
	}
//	UDC_Max=(Com_date[12]<<8)|Com_date[13];	//156
	EEPROM_Compare = (Com_date[12] << 8) | Com_date[13];	//156
	if (EEPROM_Compare != (int) UDC_Max)
	{
		UDC_Max = EEPROM_Compare;
		MsgBuffer_Send[0] = (int) UDC_Max;	// EEPROM->0X12
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x12, 0x4);
	}
//	COM_IH1=(Com_date[14]<<8)|Com_date[15];		//157
	EEPROM_Compare = (Com_date[14] << 8) | Com_date[15];		//157
	if (EEPROM_Compare != COM_IH1)
	{
		COM_IH1 = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_IH1;		// EEPROM->0X46
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x46, 0x4);
	}
//  COM_IH1_RATE=(Com_date[16]<<8)|Com_date[17];	//158
	EEPROM_Compare = (Com_date[16] << 8) | Com_date[17];	//158
	if (EEPROM_Compare != COM_IH1_RATE)
	{
		COM_IH1_RATE = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_IH1_RATE;	// EEPROM->0X58
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x58, 0x4);
	}
//  COM_IH2=(Com_date[18]<<8)|Com_date[19];		//159
	EEPROM_Compare = (Com_date[18] << 8) | Com_date[19];	//159
	if (EEPROM_Compare != COM_IH2)
	{
		COM_IH2 = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_IH2;	// EEPROM->0X48
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x48, 0x4);
	}
//  COM_IH2_RATE=(Com_date[20]<<8)|Com_date[21];	//160
	EEPROM_Compare = (Com_date[20] << 8) | Com_date[21];	//160
	if (EEPROM_Compare != COM_IH2_RATE)
	{
		COM_IH2_RATE = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_IH2_RATE;	// EEPROM->0X5A
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x5A, 0x4);
	}
//  COM_IH3=(Com_date[22]<<8)|Com_date[23];		//161
	EEPROM_Compare = (Com_date[22] << 8) | Com_date[23];	//161
	if (EEPROM_Compare != COM_IH3)
	{
		COM_IH3 = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_IH3;	// EEPROM->0X4A
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x4A, 0x4);
	}
//  COM_IH3_RATE=(Com_date[24]<<8)|Com_date[25];	//162
	EEPROM_Compare = (Com_date[24] << 8) | Com_date[25];	//162
	if (EEPROM_Compare != COM_IH3_RATE)
	{
		COM_IH3_RATE = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_IH3_RATE;	// EEPROM->0X5C
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x5C, 0x4);
	}
	//////
	EEPROM_Compare = (Com_date[26] << 8) | Com_date[27];	//163
	if (EEPROM_Compare != COM_IH4)
	{
		COM_IH4 = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_IH4;	// EEPROM->0X4E
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x4E, 0x4);
	}

	EEPROM_Compare = (Com_date[28] << 8) | Com_date[29];	//164
	if (EEPROM_Compare != COM_IH4_RATE)
	{
		COM_IH4_RATE = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_IH4_RATE;	// EEPROM->0X5C
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x52, 0x4);
	}
	//////
	Para_set_end(0xAC, 0x02);	//用于发送触摸屏参数读完毕命令？？？
}
void Para_set2(void)
{
	Read_date(0xC8, 8);
	//UDC_Kp=(Com_date[0]<<8)|Com_date[1];	//200
	EEPROM_Compare = (Com_date[0] << 8) | Com_date[1]; //200
	if (EEPROM_Compare != (int) UDC_Kp) //判断是否将数据写入EEPROM
	{
		UDC_Kp = EEPROM_Compare;
		MsgBuffer_Send[0] = (int) UDC_Kp;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x18, 0x4);
	}
	//UDC_Ki=(Com_date[2]<<8)|Com_date[3];	//201
	EEPROM_Compare = (Com_date[2] << 8) | Com_date[3];	//201
	if (EEPROM_Compare != (int) UDC_Ki)	//判断是否将数据写入EEPROM
	{
		UDC_Ki = EEPROM_Compare;
		MsgBuffer_Send[0] = (int) UDC_Ki;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x1A, 0x4);
	}
	//U_FtoI=(Com_date[4]<<8)|Com_date[5];	//202
	EEPROM_Compare = (Com_date[4] << 8) | Com_date[5];	//202
	if (EEPROM_Compare != (int) U_FtoI)	//判断是否将数据写入EEPROM
	{
		U_FtoI = EEPROM_Compare;
		MsgBuffer_Send[0] = (int) U_FtoI;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x1C, 0x4);
	}
	//COM_Ua=(Com_date[6]<<8)|Com_date[7];	//203
	EEPROM_Compare = (Com_date[6] << 8) | Com_date[7];	//203
	if (EEPROM_Compare != COM_Ua)
	{
		COM_Ua = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_Ua;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x4C, 0x4);
	}
	UDC_Rf1 = UDC_Rf2 = Udc_slip = COM_Ua;
//	COM_Tu=(Com_date[8]<<8)|Com_date[9];	//204
	/*	EEPROM_Compare=(Com_date[8]<<8)|Com_date[9];	//204
	 if(EEPROM_Compare != COM_Tu)
	 {
	 COM_Tu=EEPROM_Compare;
	 MsgBuffer_Send[0] = COM_Tu;
	 WR_DATA_PROCESS(1);
	 WR_EEPROM(2,0x4E,0x0);
	 }*/
//	COM_Ti=(Com_date[10]<<8)|Com_date[11]; 	//205
	EEPROM_Compare = (Com_date[10] << 8) | Com_date[11]; 	//205
	if (EEPROM_Compare != COM_Ti)
	{
		COM_Ti = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_Ti;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x50, 0x4);
	}
//	COM_Iz=(Com_date[12]<<8)|Com_date[13];	//206
	/*	EEPROM_Compare=(Com_date[12]<<8)|Com_date[13];	//206
	 if(EEPROM_Compare != COM_Iz)
	 {
	 COM_Iz=EEPROM_Compare;
	 MsgBuffer_Send[0] = COM_Iz;
	 WR_DATA_PROCESS(1);
	 WR_EEPROM(2,0x52,0x0);
	 }*/
//	COM_Uz=(Com_date[14]<<8)|Com_date[15]; 	//207
	EEPROM_Compare = (Com_date[14] << 8) | Com_date[15]; 	//207
	if (EEPROM_Compare != COM_Uz)
	{
		COM_Uz = EEPROM_Compare;
		MsgBuffer_Send[0] = COM_Uz;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x54, 0x4);
	}
	Para_set_end(0xAC, 0x04);
}

void Para_set3(void)
{
	Read_date(0xA9, 3);
	//补偿率1	//169
	EEPROM_Compare = (Com_date[0] << 8) | Com_date[1]; //169
	if (EEPROM_Compare != (int) C_Way1) //判断是否将数据写入EEPROM
	{
		C_Way1 = EEPROM_Compare;
		MsgBuffer_Send[0] = (int) C_Way1;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x1E, 0x4);
	}
	//补偿率2	//170
	EEPROM_Compare = (Com_date[2] << 8) | Com_date[3];	//170
	if (EEPROM_Compare != (int) C_Way2)	//判断是否将数据写入EEPROM
	{
		C_Way2 = EEPROM_Compare;
		MsgBuffer_Send[0] = (int) C_Way2;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x2E, 0x4);
	}
	//补偿率3	//171
	EEPROM_Compare = (Com_date[4] << 8) | Com_date[5];	//171
	if (EEPROM_Compare != (int) C_Way3)	//判断是否将数据写入EEPROM
	{
		C_Way3 = EEPROM_Compare;
		MsgBuffer_Send[0] = (int) C_Way3;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2, 0x3E, 0x4);
	}
	//Para_set_end(0xAC, 0x04);
}

//此函数的目的是每次锡设置芍螽后）使对应的"等待参数设置"显示灯变亮，
//提示用户参数设置完成。此函数实质就是写触摸屏操作。
void Para_set_end(int ad, int n)	//变量n表示要写入的内容
{
	GpioDataRegs.GPBSET.bit.GPIO61 = 1;		//置发送模式
	ScicRegs.SCITXBUF = 0x1B;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x57;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}		//写触摸屏命令
	ScicRegs.SCITXBUF = 0x00;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = ad;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}		//触摸屏首地址
	ScicRegs.SCITXBUF = 0x00;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x01;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}		//写一个16位的字
	ScicRegs.SCITXBUF = 0x00;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = n;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}		//要写入的内容，高字节在前，低字节在后
	ScicRegs.SCITXBUF = 0x8D;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	} //结束写操作命令
}

void Display_Send_Ready(int ad, int n)
{
	int ad_HighByte = 0;
	int ad_LowByte = 0;
	int n_HighByte = 0;
	int n_LowByte = 0;
	if (ad > 255)
	{
		ad_LowByte = (ad % 256);
		ad_HighByte = (ad >> 8);
	}
	else
	{
		ad_LowByte = ad;
		ad_HighByte = 0x00;
	}
	if (n > 255)
	{
		n_LowByte = (n % 256);
		n_HighByte = (n >> 8);
	}
	else
	{
		n_LowByte = n;
		n_HighByte = 0x00;
	}

	ScicRegs.SCITXBUF = 0x1B;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = 0x57;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = ad_HighByte;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = ad_LowByte;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = n_HighByte;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}
	ScicRegs.SCITXBUF = n_LowByte;
	while (ScicRegs.SCICTL2.bit.TXRDY == 0)
	{
		;
	}

}

void calibration(void)
{
	static float uab_z = 0, ubc_z = -14.75, uca_z = -6;
	static float uab_k = 0.4097, ubc_k = 0.407, uca_k = 0.415;
	static float ial_z = 196.3, ibl_z = 196.5, icl_z = 194.5, i0l_z = 194.5;
	static float ial_k = 0.0192, ibl_k = 0.0194, icl_k = 0.0196, i0l_k = 0.0196;
	static float iaf_z = 189.5, ibf_z = 196, icf_z = 193, i0f_z = 193;
	static float iaf_k = 0.01933, ibf_k = 0.01935, icf_k = 1, i0f_k = 1;
	static float udc_k = 0.4921, udc_b = -89.2057;
#if 0
	/*定标函数
	 注意： 在电压 电流 母线电压采样完成后调用
	 零漂与增益需通过定标确定数据   */
	static float Ua_z = 0, Ub_z = -13, Uc_z = -6; //电网电压的零漂
	static float Ua_k = 0.4097, Ub_k = 0.407, Uc_k = 0.415;//电网电压的增益
	static float Ia_z = 194.5, Ib_z = 205, Ic_z = 201;//负载电流的零漂
	static float Ia_k = 0.0192 * 50, Ib_k = 0.01916 * 50, Ic_k = 0.01939 * 50;//负载电流的增益 100倍定标
	static float Iaf_z = 189.5, Ibf_z = 196, Icf_z = 193.5, I0f_z = 193;//APF电流的零漂
	static float Iaf_k = 0.01933 * 50, Ibf_k = 0.01935 * 50, Icf_k = 0.01941
	* 50, I0f_k = 0.01940 * 50;//APF电流的增益 100倍定标
	static float Udc_z = -96, Udc_k = 0.505;//母线电压的增益与零漂
#endif
	/*
	 * 负载电流定标
	 */
	IAADRF += ial_z, IAADRF *= ial_k;
	IBADRF += ibl_z, IBADRF *= ibl_k;
	ICADRF += icl_z, ICADRF *= icl_k;
	I0ADRF += i0l_z, I0ADRF *= i0l_k;
	/*
	 * APF电流定标
	 */
	IAAPF += iaf_z, IAAPF *= iaf_k;
	IBAPF += ibf_z, IBAPF *= ibf_k;
	ICAPF += icf_z, ICAPF *= icf_k;
	I0APF += i0f_z, I0APF *= i0f_k;
	/*
	 * 线电压定标
	 */
	UAADRF += uab_z, UAADRF *= uab_k;
	UBADRF += ubc_z, UBADRF *= ubc_k;
	UCADRF += uca_z, UCADRF *= uca_k;
	/*
	 * 直流母线电压定标
	 */
	UDCADRF *= udc_k;
	UDCADRF += udc_b;

}

/*****************************控制相关*********************************/
//主要的控制函
interrupt void epwm1_timer_isr(void)
{
	float tempp1 = 0;
//	float tempp2=0;
	float invented_power = 0;                 //PLL用到
	float ia = 0, ic = 0;
	float Omiga = 0, omiga_delta = 0;
	float Rate = 0;
	DINT;
	//禁止中断
	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;  //触发F28335B中断进行电压采样和相关计算。
	GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;  //测试运行
	//ERROR_flag=0;
//保护信号采样
//	ERROR_flag=GpioDataRegs.GPADAT.all;  //假设高电平故障
//	ERROR_flag&=0x3000;
//	ERROR_flag=ERROR_flag>>11;
//采样并允萁写?
	GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;     //GPIO60/CONVST置低，启动AD；
	delay_t(12);							//保持1us
	GpioDataRegs.GPBSET.bit.GPIO60 = 1;      	//GPIO60/CONVST置高，等待进行AD；
	delay_t(24);                      	    //延时2us，等待AD结束；
	//GpioDataRegs.GPACLEAR.bit.GPIO24=1; 	//选通AD
	for (i = 0; i < 8; i++)
	{
		AD_result[i] = *AD_register;     	    //读取AD转换结果到AD_result中；
	}
	//GpioDataRegs.GPASET.bit.GPIO24=1;
	for (i = 0; i < 8; i++)
	{
		AD_result[i] &= 0x0FFF;
		if (AD_result[i] > 0x07FF)				//若AD结果为负则进行处理。
		{
			AD_result[i] -= 0x1000;
		}
	}
	IAADRF = -AD_result[0];  //   			//负载A相电流采样值
	IBADRF = -AD_result[1];  //				//负载B相电流采样值
	ICADRF = -AD_result[2];  //
	IAAPF = -AD_result[3]; 			//APFA相电流采样值
	IBAPF = -AD_result[4];   			//APFB相电流采样值
	ICAPF = -AD_result[5];
	I0APF = -AD_result[6];
	UDCADRF = AD_result[7];   //直流侧电压采样值* 0.8
	//测试运行时间,从中断到此时所用的时间约为8us
	//将F28335B采样得到的三相电网电压读出，一定要注意两片DSP采样时序配合，暂时默认到此时F28335B
	//已经将采样数据写到DRAM中
	//if ( GpioDataRegs.GPADAT.bit.GPIO18 == 0 )

	while (GpioDataRegs.GPADAT.bit.GPIO18 == 1)
	{
		;
	}

	UAADRF = (*(DRAM_data + 3));
	UBADRF = (*(DRAM_data + 4));
	UCADRF = (*(DRAM_data + 5));

	INRL_data = (int) *(DRAM_data + 0x7FFE); //将INTL置高

	calibration();
	UDC_R = UDCADRF;

	//***************************graph*************************************
	{
		graph_data[m] = IAADRF; //-Udc_alpha;//IAADRF;//Udc_alpha;//ICAPF;//Ih1_a;/Ua_APF;//Ih_a;	//作图需要
		graph_data1[m] = IBADRF; //I_apf_alpha;//IBADRF;//UAADRF;//I_apf_alpha;//Ih_beta;//Ih_beta;//IBADRF;//IBADRF;//Udc_d;//IBADRF;
		graph_data2[m] = ICADRF; //ICAPF;//UBADRF;//PI_alpha;//ICADRF;//IAADRF;//I1_a;//PI_alpha;//ModuComdC;//Ih3_a;
		graph_data3[m] = I0ADRF;
		m++;
		if (m > 499)
		{
			m -= 500;
		}
	}

	GpioDataRegs.GPASET.bit.GPIO27 = 1;
	/********************************直流侧电压控制**************************/
//	UDCADRF += UDC_Z;    					//直流侧电压零漂补偿
//	UDC_R = UDCADRF * UDC_DB;
	UDC_Rf1 = (1 - Delta_T * Ts_UDC) * UDC_Rf1 + UDC_R * Delta_T * Ts_UDC; //直流侧电压滤波

	if ((Run_Stop_Flag == 1) && (ERROR_flag == 0))			//若APF启动,UDC的PI开始调节
	{

		if ((Udc_slip < UDCREF))
		{
			//Udc_slip = Udc_slip + ( UDCREF - UDC_Rf2 ) * 0.0001;   //直流电压给定为一斜坡函数
			/*
			 * XXX 在电压没有达到给定电压时，电压的给定以一定斜率递增
			 * 这里有一些地方我觉得可以改正：
			 * （1）应该是电压小于给定电压某个值时才为了防止过冲才如此
			 * 		应该允许电压在一定范围波动时，直接进行PI调节
			 * （2）这里的增量（UDCREF-500)*0.0001实际直接改成某个数值更好
			 * 		避免给定电压小于500的情况
			 */
			Udc_slip = Udc_slip + (UDCREF - 500) * 0.0001;
		}
		else
		{
			Udc_slip = UDCREF;
		}
		UDC_erro_new = (Udc_slip - UDC_Rf1) * VolErrorGain;

		UDC_PI(UDC_Kp, UDC_Ki);

	}
	else				//若APF停止, 若干问辶
	{
		UDC_erro_old = 0;
		UDC_erro_new = 0;
		Idref = 0;
		Inref = 0;
		T_cout = 0;
		T_cout_U = 0;
		T_Flag = 0;
		Udc_slip = UDC_Rf1;

		for (i = 0; i < 6; i++)
		{

			FilterPara1[i] = 0;
			FilterPara2[i] = 0;
			FilterPara3[i] = 0;
			FilterPara4[i] = 0;
			//FilterPara5[i] = 0;
		}
	}

//UDC显示及保护
	COM_Udc1 = UDC_Rf1;
	if (COM_Udc1 < 0)
		COM_Udc1 = 0;
	COM_Udc = (int) COM_Udc1;

	//if(Wrong_Flag == 0)
	//{
	ERROR_flag = 0;
	//}

	if ((COM_Udc1 > UDC_Max) || (GpioDataRegs.GPADAT.bit.GPIO14 == 0))// 过压,一般程序的设定值小于硬件保护值
	{
		ERROR_flag |= 0x01;
		//Wrong_Flag = 1;
	}

	if (GpioDataRegs.GPADAT.bit.GPIO15 == 0)	//IGBT错
	{
		ERROR_flag |= 0x02;
		//Wrong_Flag = 1;
	}

	if (GpioDataRegs.GPADAT.bit.GPIO12 == 0)	//过流
	{
		ERROR_flag |= 0x04;
		//Wrong_Flag = 1;
	}

	if (GpioDataRegs.GPADAT.bit.GPIO13 == 0)	//过温
	{
		ERROR_flag |= 0x08;
		//Wrong_Flag = 1;
	}

//	if(GpioDataRegs.GPADAT.bit.GPIO16 == 1)//电网掉电
//	{
//	   ERROR_flag |= 0x10;
	//Wrong_Flag = 1;
//	}

	/*if( Wrong_Temp == 1)
	 {
	 Wrong_Flag = 0;
	 }*/

//************************************相位检测********************************
	Fault_flag = Rectifier(UAADRF, UBADRF, UCADRF);

	THETA_1 = THETA_0;
	/********************************CLARKE**************************************/
	if (PLL == 0)
		THETA_C = Check_Phase(UAADRF, UBADRF) + Theta_Delta;//Theta_Delta补偿滤波引起的滞后
	/********************************PLL*****************************************/
	else
	{
		Sin_Value(Omiga_t);
		ia = sinVal;
		Sin_Value(Omiga_t + 120);
		ic = sinVal;
		invented_power = -ia * UAADRF + ic * UBADRF;
		invented_integral = invented_integral + invented_power * PLL_TS;

		if (invented_integral > 1000.0)
			invented_integral = 1000.0;
		if (invented_integral < (-1000.0))
			invented_integral = -1000.0;

		Omiga = 90 * invented_power + 220 * invented_integral;

		omiga_delta = Omiga * PLL_TS;

		Omiga_t = Omiga_t + omiga_delta;
		while (Omiga_t < 0)
			Omiga_t += 360;
		while (Omiga_t >= 360)
			Omiga_t -= 360;

//    Omiga_t2= Omiga_t + 18.2;
		THETA_C = Omiga_t + 18.2;			// + 18.2;
	}

	/*
	 说明：Omiga_t和invented_integral为全局变量，本次值和上次值相关。Omiga_t为相角（超前正序90度，和原来的程序算出的一致），invented_integral为虚拟功率的积分值。ia，ic和invented_power，Omiga，omiga_delta为局部变量。PLL_TS为中断的间隔时间，取0.00008.有意调节PI使得锁定时间在5秒左右，以增强对瞬时干扰的抵抗能力。
	 */

	/****************************END*****************************************/

	while (THETA_C < 0)
		THETA_C += 360;
	while (THETA_C >= 360)
		THETA_C -= 360;

	THETA_0 = THETA_C;
	if (((THETA_1 - THETA_0) > 300) && (CROSS_FLAG == 0)) //CROSS_FLAG为了防止电压检测不准，多次过零
	{
		CROSS_ZERO = 1;
		CROSS_FLAG = 20;
	}
	else
		CROSS_ZERO = 0;

	CROSS_FLAG -= 1;
	if (CROSS_FLAG < 1)
		CROSS_FLAG = 0;

	if (THETA_C > 30.2)
	{
		if (THETA_C < 30.3)
		{
			COM_UAB = (int) (UAADRF * 1.488);
			COM_UBC = (int) (UBADRF * 1.488);
			COM_UCA = COM_UAB;
			//COM_IB=(int)(COM_IB_f*1);
			//COM_IC=COM_IB;
			//COM_IA=COM_IB;
			//Power_Factor = Factor;
		}
	}

	if (Idref > UDC_PI_Restrict)			//电压环PI输出限幅
	{
		Idref = UDC_PI_Restrict;
	}
	if (Idref < -UDC_PI_Restrict)
	{
		Idref = -UDC_PI_Restrict;
	}
	//得到alpha，beta坐系碌奈妊怪噶�主用用于分次补偿
	ACON2S2R_W(Idref, 0, THETA_C);
	Udc_alpha = I1_a;
	Udc_beta = I1_b;

//	Udc_d = Idref * VolComdGain;//稳压指令d轴（有功）分量
//	Udc_q = 0;//稳压指令q轴分量

//*******************pq_pi修改********************
	//ACON2S2R_W( Idref, 0 );
	//ACON3S2S( I1_a, I1_b );
	//Udc_a = LS1 * VolComdGain;//0.5;//*0.2; //0.016; // *I_UDCDB1需要四个ū晗凳�
	//Udc_b = LS2 * VolComdGain;//0.5;//*0.2;
	//Udc_c = LS3 * VolComdGain;//0.5;//*0.2;
//*****************pq_pi修改结束*****************
//*****************************谐波电流计算**************************/

	//*****************pq_pi修改添加部分结束*********************
//DRAM数据传递
//	*(DRAM_data+0)=UDC_R;
//	*(DRAM_data+1)=THETA_C;
//	*(DRAM_data+2)=CROSS_ZERO;
//	*(DRAM_data+3)=UAADRF;
//	*(DRAM_data+4)=UBADRF;
//	*(DRAM_data+5)=UCADRF;
//	*(DRAM_data+6)=IAADRF;// * 8.0;
//	*(DRAM_data+7)=IBADRF;// * 8.0;
//	*(DRAM_data+8)=ICADRF;// * 8.0;
//	*(DRAM_data+9)=I0ADRF;// * 8.0;
//	*(DRAM_data+0x7FFF)=0x1111;	 	//逻辑时序参考
//补偿指令用斜坡函数增加
	T_cout_U += 0.1;				//T_cout_U增加用于实现先稳压后钩�
	if (T_cout_U > 100)
	{

		T_cout += 0.1;
		if (T_cout > 300)			//T_Flag、T_cout用于实现电流的缓慢增加，尤其是在大电流时应用
		{
			T_cout = 0;
			T_Flag += 1;
		}
		if (T_Flag > 20)
		{
			T_cout = 300;
			T_Flag = 20;
		}
		T_cout_U = 100;
	}
	if (UDC_Rf1 > (UDCREF + 25)) 			//此段程序用于实现突降负载时电压的稳定
		T_Flag = 16;
	if (UDC_Rf1 > (UDCREF + 28))
		T_Flag = 12;
	if (UDC_Rf1 > (UDCREF + 31))
		T_Flag = 10;
	if (UDC_Rf1 > (UDCREF + 35))
		T_Flag = 8;
	if (UDC_Rf1 > (UDCREF + 38))
		T_Flag = 4;
	if (UDC_Rf1 > (UDCREF + 42))
		T_Flag = 0;

	if ((C_way && 0x02) == 0)						//分次补偿
	{
		Check_Harmonic_N(COM_IH1, COM_IH1_RATE, FilterPara1);
		IAh_Sum = IAh;
		IBh_Sum = IBh;
		ICh_Sum = ICh;
		Check_Harmonic_N(COM_IH2, COM_IH2_RATE, FilterPara2);
		IAh_Sum += IAh;
		IBh_Sum += IBh;
		ICh_Sum += ICh;
		Check_Harmonic_N(COM_IH3, COM_IH3_RATE, FilterPara3);
		IAh_Sum += IAh;
		IBh_Sum += IBh;
		ICh_Sum += ICh;
		Check_Harmonic_N(COM_IH4, COM_IH4_RATE, FilterPara4);
		IAh_Sum += IAh;
		IBh_Sum += IBh;
		ICh_Sum += ICh;

		Ih_a = IAh_Sum * T_Flag * 0.05;
		Ih_b = IBh_Sum * T_Flag * 0.05;
		Ih_c = ICh_Sum * T_Flag * 0.05;
		//Ih_0 = -(Ih_a + Ih_b + Ih_c);

	}
	else
	{
		Rate = COM_ALL_rate * 0.05 * T_Flag * 0.05;

		CON2S2R_W(IAADRF, THETA_C);

		I1_adf = (1 - 0.01) * I1_adf;
		I1_adf += I1_d * 0.01;
		I1_aqf = (1 - 0.01) * I1_aqf;
		I1_aqf += I1_q * 0.01;

		ACON2S2R_W(I1_adf, I1_aqf, THETA_C);
		Ih_a = (IAADRF - I1_a * 2) * Rate;

		THETA_C_B = THETA_C + 240;
		while (THETA_C_B >= 360)						//使角度一直在零到360度之间循环
			THETA_C_B -= 360;

		CON2S2R_W(IBADRF, THETA_C_B);						//THETA_C);
		I1_bdf = (1 - 0.01) * I1_bdf;
		I1_bdf += I1_d * 0.01;
		I1_bqf = (1 - 0.01) * I1_bqf;
		I1_bqf += I1_q * 0.01;
		/*
		 Filter_All_X3[0] = I1_d;
		 I1_bdf = LP_Filter_2nd(Filter_All_Y3, Filter_All_X3);
		 Filter_All_X4[0] = I1_q;
		 I1_bqf = LP_Filter_2nd(Filter_All_Y4, Filter_All_X4);
		 */
		ACON2S2R_W(I1_bdf, I1_bqf, THETA_C_B);
		Ih_b = (IBADRF - I1_a * 2) * Rate;

		THETA_C_C = THETA_C + 120;
		while (THETA_C_C >= 360)						//使角度一直在零到360度之间循环
			THETA_C_C -= 360;

		CON2S2R_W(ICADRF, THETA_C_C);						//THETA_C);
		I1_cdf = (1 - 0.01) * I1_cdf;
		I1_cdf += I1_d * 0.01;
		I1_cqf = (1 - 0.01) * I1_cqf;
		I1_cqf += I1_q * 0.01;

		I1_adf *= 0.88;
		I1_aqf *= 0.88;
		I1_bdf *= 0.88;
		I1_bqf *= 0.88;
		I1_cdf *= 0.88;
		I1_cqf *= 0.88;

		/*Filter_All_X5[0] = I1_d;
		 I1_cdf = LP_Filter_2nd_A(Filter_All_Y5, Filter_All_X5);
		 Filter_All_X6[0] = I1_q;
		 I1_cqf = LP_Filter_2nd_A(Filter_All_Y6, Filter_All_X6);*/

		ACON2S2R_W(I1_cdf, I1_cqf, THETA_C_C);
		Ih_c = (ICADRF - I1_a * 2) * Rate;

		Check_Harmonic_N_S(5, Rate, FilterPara1); //采样电网电流进行逐次单闭环调节20131006
		IAh_Sum = 1.4 * IAh;
		IBh_Sum = 1.4 * IBh;
		ICh_Sum = 1.4 * ICh;
		Check_Harmonic_N_S(7, Rate, FilterPara2);
		IAh_Sum += IAh;
		IBh_Sum += IBh;
		ICh_Sum += ICh;

		Ihw_a = IAh_Sum * 3;
		Ihw_b = IBh_Sum * 3;
		Ihw_c = ICh_Sum * 3;

	}

	//Ih_0 = -(Ih_a + Ih_b + Ih_c);

//*************************基波无功补偿算法************************
	if (Reactive_Power == 2) //2
	{
		THETA_C_B = THETA_C + 240;
		while (THETA_C_B >= 360)						//使角度一直在零到360度之间循环
			THETA_C_B -= 360;

		THETA_C_C = THETA_C + 120;
		while (THETA_C_C >= 360)						//使角度一直在零到360度之间循环
			THETA_C_C -= 360;

		CON2S2R_W(IAADRF, THETA_C);
		I1_adf = (1 - 0.001) * I1_adf + I1_d * 0.001;
		I1_aqf = (1 - 0.001) * I1_aqf + I1_q * 0.001;

		CON2S2R_W(IBADRF, THETA_C_B);
		I1_bdf = (1 - 0.001) * I1_bdf + I1_d * 0.001;
		I1_bqf = (1 - 0.001) * I1_bqf + I1_q * 0.001;

		CON2S2R_W(ICADRF, THETA_C_C);
		I1_cdf = (1 - 0.001) * I1_cdf + I1_d * 0.001;
		I1_cqf = (1 - 0.001) * I1_cqf + I1_q * 0.001;

		if (COM_IHB_RATE <= 50)
		{
			I1_aqf_f = I1_aqf - (I1_adf * tan_theta[COM_IHB_RATE]);
			I1_bqf_f = I1_bqf - (I1_bdf * tan_theta[COM_IHB_RATE]);
			I1_cqf_f = I1_cqf - (I1_cdf * tan_theta[COM_IHB_RATE]);
		}
		else
		{
			I1_aqf_f = 0;
			I1_bqf_f = 0;
			I1_cqf_f = 0;
		}

		ACON2S2R_W(0, I1_aqf_f, THETA_C);
		If_a = I1_a * T_Flag * 0.1; //* T_Flag * 0.05 * 2
		ACON2S2R_W(0, I1_bqf_f, THETA_C_B);
		If_b = I1_a * T_Flag * 0.1; //* T_Flag * 0.05 * 2
		ACON2S2R_W(0, I1_cqf_f, (THETA_C_C + 90));
		If_c = I1_a * T_Flag * 0.1; //* T_Flag * 0.05 * 2

		Ih_a += If_a;
		Ih_b += If_b;
		Ih_c += If_c;

	}
	else if (Reactive_Power == 1) //1
	{
		I1_aqf_f = -1000;                         //固定无功20131007
		I1_bqf_f = -1000;
		I1_cqf_f = -1000;

		THETA_C_B = THETA_C + 240;
		while (THETA_C_B >= 360)						//使角度一直在零到360度之间循环
			THETA_C_B -= 360;

		THETA_C_C = THETA_C + 120;
		while (THETA_C_C >= 360)						//使角度一直在零到360度之间循环
			THETA_C_C -= 360;

		ACON2S2R_W(0, I1_aqf_f, THETA_C);
		If_a = I1_a * COM_IHB_RATE * 0.02 * T_Flag * 0.05;				// * 2
		ACON2S2R_W(0, I1_bqf_f, THETA_C_B);
		If_b = I1_a * COM_IHB_RATE * 0.02 * T_Flag * 0.05;				// * 2
		ACON2S2R_W(0, I1_cqf_f, THETA_C_C);
		If_c = I1_a * COM_IHB_RATE * 0.02 * T_Flag * 0.05;				// * 2

		Ih_a += If_a;
		Ih_b += If_b;
		Ih_c += If_c;
	}
	else if (Reactive_Power == 3)						//3
	{
		THETA_C_B = THETA_C + 240;
		while (THETA_C_B >= 360)						//使角度一直在零到360度之间循环
			THETA_C_B -= 360;

		THETA_C_C = THETA_C + 120;
		while (THETA_C_C >= 360)						//使角度一直在零到360度之间循环
			THETA_C_C -= 360;

		CON2S2R_W(IAADRF, THETA_C);
//	        I1_adf = (1 - 0.001) * I1_adf + I1_d * 0.001;
		I1_aqf = (1 - 0.001) * I1_aqf + I1_q * 0.001;

		CON2S2R_W(IBADRF, THETA_C_B);
//	        I1_bdf = (1 - 0.001) * I1_bdf + I1_d * 0.001;
		I1_bqf = (1 - 0.001) * I1_bqf + I1_q * 0.001;

		CON2S2R_W(ICADRF, THETA_C_C);
//	        I1_cdf = (1 - 0.001) * I1_cdf + I1_d * 0.001;
		I1_cqf = (1 - 0.001) * I1_cqf + I1_q * 0.001;

		ACON2S2R_W(0, I1_aqf, THETA_C);
		If_a = I1_a * T_Flag * 0.1 * COM_IHB_RATE * 0.02; // T_Flag * 0.05 * 2
		ACON2S2R_W(0, I1_bqf, THETA_C_B);
		If_b = I1_a * T_Flag * 0.1 * COM_IHB_RATE * 0.02; // T_Flag * 0.05 * 2
		ACON2S2R_W(0, I1_cqf, THETA_C_C);
		If_c = I1_a * T_Flag * 0.1 * COM_IHB_RATE * 0.02; // T_Flag * 0.05 * 2

		/*			Ih_a += If_a;
		 Ih_b += If_b;
		 Ih_c += If_c;
		 If_a = 0;
		 If_b = 0;
		 If_c = 0;
		 I1_aqf_f = 0;
		 I1_bqf_f = 0;
		 I1_cqf_f = 0;
		 */
	}

//***************************END************************************

//***************************超前校正******************************
	/*
	 * 超前校正的方法是：
	 * 先将当前的谐波保存到数组中去，然后取出上一个周期某个时刻保存的谐波值
	 */
	if ((Leading_Comp_Flag == 1) && ((C_way && 0x02) != 0))
	{
		if (COM_EXT_count > 0)
		{
			I_H_A[n_cycle] = Ih_a; //Ih_alpha;
			I_H_B[n_cycle] = Ih_b; //Ih_beta;
			I_H_C[n_cycle] = Ih_c;
			if (n_cycle >= (AllSampleCount - COM_EXT_count))
			{
				m_cycle = n_cycle - AllSampleCount + COM_EXT_count; //针对开关周期是80uS，一个周期采样250个点
			}
			else
			{
				m_cycle = n_cycle + AddSampleCount + COM_EXT_count;
			}
			Ih_a = I_H_A[m_cycle];
			Ih_b = I_H_B[m_cycle];
			Ih_c = I_H_C[m_cycle];
			n_cycle += 1;
			if (n_cycle >= (AllSampleCount + AddSampleCount))
			{
				n_cycle -= (AllSampleCount + AddSampleCount);
			}
		}
	}
	else
	{
		;
	}

	/*************************补偿电流控制**************************/

//***********************************END*******************************
	CON3S3S(IAAPF, IBAPF, ICAPF);	//计算APF的输出电流到静止坐标系的值
	I_apf_alpha = LS1;
	I_apf_beta = LS2;
	I_apf_0 = LS3;

	CON3S3S(Ih_a, Ih_b, Ih_c);	//计算谐波电流在静止坐标系下的值
	Ih_alpha = LS1;
	Ih_beta = LS2;
	Ih_0x = LS3;

//***************************限流处理**********************************

	Ih_Limit = Ih_Max * C_SQRT23 * 1.15; //0.04为电流霍尔量程的25A的倒数，若修改为200A，则将0.04修改为0.005
	CON2S2R_W_3(Ih_alpha * 0.1, Ih_beta * 0.1);
	Ih_d_Limit = I1_d;
	Ih_q_Limit = I1_q;

	I1_d_square = Ih_d_Limit * Ih_d_Limit;
	I1_q_square = Ih_q_Limit * Ih_q_Limit;

	I1_Square = I1_d_square + I1_q_square;
	Ih_Limit_square = Ih_Limit * Ih_Limit;

	if (I1_Square > Ih_Limit_square)
	{
		I1_Value = sqrt(Ih_Limit_square / I1_Square);
		Ih_d_Limit = Ih_d_Limit * I1_Value;
		Ih_q_Limit = Ih_q_Limit * I1_Value;
		ACON2S2R_W_3(Ih_d_Limit * 10, Ih_q_Limit * 10);
		Ih_alpha = I1_a;
		Ih_beta = I1_b;
	}
	else
	{
//*******************重复控制器部分****************************
/* XXX 重复控制
 * 超前控制与重复控制有一些类似
 * 重复控制的控制框图请参加学习笔记
 * 重复控制的输入e(k)与输出u(k)的关系是：
 * u(k)=e(k-N)+kr*u(k-N)
 * 因此需要用数组保存一定量的数据
 */
		if (Repiti_Sign == 1) //alpha,beta坐标系下
		{
			CON3S3S(Ih_a, Ih_b, Ih_c);
			Ih_alpha = LS1;
			Ih_beta = LS2;
			Ih_0x = LS3;

			alpha_New_error = Ih_alpha - Udc_alpha - I_apf_alpha;
			beta_New_error = Ih_beta - Udc_beta - I_apf_beta;

			if (Repiti_Count >= AllSampleCount)
			{
				Repiti_Count -= AllSampleCount;
			}

			if ((AllSampleCount - Repiti_Count) > Repiti_lead)
			{
				CycleErrorHit = Repiti_Count + Repiti_lead;
			}
			else
			{
				CycleErrorHit = Repiti_Count + Repiti_lead - AllSampleCount;
			}

			Repiti_alpha_NewOut = Qz * Repiti_alpha_CycleOut[Repiti_Count]
					+ Kr * alpha_Cycle_error[CycleErrorHit];
			Repiti_beta_NewOut = Qz * Repiti_beta_CycleOut[Repiti_Count]
					+ Kr * beta_Cycle_error[CycleErrorHit];
			alpha_Cycle_error[Repiti_Count] = alpha_New_error;
			beta_Cycle_error[Repiti_Count] = beta_New_error;
			Repiti_alpha_CycleOut[Repiti_Count] = Repiti_alpha_NewOut;
			Repiti_beta_CycleOut[Repiti_Count] = Repiti_beta_NewOut;
			Repiti_Count++;

			ACON3S2S(Repiti_alpha_NewOut, Repiti_beta_NewOut);
			ModuComdA1 = LS1;
			ModuComdB1 = LS2;
			ModuComdC1 = LS3;

		}
//*******************重复控制器部分结束***********************
		CON3S2S(Ihw_a, Ihw_b, Ihw_c);
		Ih_alpha = Ih_alpha + LS1;
		Ih_beta = Ih_beta + LS2;
	}

	//********************PI调节器部分****************************
	I_Kp = UDC_Rf2 * 0.5;  //Current_Kp;//40;//(float) COM_IHB_RATE;
	Current_PI(Ih_alpha - Udc_alpha - I_apf_alpha,
			Ih_beta - Udc_beta - I_apf_beta, I_Kp, I_Ki);
//	Current_PI( - Udc_alpha- I_apf_alpha,  -Udc_beta - I_apf_beta, I_Kp, I_Ki );
	Comd_alpha = PI_alpha;
	Comd_beta = PI_beta;

	Current_PI(Ih_0x - I_apf_0, 0, I_Kp, I_Ki);
	Comd_0 = PI_alpha;

	ACON3S3S(Comd_alpha, Comd_beta, Comd_0);
	ModuComdA = LS1;
	ModuComdB = LS2;
	ModuComdC = LS3;
	ModuComd0 = -(ModuComdA + ModuComdB + ModuComdC);

	//**********************PI调节器结束**************************
	/*******************************前馈电压*****************************/

	U1_a = (UAADRF - UCADRF) * UA_DB2;
	U1_b = (UBADRF - UAADRF) * UA_DB2;
	U1_c = (UCADRF - UBADRF) * UA_DB2;
	/*    CON3S2S( U1_a, U1_b, U1_c ) ;				//全补偿
	 tempp1 = LS1;
	 tempp2 = LS2;
	 CON2S2R_W( tempp1, tempp2 );
	 U1_d=(1 - 0.001) * U1_d + I1_d * 0.001;			//2Hz滤波限
	 U1_q=(1 - 0.001) * U1_q + I1_q * 0.001;
	 ACON2S2R_W( U1_d, U1_q );
	 tempp1 = I1_a;
	 tempp2 = I1_b;
	 ACON3S2S( tempp1, tempp2 );
	 U1_a = LS1;
	 U1_b = LS2;
	 U1_c = LS3;
	 */
	/*
	 * XXX 电压前馈的原则
	 * 这里面有一个电压调制系数的问题
	 * 假设直流母线电压为Udc，则SPWM的调制相电压的基波幅值最高到Udc/2
	 * 因此待调制的相电压幅值为Ux，则相应的占空比D为
	 * D=3000*Ux/(Udc/2)=6000*Ux/Udc
	 * 而假设如果是SVPWM调制，则基波相电压的幅值最高可以到sqrt(3)/3*Udc
	 */
	if (UDC_Rf1 > 20)
	{
		tempp1 = U_FtoI * 0.1 * 5196 / (UDC_Rf1);
		U1_a *= tempp1;				//UA*sqrt(2)/UDC=M 髦票?
		U1_b *= tempp1;
		U1_c *= tempp1;
	}

//**********************PI调节器结束**************************

	Ua_APF = ModuComdA + U1_a + Tri_wave_amp + ModuComdA1;
	Ub_APF = ModuComdB + U1_b + Tri_wave_amp + ModuComdB1;
	Uc_APF = ModuComdC + U1_c + Tri_wave_amp + ModuComdC1;
	U0_APF = ModuComd0 + Tri_wave_amp;

	if (Ua_APF > ModuWaveLimit_High)	//调制波限幅
	{
		Ua_APF = ModuWaveLimit_High;
	}
	if (Ua_APF < ModuWaveLimit_Low)
	{
		Ua_APF = ModuWaveLimit_Low;
	}
	if (Ub_APF > ModuWaveLimit_High)
	{
		Ub_APF = ModuWaveLimit_High;
	}
	if (Ub_APF < ModuWaveLimit_Low)
	{
		Ub_APF = ModuWaveLimit_Low;
	}
	if (Uc_APF > ModuWaveLimit_High)
	{
		Uc_APF = ModuWaveLimit_High;
	}
	if (Uc_APF < ModuWaveLimit_Low)
	{
		Uc_APF = ModuWaveLimit_Low;
	}
	if (U0_APF > ModuWaveLimit_High)
	{
		U0_APF = ModuWaveLimit_High;
	}
	if (U0_APF < ModuWaveLimit_Low)
	{
		U0_APF = ModuWaveLimit_Low;
	}
//读取DRAM数据
//  	if(GpioDataRegs.GPADAT.bit.GPIO18==0)
//	while ( GpioDataRegs.GPADAT.bit.GPIO18 == 1 )
//	{
//	    ;
//	}
	//{

//		ERROR_flag|=(int) *(DRAM_data + 9);

	THD_IA = (int) (*(DRAM_data + 11) * 0.0);  	//0.288
	THD_IB = (int) (*(DRAM_data + 12) * 0.0);  	//0.602
	THD_IC = (int) (*(DRAM_data + 13) * 0.0);  	//0.507
	COM_I0 = (int) (*(DRAM_data + 14) * 8);  	//1.67
	COM_IA = (int) (*(DRAM_data + 15) * 8);  	//3.99
	COM_IB = (int) (*(DRAM_data + 16) * 8);  	//2.602
	COM_IC = (int) (*(DRAM_data + 17) * 8);  	//3.399
	CompensateA = (int) (*(DRAM_data + 18) * COM_ALL_rate * T_Flag * 0.070588); // + I_P * COM_IHB_RATE * 0.0357 * I_Q_flag);//
	CompensateB = (int) (*(DRAM_data + 19) * COM_ALL_rate * T_Flag * 0.070588); // + I_P * COM_IHB_RATE * 0.0357 * I_Q_flag);//
	CompensateC = (int) (*(DRAM_data + 18) * COM_ALL_rate * T_Flag * 0.070588); // + I_P * COM_IHB_RATE * 0.0357 * I_Q_flag);//
	THD_IA3 = (int) (*(DRAM_data + 21) * 0.046);
	THD_IA5 = (int) (*(DRAM_data + 22) * 1.03);
	THD_IA7 = (int) (*(DRAM_data + 23) * 1.71);
	THD_IA9 = (int) (*(DRAM_data + 24) * 0.1);
	THD_IA11 = (int) (*(DRAM_data + 25) * 2.25);

	INRL_data = (int) *(DRAM_data + 0x7FFE);
	//}

	ERROR_flag &= 0x2F;

	EPwm1Regs.CMPA.half.CMPA = (int) Ua_APF; // adjust duty for output EPWM1A
	EPwm2Regs.CMPA.half.CMPA = (int) Ub_APF; // adjust duty for output EPWM2A
	EPwm3Regs.CMPA.half.CMPA = (int) Uc_APF; // adjust duty for output EPWM3A
	EPwm4Regs.CMPA.half.CMPA = (int) U0_APF;
	if ((Run_Stop_Flag == 1) && (ERROR_flag == 0))	  		//若APF启动
	{
		EN_PWM();
	}
	else
	{
		DIS_PWM();
	}
// Clear INT flag for this timer
	EPwm1Regs.ETCLR.bit.INT = 1;
// Acknowledge this interrupt to get more from group 1
	PieCtrlRegs.PIEACK.all = 0x0004;
	GpioDataRegs.GPASET.bit.GPIO30 = 1;   //使能F28335B中
	EINT;
}
void UDC_PI(float Kp, float Ki)      //PI调节函数
{
	float temp1 = 0;
//		float temp2 = Inref;        //积分项
	float temp3 = (Idref - Inref);      //比例项

	temp1 = UDC_erro_new * UDC_DELT * Ki * 0.1;	//12.5K 一次  UDC_DELT=0.00008 s
	Inref += temp1;

	if (Inref > ID_MAXOUT)                 //积分项限幅
	{
		Inref = ID_MAXOUT;
	}

	if (Inref < -ID_MAXOUT)
	{
		Inref = -ID_MAXOUT;
	}

	temp1 = Kp * (UDC_erro_new - UDC_erro_old) * 0.1;
	temp3 += temp1;
	Idref = (temp3 + Inref);

	UDC_erro_old = UDC_erro_new;
}
float Check_Phase(float Ua, float Ub)     	//相位检测函数（含代计算三相电网相电压）
{
	float Uc;
	int temp;
	float U_alpha;
	float U_beta;
	float temp1;
	float temp2;
	float temp3;
	float theta;

	Uc = -(Ua + Ub);     	//UCADRF;

//交流电压实际值计算
	U1_a = (Ua - Uc) * UA_DB2;
	U1_b = (Ub - Ua) * UA_DB2;
	U1_c = (Uc - Ub) * UA_DB2;
//注意:比较时,正弦越大,输出越大

	if (UDC_Rf1 > 20)
	{
		//
		temp1 = U_FtoI * 0.1 * 6000 / UDC_Rf1;
		//temp1*=C_SQRT2;
		U1_a *= temp1;				//UA*sqrt(2)/UDC=M 髦票�
		U1_b *= temp1;
		U1_c *= temp1;

	}

	CON3S2S(U1_a, U1_b, U1_c);
	U_alpha = LS1;
	U_beta = LS2;
	if (U_alpha < 0)
		temp1 = -U_alpha;
	else
		temp1 = U_alpha;
	if (U_beta < 0)
		temp2 = -U_beta;
	else
		temp2 = U_beta;

	//计算tan(theta)值，查表得到角度
	if ((U_alpha > 0) && (U_beta == 0))				//0
	{
		theta = 0;
	}
	else if ((U_alpha == 0) && (U_beta > 0))		//90
	{
		theta = 90;
	}
	else if ((U_alpha < 0) && (U_beta == 0))		//180
	{
		theta = 180;
	}
	else if ((U_alpha == 0) && (U_beta < 0))		//270
	{
		theta = 270;
	}
	else
	{
		if (U_alpha > 0)			//1,4象限
		{
			if (U_beta > 0)			//1象限
			{
				if (temp1 >= temp2)	//0-45
				{
					temp3 = 256 * temp2 / temp1;
					temp = (int) (temp3);
					theta = ARCTG_TABLE[temp];							//不需插值
				}
				else						//45-90
				{
					temp3 = 256 * temp1 / temp2;
					temp = (int) (temp3);
					theta = 90 - ARCTG_TABLE[temp];						//不需插值
				}
			}
			else							//4象限
			{
				if (temp1 >= temp2)	//315-360
				{
					temp3 = 256 * temp2 / temp1;
					temp = (int) (temp3);
					theta = 360 - ARCTG_TABLE[temp];					//不需插值
				}
				else						//270-315
				{
					temp3 = 256 * temp1 / temp2;
					temp = (int) (temp3);
					theta = 270 + ARCTG_TABLE[temp];					//不需插值
				}
			}
		}
		else				//2,3象限
		{
			if (U_beta > 0)			//2象限
			{
				if (temp1 >= temp2)	//135-180
				{
					temp3 = 256 * temp2 / temp1;
					temp = (int) (temp3);
					theta = 180 - ARCTG_TABLE[temp];					//不需插值
				}
				else						//90-135
				{
					temp3 = 256 * temp1 / temp2;
					temp = (int) (temp3);
					theta = 90 + ARCTG_TABLE[temp];						//不需插值
				}
			}
			else							//3象限
			{
				if (temp1 >= temp2)	//180-225
				{
					temp3 = 256 * temp2 / temp1;
					temp = (int) (temp3);
					theta = 180 + ARCTG_TABLE[temp];						//徊宓
				}
				else						//225-270
				{
					temp3 = 256 * temp1 / temp2;
					temp = (int) (temp3);
					theta = 270 - ARCTG_TABLE[temp];					//不需插值
				}
			}
		}

	}
	return theta;
}
float Check_Theta(float U_a, float U_b)     	//仅为三相计算角度
{
	int temp;
	float U_alpha;
	float U_beta;
	float temp1;
	float temp2;
	float temp3;
	float theta;
	/*Uc=-(Ua+Ub);
	 CON3S2S(Ua,Ub,Uc);
	 U_alpha=LS1;
	 U_beta=LS2;*/
	U_alpha = U_a;
	U_beta = U_b;
	if (U_alpha < 0)
		temp1 = -U_alpha;
	else
		temp1 = U_alpha;
	if (U_beta < 0)
		temp2 = -U_beta;
	else
		temp2 = U_beta;
	//计算tan(theta)值，查表得到角度
	if ((U_alpha > 0) && (U_beta == 0))				//0
	{
		theta = 0;
	}
	else if ((U_alpha == 0) && (U_beta > 0))		//90
	{
		theta = 90;
	}
	else if ((U_alpha < 0) && (U_beta == 0))		//180
	{
		theta = 180;
	}
	else if ((U_alpha == 0) && (U_beta < 0))		//270
	{
		theta = 270;
	}
	else
	{
		if (U_alpha > 0)			//1,4象限
		{
			if (U_beta > 0)			//1象限
			{
				if (temp1 >= temp2)	//0-45
				{
					temp3 = 256 * temp2 / temp1;
					temp = (int) (temp3);
					theta = ARCTG_TABLE[temp];							//不需插值
				}
				else						//45-90
				{
					temp3 = 256 * temp1 / temp2;
					temp = (int) (temp3);
					theta = 90 - ARCTG_TABLE[temp];						//不需插值
				}
			}
			else							//4象限
			{
				if (temp1 >= temp2)	//315-360
				{
					temp3 = 256 * temp2 / temp1;
					temp = (int) (temp3);
					theta = 360 - ARCTG_TABLE[temp];					//不需插值
				}
				else						//270-315
				{
					temp3 = 256 * temp1 / temp2;
					temp = (int) (temp3);
					theta = 270 + ARCTG_TABLE[temp];					//不需插值
				}
			}
		}
		else				//2,3象限
		{
			if (U_beta > 0)			//2象限
			{
				if (temp1 >= temp2)	//135-180
				{
					temp3 = 256 * temp2 / temp1;
					temp = (int) (temp3);
					theta = 180 - ARCTG_TABLE[temp];					//不需插值
				}
				else						//90-135
				{
					temp3 = 256 * temp1 / temp2;
					temp = (int) (temp3);
					theta = 90 + ARCTG_TABLE[temp];						//不需插值
				}
			}
			else							//3象限
			{
				if (temp1 >= temp2)	//180-225
				{
					temp3 = 256 * temp2 / temp1;
					temp = (int) (temp3);
					theta = 180 + ARCTG_TABLE[temp];					//不需插值
				}
				else						//225-270
				{
					temp3 = 256 * temp1 / temp2;
					temp = (int) (temp3);
					theta = 270 - ARCTG_TABLE[temp];					//不需插值
				}
			}
		}

	}
	return theta;
}

void Check_Harmonic_N(int H_n, int H_n_rate, float * filter_para)
{
	float temp = 0;
	float IAd, IAq, IBd, IBq, ICd, ICq;

	temp = THETA_C * H_n;
	while (temp >= 360)
	{
		temp -= 360;
	}
	Sin_Value(temp);

	IAd = IAADRF * cosVal;
	IAq = -IAADRF * sinVal;
	IBd = IBADRF * cosVal;
	IBq = -IBADRF * sinVal;
	ICd = ICADRF * cosVal;
	ICq = -ICADRF * sinVal;

	IAdh = (1 - Ts[1]) * filter_para[0];
	IAdh += IAd * Ts[1];
	filter_para[0] = IAdh;
	IAqh = (1 - Ts[1]) * filter_para[1];
	IAqh += IAq * Ts[1];
	filter_para[1] = IAqh;

	IBdh = (1 - Ts[1]) * filter_para[2];
	IBdh += IBd * Ts[1];
	filter_para[2] = IBdh;
	IBqh = (1 - Ts[1]) * filter_para[3];
	IBqh += IBq * Ts[1];
	filter_para[3] = IBqh;

	ICdh = (1 - Ts[1]) * filter_para[4];
	ICdh += ICd * Ts[1];
	filter_para[4] = ICdh;
	ICqh = (1 - Ts[1]) * filter_para[5];
	ICqh += ICq * Ts[1];
	filter_para[5] = ICqh;
//if(H_n=5)
	//temp = H_n * (THETA_C + 90) + H_n * 18 * Sampl_Interval * (COM_EXT_count + 4) * 0.5 ;
	temp = H_n * (THETA_C) + H_n * 18 * Sampl_Interval * COM_EXT_count;	//超前3个采样点。; //3
	while (temp >= 360)
	{
		temp -= 360;
	}
	Sin_Value(temp);

	IAh = 2 * (IAdh * cosVal - IAqh * sinVal) * H_n_rate * 0.05;
	IBh = 2 * (IBdh * cosVal - IBqh * sinVal) * H_n_rate * 0.05;
	ICh = 2 * (ICdh * cosVal - ICqh * sinVal) * H_n_rate * 0.05;
}

void Check_Harmonic_N_S(int H_n, float H_n_rate, float * filter_para)
{
	float temp = 0;
	float IAd, IAq, IBd, IBq, ICd, ICq;

	temp = THETA_C * H_n;
	while (temp >= 360)
	{
		temp -= 360;
	}
	Sin_Value(temp);

	IAd = (IAADRF * H_n_rate - IAAPF) * cosVal;
	IAq = -(IAADRF * H_n_rate - IAAPF) * sinVal;
	IBd = (IBADRF * H_n_rate - IBAPF) * cosVal;
	IBq = -(IBADRF * H_n_rate - IBAPF) * sinVal;
	ICd = (ICADRF * H_n_rate - ICAPF) * cosVal;
	ICq = -(ICADRF * H_n_rate - ICAPF) * sinVal;

	IAdh = (1 - Ts[1]) * filter_para[0];
	IAdh += IAd * Ts[1];
	filter_para[0] = IAdh;
	IAqh = (1 - Ts[1]) * filter_para[1];
	IAqh += IAq * Ts[1];
	filter_para[1] = IAqh;

	IBdh = (1 - Ts[1]) * filter_para[2];
	IBdh += IBd * Ts[1];
	filter_para[2] = IBdh;
	IBqh = (1 - Ts[1]) * filter_para[3];
	IBqh += IBq * Ts[1];
	filter_para[3] = IBqh;

	ICdh = (1 - Ts[1]) * filter_para[4];
	ICdh += ICd * Ts[1];
	filter_para[4] = ICdh;
	ICqh = (1 - Ts[1]) * filter_para[5];
	ICqh += ICq * Ts[1];
	filter_para[5] = ICqh;

	temp = H_n * (THETA_C) + H_n * 18 * Sampl_Interval * 4.5;//COM_EXT_count;//4;//COM_EXT_count * 0.8;//超前3个采样点。; //3
	while (temp >= 360)
	{
		temp -= 360;
	}
	Sin_Value(temp);

	IAh = 2 * (IAdh * cosVal - IAqh * sinVal);
	IBh = 2 * (IBdh * cosVal - IBqh * sinVal);
	ICh = 2 * (ICdh * cosVal - ICqh * sinVal);
}
/*
 void CON2S2R_W_n(float U_alpha,float U_beta,int n)
 {
 float temp;
 temp=n * THETA_C;
 while(temp>=360)
 temp-=360;
 Sin_Value(temp);

 //In_d=U_alpha*cosVal+U_beta*sinVal;
 //In_q=U_beta*cosVal-U_alpha*sinVal;
 if( n%3==2 )
 {
 In_d = U_alpha * cosVal - U_beta * sinVal;		//空间电压矢量负序旋�
 In_q = U_beta * cosVal + U_alpha * sinVal;
 }
 else if( n%3==1 )
 {
 In_d = U_alpha * cosVal + U_beta * sinVal;
 In_q = U_beta * cosVal - U_alpha * sinVal;
 }
 else
 {
 In_d = 0;
 In_q = 0;
 }
 }

 void ACON2S2R_W_n(float Um,float Ut,int n)
 {
 float temp;
 //补偿采样延时时间，不同次谐波补偿难邮苯嵌妊邮苯嵌炔煌�
 //	temp=n*(THETA_C+Theta_SD)+n*18*Sampl_Interval*SeptCompAhead;
 //temp=n*(THETA_C)+n*18*Sampl_Interval*SeptCompAhea;
 temp = n * (THETA_C);// + n * 18 * Sampl_Interval * 0.1;//COM_IH3_RATE * 0.1;//将COM_IH3作为超前点数用，并缩小10倍。
 while( temp >= 360 )
 temp -= 360;
 Sin_Value( temp );
 //In_a=Um*cosVal-Ut*sinVal;
 //In_b=Ut*cosVal+Um*sinVal;
 if( (n % 3) == 2 )
 {
 In_a = Um * cosVal + Ut * sinVal;	//空间电压矢量负序旋转
 In_b = Ut * cosVal - Um * sinVal;
 }
 else if( (n % 3) == 1 )
 {
 In_a = Um * cosVal - Ut * sinVal;
 In_b = Ut * cosVal + Um * sinVal;
 }
 else
 {
 In_a = 0;
 In_b = 0;
 }
 }
 */

/**********************end******************************/
float Rectifier(float Uab, float Ubc, float Uca)
{
	float Ua;
	float Ub;
	float Uc;
	float U_temp;
	float U_rectifer;

	Ua = Uab * 2.5;
	Ub = Ubc * 2.5;
	Uc = Uca * 2.5;

	if (Ua < 0)
	{
		Ua = -Ua;
	}
	else
		;
	if (Ub < 0)
	{
		Ub = -Ub;
	}
	else
		;
	if (Uc < 0)
	{
		Uc = -Uc;
	}
	else
		;

	if (Ua <= Ub)
	{
		U_temp = Ub;
	}
	else
	{
		U_temp = Ua;
	}
	if (U_temp <= Uc)
	{
		U_rectifer = Uc;
	}
	else
	{
		U_rectifer = U_temp;
	}
	U_rectifer = 1200;
	return U_rectifer;
}
/********************全补****************************/

//电网电压显示时用到的3S/2S变换
void Display_CON3S2S(float U, float V, float W)
{
	float temp;

	temp = U - (V + W) * C_COS60;
	DisplayLS1 = temp * C_32COEF;				//A=sqrt(2/3)*(U-(V+W)*cos60)

	temp = (V - W) * C_SIN60;
	DisplayLS2 = temp * C_32COEF;				//B=sqrt(2/3)*((V-W)*sin60)

}

//静止坐标变换
void CON3S2S(float U, float V, float W)
{
	float temp;

	temp = U - (V + W) * C_COS60;
	LS1 = temp * C_32COEF;				//A=sqrt(2/3)*(U-(V+W)*cos60)

	temp = (V - W) * C_SIN60;
	LS2 = temp * C_32COEF;				//B=sqrt(2/3)*((V-W)*sin60)

}

void CON3S3S(float U, float V, float W)
{
	float temp;

	temp = U - (V + W) * C_COS60;
	LS1 = temp * C_32COEF;				//A=sqrt(2/3)*(U-(V+W)*cos60)

	temp = (V - W) * C_SIN60;
	LS2 = temp * C_32COEF;				//B=sqrt(2/3)*((V-W)*sin60)

	temp = (U + V + W) * 0.707106;
	LS3 = temp * C_32COEF;
}

void ACON3S2S(float Ua, float Ub)
{
	LS1 = Ua * C_32COEF;				//A=sqrt(2/3)*Ualpha
	LS2 = (-C_COS60 * Ua + Ub * C_SIN60) * C_32COEF;//B=sqrt(2/3)*(-cos60*Ualpha+sin60*Ubeta)
	LS3 = (-C_COS60 * Ua - Ub * C_SIN60) * C_32COEF;//C=sqrt(2/3)*(-cos60*Ualpha-sin60*Ubeta)
}

void ACON3S3S(float Ua, float Ub, float Uc)
{
	LS1 = (Ua + Uc * 0.707106) * C_32COEF;			//sqrt(2/3)[iα+iγ/sqrt(2)]
	LS2 = (-C_COS60 * Ua + Ub * C_SIN60 + Uc * 0.707106) * C_32COEF;//sqrt(2/3)[-0.5*iα+iγ/sqrt(2)+iβ*sqrt(3)/2]
	LS3 = (-C_COS60 * Ua - Ub * C_SIN60 + Uc * 0.707106) * C_32COEF;//sqrt(2/3)[-0.5*iα+iγ/sqrt(2)-iβ*sqrt(3)/2]
}
//旋转坐标变换

/********************************旋转坐标变换***************************************/
void CON2S2R_W_3(float U_alpha, float U_beta)
{

	//坐标变换 cos   sin
	//        -sin   cos

	Sin_Value(THETA_C);

	I1_d = U_alpha * cosVal + U_beta * sinVal;
	I1_q = -U_alpha * sinVal + U_beta * cosVal;
}
void CON2S2R_W(float U_alpha, float THETA_THETA)
{
	Sin_Value(THETA_THETA);

	I1_d = U_alpha * cosVal;
	I1_q = -U_alpha * sinVal;
}
/********************************转坐标反变换***************************************/

void ACON2S2R_W(float Um, float Ut, float Theta)
{
	/*
	 坐标变换 cos   -sin
	 sin   cos
	 */
	float temp;

	temp = Theta + Theta_SD + 18 * Sampl_Interval * 0.5;//Theta_SD;//THETA_C + Theta_SD;    //补偿采样延时时间,因为是在直流轴上做滤波,故不用补滤波延时
	while (temp >= 360)
	{
		temp -= 360;
	}
	Sin_Value(temp);
	I1_a = Um * cosVal - Ut * sinVal;
	I1_b = Um * sinVal + Ut * cosVal;
}
void ACON2S2R_W_3(float Um, float Ut)
{
	/*
	 坐标变换 cos   -sin
	 sin   cos
	 */
	float temp;

	temp = THETA_C;	// + Theta_SD;		//补偿采样延时时间,因为是在直流轴上做滤波,故不用补滤波延时
	while (temp >= 360)
	{
		temp -= 360;
	}
	Sin_Value(temp);
	I1_a = Um * cosVal - Ut * sinVal;
	I1_b = Um * sinVal + Ut * cosVal;
}
/*****************电流调节******************/
//***************pq_pi添加部分****************************************
void Current_PI(float Er_Id, float Er_Iq, float Kp, float ki)
{

	PI_alpha = Er_Id * Kp * 0.2;
	PI_beta = Er_Iq * Kp * 0.2;

}
//**************pq_pi添加部分结束*************************************

//求sin 和cos 值
void Sin_Value(float theta)
{
	int temp;
	float AB[2] =
	{ 0, 0 };

	if (theta >= 360)
		theta -= 360;

	if (theta == 0)				//sin(0)
	{
		AB[0] = 0;
		AB[1] = 1;
	}
	else if (theta == 90)		//sin(90)
	{
		AB[0] = 1;
		AB[1] = 0;
	}
	else if (theta == 180)		//sin(180)
	{
		AB[0] = 0;
		AB[1] = -1;
	}
	else if (theta == 270)		//sin(270)
	{
		AB[0] = -1;
		AB[1] = 0;
	}
	else
	{
		if (theta < 90)
		{
			temp = (int) (theta * Theta_DB);		//sin
			AB[0] = SIN_TABLE[temp];
			temp = (int) ((90 - theta) * Theta_DB);	//cos=sin(90-theta)
			AB[1] = SIN_TABLE[temp];
		}
		else if (theta < 180)
		{
			temp = (int) ((180 - theta) * Theta_DB);		//sin
			AB[0] = SIN_TABLE[temp];
			temp = (int) ((theta - 90) * Theta_DB);	//cos=sin(90-theta)
			AB[1] = -SIN_TABLE[temp];
		}
		else if (theta < 270)
		{
			temp = (int) ((theta - 180) * Theta_DB);		//sin
			AB[0] = -SIN_TABLE[temp];
			temp = (int) ((270 - theta) * Theta_DB);	//cos=sin(90-theta)
			AB[1] = -SIN_TABLE[temp];
		}
		else if (theta < 360)
		{
			temp = (int) ((360 - theta) * Theta_DB);		//sin
			AB[0] = -SIN_TABLE[temp];
			temp = (int) ((theta - 270) * Theta_DB);	//cos=sin(90-theta)
			AB[1] = SIN_TABLE[temp];
		}
	}
	sinVal = AB[0];
	cosVal = AB[1];
}

void EN_PWM(void)
{
	//将相应的IO置低
	GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1;

}
void DIS_PWM(void)
{
	//将相应的IO置高
	GpioDataRegs.GPBSET.bit.GPIO56 = 1;
}

void Clear_Comd(void)
{
	unsigned i = 0;
	for (i = 0; i < 257; i++)
	{
		I_H_A[i] = 0;
		I_H_B[i] = 0;
		I_H_C[i] = 0;
	}
	//补偿令清零
}

/******************************通用函数*********************************/
void delay_t(int a)
{

	for (n = 0; n < a; n++)
	{
		;
	}
}
void delay_t1(int a)
{

	for (n = 0; n < a; n++)
	{
		;
	}
}
float LP_Filter_2nd_A(float *y, float *x)
{

	// 入口：二阶低通滤波器输入值数组首地址，x[]包含3个变量，x[0]输入新值
	//      x[1]输入的前一次值，x[2]输入的前两次值。
	//      二阶低通滤波器以前输出值数组首地址,其中y[0]表示输出的前一次值，
//		y[1]表示输出的前两次值。
	//  出口：二阶低通滤波器输出值。
	//  中间变量：y_new用于暂时存储二阶低通滤波器的当前输出值。
	//  使用条件：滤波器的常数对应的采样频率为12.5KHz，截止频率fn = 950Hz;

	float y_new;
	y_new = (FilterCnstA_2nd_A * y[0]) - (FilterCnstB_2nd_A * y[1])
			+ (FilterCnstC_2nd_A * x[0]) + (FilterCnstD_2nd_A * x[1])
			+ (FilterCnstE_2nd_A * x[2]);

	y[1] = y[0];
	y[0] = y_new;
	x[2] = x[1];
	x[1] = x[0];
	return y_new;
}

/*float BP_Filter_1st_A(float *y, float *x)
 {

 // 入口：二阶低通滤波器输入凳首地址，x[]包含3个变量，x[0]潆新�
 //      x[1]输入的前一沃担瑇[2]输入的前两次值。
 //      二阶低通滤波器以前输出值数组首地址,其中y[0]表示输出的前一次值，
 //		y[1]表示输出的前两次值。
 //  出口：二阶低通滤波器输出值。
 //  中间变量：y_new用于暂时存储二阶低通滤波器的当前输出值。
 //  使用条件：滤波器的常数对应的采样频率为12.5KHz，截止频率fn = 950Hz;

 float y_new;
 y_new = ( FilterCnstA_1st_A * y[0] )
 - ( FilterCnstB_1st_A * y[1] )
 + ( FilterCnstC_1st_A * x[0] )
 + ( FilterCnstD_1st_A * x[1] )
 + ( FilterCnstE_1st_A * x[2] );

 y[1] = y[0];
 y[0] = y_new;
 x[2] = x[1];
 x[1] = x[0];
 return y_new;
 }
 */
/*
 void DA_OUT(void)
 {
 for(i=0;i<8;i++)
 {
 DA_date[i] = 4096-DA_date[i];           	//注意DA_date为浮点转换后的int值，且定标限制在12位
 DA_Val = DA_date[i];
 *(DA_register+i) = DA_date[i];    //注意没路输出的数据是否对应
 }
 //假鐶PIO59上缡蔽电平
 GpioDataRegs.GPBCLEAR.bit.GPIO59=1; //GPIO59/LD置低，输出数据；
 delay_t(12);                  		//保持1us
 GpioDataRegs.GPBSET.bit.GPIO59=1;	//GPIO59/LD置高
 }
 */

/*
 入口参数：三相电网电压、三相负载电流、三相APF反馈电流、零相APF反馈电流，一个周期采样点数
 AllSampleCount，一个周期采样点数的倒数RprocalValue，滤ㄆ鞯闹屑浔淞渴�如UAB_Fy和UAB_Fx等，
 各次滤波量的累加和，如UAB_Sum等。
 应用前提：
 中断程序中必须对一个周期连续采样才能计算零漂，否则不可以。中断程序中按如下迪郑�
 每次中断则：
 CountCmpar2++;
 if ( CountCmpar2 > (int)AllSampleCount )
 {
 CountCmpar2 -= ((int)AllSampleCount+1);
 }
 当停止稳压则：
 CountCmpar1++;
 if ( CountCmpar1 > (int)AllSampleCount)
 {
 CountCmpar1 -= ((int)AllSampleCount + 1);
 }
 如果CountCmpar1和CountCmpar2相等，则进行零漂计算，否则加和参数清零不进行零漂计算
 只要判断CountCmpar1和CountCmpar2不相等，则该子程序中将其都置0
 算法实现：
 1、连续加和一个周期要计算零漂的滤波参数，滤波器采用二阶低通滤波器，截止频率5Hz。加和一个周期后做平均
 出口参数：三相电网电压、三相负载电流、三相APF反馈电流，零郃FP反馈电流的零漂
 2、对于不能连续记录一个周期的值均舍弃，并重新开始记录，只有完全记录一个周期后在计算零漂。
 检验方法：通过CCS提供的作图软件画图观测自动零蟮氖导柿闫榭�

 */
/*void Compensate_shift()
 {
 if ( CountCmpar2 != CountCmpar1 )
 {
 UAB_Sum = 0;
 UBC_Sum = 0;
 UCA_Sum = 0;
 IA_Sum = 0;
 IB_Sum = 0;
 IC_Sum = 0;
 IAF_Sum = 0;
 IBF_Sum = 0;
 ICF_Sum = 0;
 I0F_Sum = 0;
 CountCmpar2 = 1;
 CountCmpar1 = 1;
 }

 if ( CountCmpar1 <= (int)AllSampleCount )
 {
 UAB_Fx[0] = UAADRF;
 UAB_Sum += LP_Filter_2nd( UAB_Fy, UAB_Fx );
 UBC_Fx[0] = UBADRF;
 UBC_Sum += LP_Filter_2nd( UBC_Fy, UBC_Fx );
 //UCA_Fx[0] = UCADRF;
 //UCA_Sum += LP_Filter_2nd( UCA_Fy, UCA_Fx );

 IA_Fx[0] = IAADRF;
 IA_Sum += LP_Filter_2nd( IA_Fy, IA_Fx );
 IB_Fx[0] = IBADRF;
 IB_Sum += LP_Filter_2nd( IB_Fy, IB_Fx );
 IC_Fx[0] = ICADRF;
 IC_Sum += LP_Filter_2nd( IC_Fy, IC_Fx );

 IAF_Fx[0] = IAAPF;
 IAF_Sum += LP_Filter_2nd( IAF_Fy, IAF_Fx );
 IBF_Fx[0] = IBAPF;
 IBF_Sum += LP_Filter_2nd( IBF_Fy, IBF_Fx );
 ICF_Fx[0] = ICAPF;
 ICF_Sum += LP_Filter_2nd( ICF_Fy, ICF_Fx );
 //I0F_Fx[0] = I0APF;
 //I0F_Sum += LP_Filter_2nd( I0F_Fy, I0F_Fx );
 }
 if ( CountCmpar1 == (int)AllSampleCount )
 {
 UA_Z = - ( RprocalValue * UAB_Sum );
 UB_Z = - ( RprocalValue * UBC_Sum );
 //UC_Z = RprocalValue * UCA_Sum;
 IA_Z = - ( RprocalValue * IA_Sum );
 IB_Z = - ( RprocalValue * IB_Sum );
 IC_Z = - RprocalValue * IB_Sum;
 IAF_Z = - ( RprocalValue * IAF_Sum );
 IBF_Z = - ( RprocalValue * IBF_Sum );
 ICF_Z = - ( RprocalValue * ICF_Sum );
 I0F_Z = - ( RprocalValue * I0F_Sum );

 UAB_Sum = 0;
 UBC_Sum = 0;
 UCA_Sum = 0;
 IA_Sum = 0;
 IB_Sum = 0;
 IC_Sum = 0;
 IAF_Sum = 0;
 IBF_Sum = 0;
 ICF_Sum = 0;
 I0F_Sum = 0;
 //I0F_Z = RprocalValue * I0F_Sum;
 }
 }*/

/******************************初始化***************************************/
//系统时钟及外设时钟初始化
void InitSysCtrl(void)
{

	// Disable the watchdog
	DisableDog();

	// Initialize the PLL control: PLLCR and DIVSEL
	// DSP28_PLLCR and DSP28_DIVSEL are defined in DSP2833x_Examples.h
	InitPll(DSP28_PLLCR, DSP28_DIVSEL);

	// Initialize the peripheral clocks
	InitPeripheralClocks();
}
void ServiceDog(void)
{
	EALLOW;
	SysCtrlRegs.WDKEY = 0x0055;
	SysCtrlRegs.WDKEY = 0x00AA;
	EDIS;
}
void DisableDog(void)
{
	EALLOW;
	SysCtrlRegs.WDCR = 0x0068;
	EDIS;
}
void InitPll(Uint16 val, Uint16 divsel)
{

	// Make sure the PLL is not running in limp mode
	if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
	{
		// Missing external clock has been detected
		// Replace this line with a call to an appropriate
		// SystemShutdown(); function.
		asm("        ESTOP0");
	}

	// DIVSEL MUST be 0 before PLLCR can be changed from
	// 0x0000. It is set to 0 by an external reset XRSn
	// This puts us in 1/4
	if (SysCtrlRegs.PLLSTS.bit.DIVSEL != 0)
	{
		EALLOW;
		SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
		EDIS;
	}

	// Change the PLLCR
	if (SysCtrlRegs.PLLCR.bit.DIV != val)
	{

		EALLOW;
		// Before setting PLLCR turn off missing clock detect logic
		SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
		SysCtrlRegs.PLLCR.bit.DIV = val;
		EDIS;

		// Optional: Wait for PLL to lock.
		// During this time the CPU will switch to OSCCLK/2 until
		// the PLL is stable.  Once the PLL is stable the CPU will
		// switch to the new PLL value.
		//
		// This time-to-lock is monitored by a PLL lock counter.
		//
		// Code is not required to sit and wait for the PLL to lock.
		// However, if the code does anything that is timing critical,
		// and requires the correct clock be locked, then it is best to
		// wait until this switching has completed.

		// Wait for the PLL lock bit to be set.

		// The watchdog should be disabled before this loop, or fed within
		// the loop via ServiceDog().

		// Uncomment to disable the watchdog
		DisableDog();

		while (SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1)
		{
			// Uncomment to service the watchdog
			// ServiceDog();
		}

		EALLOW;
		SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
		EDIS;
	}

	EALLOW;
	SysCtrlRegs.PLLSTS.bit.DIVSEL = divsel;
	EDIS;
}
void InitPeripheralClocks(void)
{
	EALLOW;

// HISPCP/LOSPCP prescale register settings, normally it will be set to default values
	SysCtrlRegs.HISPCP.all = 0x0001;
	SysCtrlRegs.LOSPCP.all = 0x0002;

// XCLKOUT to SYSCLKOUT ratio.  By default XCLKOUT = 1/4 SYSCLKOUT
	// XTIMCLK = SYSCLKOUT/2
//	XintfRegs.XINTCNF2.all=0x03;
	XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
	// XCLKOUT = XTIMCLK/2
	XintfRegs.XINTCNF2.bit.CLKMODE = 1;
	// Enable XCLKOUT
	XintfRegs.XINTCNF2.bit.CLKOFF = 0;

// Peripheral clock enables set for the selected peripherals.
// If you are not using a peripheral leave the clock off
// to save on power.
//
// Note: not all peripherals are available on all 2833x derivates.
// Refer to the datasheet for your particular device.
//
// This function is not written to be an example of efficient code.

	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;    // ADC

	// *IMPORTANT*
	// The ADC_cal function, which  copies the ADC calibration values from TI reserved
	// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
	// Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC to function according
	// to specification. The clocks to the ADC MUST be enabled before calling this
	// function.
	// See the device data manual and/or the ADC Reference
	// Manual for more information.

	ADC_cal();

//	SysCtrlRegs.PCLKCR0.all=0x0028;
//	SysCtrlRegs.PCLKCR3.all=0xFFFF;

	SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;   // I2C
	SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 0;   // SCI-A
	SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 0;   // SCI-B
	SysCtrlRegs.PCLKCR0.bit.SCICENCLK = 1;   // SCI-C
	SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 0;   // SPI-A
	SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = 1; // McBSP-A
	SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK = 1; // McBSP-B
	SysCtrlRegs.PCLKCR0.bit.ECANAENCLK = 0;    // eCAN-A
	SysCtrlRegs.PCLKCR0.bit.ECANBENCLK = 0;    // eCAN-B

	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Disable TBCLK within the ePWM
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // ePWM1
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // ePWM2
	SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // ePWM3
	SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 1;  // ePWM4
	SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 0;  // ePWM5
	SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 0;  // ePWM6
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Enable TBCLK within the ePWM

	SysCtrlRegs.PCLKCR1.bit.ECAP3ENCLK = 0;  // eCAP3
	SysCtrlRegs.PCLKCR1.bit.ECAP4ENCLK = 0;  // eCAP4
	SysCtrlRegs.PCLKCR1.bit.ECAP5ENCLK = 0;  // eCAP5
	SysCtrlRegs.PCLKCR1.bit.ECAP6ENCLK = 0;  // eCAP6
	SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 0;  // eCAP1
	SysCtrlRegs.PCLKCR1.bit.ECAP2ENCLK = 0;  // eCAP2
	SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 0;  // eQEP1
	SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK = 0;  // eQEP2

	SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1; // CPU Timer 0
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK = 1; // CPU Timer 1
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = 1; // CPU Timer 2

	SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 0;       // DMA Clock
	SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;     // XTIMCLK
	SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;    // GPIO input clock

	EDIS;
}

void InitGPIO(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;       //过流保护信号
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;       //过温保护信号
	GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;       //过压保护信号
	GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;       //IGBT故障保护信号
	GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;       //欠压保护信号
	GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;       //保护信号
	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;
	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;   // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	// Enable pull-up for GPIO63 (SCITXDC)
	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  	// Asynch input GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1; // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1; // Configure GPIO63 for SCITXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;   	//GPIO61用于485通信控制
	GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;   	//GPIO60用于AD开启控制
	GpioCtrlRegs.GPBPUD.bit.GPIO60 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;   	//GPIO24用于AD片选
	GpioCtrlRegs.GPAPUD.bit.GPIO24 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;   	//GPIO26用于诵时测试
	GpioCtrlRegs.GPAPUD.bit.GPIO26 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;   	//GPIO27用于运行
	GpioCtrlRegs.GPAPUD.bit.GPIO27 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;   //与F28335B交换数据时触发F28335B中断
	GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;   	//GPIO59用于DA开启控制
	GpioCtrlRegs.GPBPUD.bit.GPIO59 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;       //GPIO56用于输出DIS控制信号
	GpioCtrlRegs.GPBDIR.bit.GPIO56 = 1;
	GpioCtrlRegs.GPBPUD.bit.GPIO56 = 1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;       //GPIO57用于控制程序复位
	GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;
	GpioCtrlRegs.GPBPUD.bit.GPIO57 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;       //用于DRAM数据读取控制
	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;
	GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM3B)
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
	GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
	GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWM4A)
	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pull-up on GPIO7 (EPWM4B)
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B
	GpioCtrlRegs.GPBMUX1.all = 0xFFFFFF00; //配置地址线（XA0~XA7），地址线第16位XA16,XWE0使能和zone0、zone7 CS（即GPIO36和GPIO37为11B）
	GpioCtrlRegs.GPCMUX2.all = 0x0000FFFF;    //配置地址线（XA8~XA15）
	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;      //zone6 CS
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 3;	    //XA17
	GpioCtrlRegs.GPCMUX1.all = 0xFFFFFFFF;   //数据线涿，共16位数据线
	XintfRegs.XTIMING0.all = 0x00033468;       //zone0 25MHz   7尚未配置
	XintfRegs.XTIMING6.all = 0x00033468;       //zone6
	EDIS;
}

void InitSCIC(void)
{
	// Note: Clocks were turned on to the SCIC peripheral
	// in the InitSysCtrl() function

	ScicRegs.SCICCR.all = 0x0007;   	// 1 stop bit,  No loopback
										// No parity,8 char bits,
										// async mode, idle-line protocol
	ScicRegs.SCICTL1.all = 0x0003;  	// enable TX, RX, internal SCICLK,
										// Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all = 0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA = 0;
	ScicRegs.SCICTL2.bit.RXBKINTENA = 0;
	ScicRegs.SCIHBAUD = 0x0003;  	//3; //7
	ScicRegs.SCILBAUD = 0x00B6;  	//B6;      //5A;
	ScicRegs.SCICCR.bit.LOOPBKENA = 0;	// Disable loop back
	ScicRegs.SCICTL1.all = 0x0023;     	// Relinquish SCI from Reset
	ScicRegs.SCIFFRX.bit.RXFFIENA = 0;
}

void InitTIMER1(void)
{
	CpuTimer1.RegsAddr = &CpuTimer1Regs;
	// Initialize timer period to maximum:
	CpuTimer1Regs.PRD.all = 0xFFFFFFFF;

	// Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
	CpuTimer1Regs.TPR.all = 0;
	CpuTimer1Regs.TPRH.all = 0;

	// Make sure timers are stopped:
	CpuTimer1Regs.TCR.bit.TSS = 1;

	// Reload all counter register with period value:
	CpuTimer1Regs.TCR.bit.TRB = 1;

	// Reset interrupt counters:
	CpuTimer1.InterruptCount = 0;
}
void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period)
{
	Uint32 temp;

	// Initialize timer period:
	Timer->CPUFreqInMHz = Freq;
	Timer->PeriodInUSec = Period;
	temp = (long) (Freq * Period);
	Timer->RegsAddr->PRD.all = temp;

	// Set pre-scale counter to divide by 1 (SYSCLKOUT):
	Timer->RegsAddr->TPR.all = 0;
	Timer->RegsAddr->TPRH.all = 0;

	// Initialize timer control register:
	Timer->RegsAddr->TCR.bit.TSS = 1; // 1 = Stop timer, 0 = Start/Restart Timer
	Timer->RegsAddr->TCR.bit.TRB = 1;      // 1 = reload timer
	Timer->RegsAddr->TCR.bit.SOFT = 0;
	Timer->RegsAddr->TCR.bit.FREE = 0;     // Timer Free Run Disabled
	Timer->RegsAddr->TCR.bit.TIE = 1; // 0 = Disable/ 1 = Enable Timer Interrupt

	// Reset interrupt counter:
	Timer->InterruptCount = 0;
}
void InitEPWM(void)
{
	//=====================================================================
	// Configuration
	//=====================================================================
	// Initialization Time
	//========================// EPWM Module 1 config
	EPwm1Regs.TBPRD = 6000;	//5000;//8000; // Period = 12000 TBCLK counts  采样周期：6000*2/150 = 80us
	EPwm1Regs.TBPHS.all = 0; // Set Phase register to zero
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Master module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module???????
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;
	//TBCTL的HSPCLKDIV、CLKDIV两位可以设置TBCLk，但是此处没有设置而是使用其默认值
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; //CC_CTR_ZERO_PRD;//CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD; //CC_CTR_ZERO_PRD;//CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM1A
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR; //增比较匹配置高减比较匹配置低
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
	EPwm1Regs.DBFED = 600; //850; // FED = 600 TBCLKs  死区时间：600/150 = 4us
	EPwm1Regs.DBRED = 600; //850; // RED = 600 TBCLKs  死区时间：600/150 = 4us
	// EPWM Module 2 config
	EPwm2Regs.TBPRD = 6000; //5000;// Period = 15000 TBCLK counts
	EPwm2Regs.TBPHS.all = 0; // Set Phase register to zero
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm2Regs.TBCTL.bit.CLKDIV = 0;
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; //CC_CTR_ZERO_PRD;//CC_CTR_ZERO; // load on CTR=Zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD; //CC_CTR_ZERO_PRD;//CC_CTR_ZERO; // load on CTR=Zero CC_CTR_ZERO_PRD
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM2A
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
	EPwm2Regs.DBFED = 600; // FED = 600 TBCLKs
	EPwm2Regs.DBRED = 600; // RED = 600 TBCLKs
	// EPWM Module 3 config
	EPwm3Regs.TBPRD = 6000; //5000;// Period = 15000 TBCLK counts
	EPwm3Regs.TBPHS.all = 0; // Set Phase register to zero
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm3Regs.TBCTL.bit.CLKDIV = 0;
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; //CC_CTR_ZERO_PRD;//CC_CTR_ZERO; // load on CTR=Zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD; //CC_CTR_ZERO_PRD;//CC_CTR_ZERO; // load on CTR=Zero
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM3A
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
	EPwm3Regs.DBFED = 600; // FED = 600 TBCLKs
	EPwm3Regs.DBRED = 600; // RED = 600 TBCLKs
	// EPWM Module 4 config
	EPwm4Regs.TBPRD = 6000;		//7500 // Period = 1600 TBCLK counts
	EPwm4Regs.TBPHS.all = 0; // Set Phase register to zero
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm4Regs.TBCTL.bit.CLKDIV = 0;
	EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD; // load on CTR=Zero
	EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD; // load on CTR=Zero
	EPwm4Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM3A
	EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
	EPwm4Regs.DBFED = 600; //680 // FED = 50 TBCLKs
	EPwm4Regs.DBRED = 600; //680 // RED = 50 TBCLKs

	// Run Time (Note: Example execution of one run-time instant)
	//=========================================================
	//	EPwm1Regs.CMPA.half.CMPA = 50; // adjust duty for output EPWM1A
	//	EPwm2Regs.CMPA.half.CMPA = 2000; // adjust duty for output EPWM2A
	//	EPwm3Regs.CMPA.half.CMPA = 7800; // adjust duty for output EPWM3A

}

void InitFlash(void)
{
	EALLOW;
	//Enable Flash Pipeline mode to improve performance
	//of code executed from Flash.
	FlashRegs.FOPT.bit.ENPIPE = 1;

	//                CAUTION
	//Minimum waitstates required for the flash operating
	//at a given CPU rate must be characterized by TI.
	//Refer to the datasheet for the latest information.
	//Set the Paged Waitstate for the Flash
	FlashRegs.FBANKWAIT.bit.PAGEWAIT = 5;

	//Set the Random Waitstate for the Flash
	FlashRegs.FBANKWAIT.bit.RANDWAIT = 5;

	//Set the Waitstate for the OTP
	FlashRegs.FOTPWAIT.bit.OTPWAIT = 8;

	//                CAUTION
	//ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED
	FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;
	FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;
	EDIS;

	//Force a pipeline flush to ensure that the write to
	//the last register configured occurs before returning.

	delay_t(2);
}
void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
	while (SourceAddr < SourceEndAddr)
	{
		*DestAddr++ = *SourceAddr++;
	}
	return;
}

//*******************************eeprom****************************8

void RD_EEPROM(unsigned Byte_Sum, unsigned Offet_Addr, unsigned Page_Addr)
{
	unsigned i = 0;
	unsigned I2C_com = 0;
	loop1: I2caRegs.I2CSAR = 0x50 + Page_Addr; // EEPROM某一页地址（由命令及设备号1010A2和页地址P1P0组成）
	//主动建立读某一数据帧的操作地址
	I2caRegs.I2CCNT = 1;
	I2caRegs.I2CDXR = Offet_Addr;	//EEPROM某一小页的首地址
	I2caRegs.I2CMDR.all = 0x6E20;
	//在启动发送后一定要有一个延时
	//保持时间最少0.6us(2.7~5V供�外悠渌渌邮贝笥谝时2us左右,见AT24C08表5），
	//否则不能正建立读操鳎庖坏惴浅Ｖ匾�
//	for(i=0;i<48;i++){;}//建立读操作时也需要几个微妙建立读操档刂�
	I2C_com = 0;
	while (I2caRegs.I2CMDR.bit.STP == 1)	//程序跑死之后的处砘�
	{
		I2C_com++;
		for (i = 0; i < 10; i++)
		{
			;
		}
		if (I2C_com > 10000)
		{
			I2C_com = 0;
			goto loop1;
		}
	}
	I2caRegs.I2CMDR.bit.STP = 1; //清总线忙状态
	I2caRegs.I2CCNT = Byte_Sum;

	I2caRegs.I2CMDR.all = 0x6C20;
	I2C_com = 0;
	while (I2caRegs.I2CMDR.bit.STP == 1) //程序跑死之后的处理机制
	{
		I2C_com++;
		for (i = 0; i < 10; i++)
		{
			;
		}
		if (I2C_com > 10000)
		{
			I2C_com = 0;
			goto loop1;
		}
	}
	I2caRegs.I2CMDR.bit.STP = 1; //清总线忙状态
//	for(i=0;i<150;i++){;}
	for (i = 0; i < Byte_Sum; i++)
	{
		MsgBuffer_Receive[i] = I2caRegs.I2CDRR;
	}

}

void WR_EEPROM(unsigned Byte_Sum, unsigned Offet_Addr, unsigned Page_Addr)
{

	unsigned int i = 0;
	unsigned I2C_com = 0;
	loop2: I2caRegs.I2CSAR = 0x50 + Page_Addr; // EEPROM某一页地址（由命令及设备�010A2和页地址P1P0组成）
	I2caRegs.I2CCNT = Byte_Sum + 1; // 发送的字节鼴yte_Sum + 2个字节地址（实际有效字诿看巫疃喾⑺�4个字节）
	I2caRegs.I2CDXR = Offet_Addr; //EEPROM页内地址首刂�
	//依次向I2CDXR中入⑺的字节数（FIFO）
	for (i = 0; i < Byte_Sum; i++)
	{
		I2caRegs.I2CDXR = MsgBuffer_Send[i];
	}
	//启动发送(自由方式不受断点影响；置开始模式；置停止模式；主动模式；发送；I2C模槭鼓�
	I2caRegs.I2CMDR.all = 0x6E20;
//	for(i=0;i<150;i++){;}
	I2C_com = 0;
	while (I2caRegs.I2CMDR.bit.STP == 1)	//程序跑乐荡砘�
	{
		I2C_com++;
		for (i = 0; i < 10; i++)
		{
			;
		}
		if (I2C_com > 10000)
		{
			I2C_com = 0;
			goto loop2;
		}
	}
	I2caRegs.I2CMDR.bit.STP = 1; //清总线忙状态
//	I2caRegs.I2CMDR.bit.IRS = 0;//复位
	for (i = 0; i < 30000; i++)
	{
		;
	} //AT24C08每次写完必须等待毫秒级才能再次建立起地址，进性尾禀。
}

void Restore_Factory_Set()
{
	//暂时不考虑自动重启功能
	unsigned i = 0;
	MsgBuffer_Send[i++] = 0x55AA; //EEPROM非空校验字
	MsgBuffer_Send[i++] = Version_No; //程序版本号，每个程序在Flash中有该程序的版本号
	WR_DATA_PROCESS(2); //处理2鲎�处理之后变为4个字节,最多处理7个字
	//向EEPROM第0x0页页内偏移地址为0x00的空间中写入4个字节
	WR_EEPROM(4, 0x00, 0x4);	//最多同时发送15个字节

	i = 0;
	MsgBuffer_Send[i++] = UDCREF_initial;
	MsgBuffer_Send[i++] = UDC_Max_initial;
	MsgBuffer_Send[i++] = Ih_Max_initial;
	MsgBuffer_Send[i++] = C_way_initial;
	MsgBuffer_Send[i++] = UDC_Kp_initial;
	MsgBuffer_Send[i++] = UDC_Ki_initial;
	MsgBuffer_Send[i++] = U_FtoI_initial;
	WR_DATA_PROCESS(7);
	WR_EEPROM(14, 0x10, 0x4);

	i = 0;
	MsgBuffer_Send[i++] = UABC_initial;
	WR_DATA_PROCESS(1);
	WR_EEPROM(2, 0x1E, 0x4);

	i = 0;
	MsgBuffer_Send[i++] = UABC_initial;
	MsgBuffer_Send[i++] = UABC_initial;
	WR_DATA_PROCESS(2);
	WR_EEPROM(4, 0x20, 0x4);

	/*	i = 0;
	 MsgBuffer_Send[i++]=C_Way1_initial;
	 WR_DATA_PROCESS(1);
	 WR_EEPROM(2,0x1E,0x4);

	 i = 0;
	 MsgBuffer_Send[i++]=C_Way2_initial;
	 WR_DATA_PROCESS(1);
	 WR_EEPROM(2,0x2E,0x4);

	 i = 0;
	 MsgBuffer_Send[i++]=C_Way3_initial;
	 WR_DATA_PROCESS(1);
	 WR_EEPROM(2,0x3E,0x4);
	 */

	//出厂设置参数，可以一次写入EEPROM的指定位置，需要时从EEPROM中读出，不对用户公开
	//调试阶段公开相关参数
	UDCREF = UDCREF_initial;
	UDC_Max = UDC_Max_initial;
	Ih_Max = Ih_Max_initial;
	C_way = C_way_initial;
	/*	C_Way1=C_Way1_initial;
	 C_Way2=C_Way2_initial;
	 C_Way3=C_Way3_initial;*/
	UDC_Kp = UDC_Kp_initial;
	UDC_Ki = UDC_Ki_initial;
	U_FtoI = U_FtoI_initial;
	UDC_Rf2 = Udc_slip = UABC_initial;
	//UDC_Rf1=UDC_Rf2=Udc_slip=UABC_initial;
	COM_Ua = 550;

}

void RD_DATA_PROCESS(unsigned Byte_Sum)
{
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int High_byte;
	unsigned int Low_byte;
	unsigned int Data_Buffer[16] =
	{ 0 };

	for (i = 0; i < (Byte_Sum * 2); i++)
	{
		j++;
		Low_byte = MsgBuffer_Receive[i];
		High_byte = MsgBuffer_Receive[++i];
		High_byte = High_byte << 8;
		Data_Buffer[i - j] = High_byte | Low_byte;
	}
	//两个连续字节合并成一个
	for (i = 0; i < Byte_Sum / 2; i++)
	{
		MsgBuffer_Receive[i] = Data_Buffer[i];
	}
}

void WR_DATA_PROCESS(unsigned Word_Sum)
{
	unsigned int HIGH_RESVD = 0xff00;
	unsigned int LOW_RESVD = 0x00FF;
	unsigned int High_byte;
	unsigned int Low_byte;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int Data_Process_Buffer = 0;
	unsigned int Dada_Buffer[15] =
	{ 0 };
	for (i = 0; i < Word_Sum; i++)
	{
		Dada_Buffer[i] = MsgBuffer_Send[i];
	}
	//一个字拆成2个字节重新装入MsgBuffer_Send[]
	for (i = 0; i < Word_Sum * 2; i++)
	{
		Data_Process_Buffer = Dada_Buffer[i - j];
		Low_byte = (Data_Process_Buffer & LOW_RESVD);
		MsgBuffer_Send[i] = Low_byte;
		High_byte = (Data_Process_Buffer & HIGH_RESVD);
		High_byte = High_byte >> 8;
		MsgBuffer_Send[++i] = High_byte;
		j++;	//注意j的使用，为的是在处理Data_Buffer[]时不会遗漏数据
	}
}
void InitI2C(void)
{
	// Initialize I2C-A:
	DINT;
	//I2C写操作初始化
	I2caRegs.I2CPSC.all = 14; // Prescaler - need 7-12 Mhz on module clk (150/15 = 10MHz)
	I2caRegs.I2CCLKL = 10;			// LOW-time duration of SCL:1.5us
	I2caRegs.I2CCLKH = 5;			// HIGH-time duration of SCL: 1us
	I2caRegs.I2CIER.all = 0x00;		// 不使用中断
	I2caRegs.I2CMDR.all = 0x0020;	// I2C模块使能
									// Stop I2C when suspende
	// I2caRegs.I2CFFTX.all = 0x6000;
	I2caRegs.I2CFFTX.all = 0x6040;	// Enable FIFO mode and TXFIFO
	I2caRegs.I2CFFRX.all = 0x2040;	// Enable RXFIFO, clear RXFFINT,
	EINT;
}
void InitI2CGpio()
{

	EALLOW;
	/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // Enable pull-up for GPIO32 (SDAA)
	GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;	   // Enable pull-up for GPIO33 (SCLA)

	/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SDAA)
	GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;  // Asynch input GPIO33 (SCLA)

	/* Configure SCI pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be I2C functional pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;  // Configure GPIO32 for SDAA operation
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;  // Configure GPIO33 for SCLA operation

	EDIS;
}
float LP_Filter_2nd(float *y, float *x)
{
	/*
	 入口：二阶低通滤波器输入值数组首地址，x[]包含3个变量，x[0]输入新值
	 x[1]输入的前一次值，x[2]输入的前两次值。
	 二阶低通滤波器以前输出值数组首地址,其中y[0]表示输出的前一次值，
	 y[1]表示输出的前两次值。
	 出口：二阶低通滤波器输出值。
	 中间变量：y_new用于暂时存储二阶低通滤波器的当前输出值。
	 使用条件：滤波器的常数对应的采样频率为9.375KHz，截止频率700Hz。离散化方法为双线性变化法。
	 */
	float y_new;
	y_new = (F_2nd_Cnst1 * y[0]) - (F_2nd_Cnst2 * y[1])
			+ (F_2nd_Cnst3 * (x[0] + x[2] + (2 * x[1])));

	y[1] = y[0];
	y[0] = y_new;
	x[2] = x[1];
	x[1] = x[0];
	return y_new;
}

//****************************end***************************************	

//===========================================================================
// No more.
//===========================================================================

