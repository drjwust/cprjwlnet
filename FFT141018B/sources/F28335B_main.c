
//现在需要做的工作: 1将程序重新读一遍验证正确性，2写FLASH的CMD文件及程序调用文件3外设时钟重新配置080729
#include "APF_Variable.h"
#include "APF_Constant.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h" 
#include "math.h"
//初始化配置
#define DSP28_PLLCR    10
#define DSP28_DIVSEL   2
#define ADC_Cal_Offset_A    1539//1510//1510//1700  //依靠这个量将单极性数据转换成双极性
#define ADC_Cal_Offset_B    1556//1500//1690
#define ADC_Cal_Offset_C    1539//1500//1690


//中心频率50Hz,通带宽度40Hz带通滤波器，采样频率12.5K，双线性变换方式,一阶
#define BP_1st_Cnst1 1.979467926//1.904
#define BP_1st_Cnst2 0.980093261//0.904
#define BP_1st_Cnst3 0.009953370

//基准校正源的理想输出
#define AdcRefHighIdealValue      3400//3138
#define AdcRefLowIdealValue       2683//2772//2634

//截止频率2HZ,采样频率9.375kHz，离散化方法为双线性变换法的一阶低通滤波器常数
#define F_1st_Cnst1   0.9987 
#define F_1st_Cnst2  0.0006698

//截止频率700Hz,采样频率9.375kHz，离散化方法为双线性变换法的二阶低通滤波器常数
#define F_2nd_Cnst1  1.352 
#define F_2nd_Cnst2  0.5155 
#define F_2nd_Cnst3  0.04094 

//中心频率50Hz,通带宽度20Hz带通滤波器，采样频率12.5K，双线性变换方式。
#define BP_2nd_Cnst1 3.985
#define BP_2nd_Cnst2 5.955
#define BP_2nd_Cnst3 3.956
#define BP_2nd_Cnst4 0.986
#define BP_2nd_Cnst5 0.00002509


#define DRAM_data  (int *) 0x0200000

void InitFlash(void);           	//FLASH使能
#pragma CODE_SECTION(InitFlash, "ramfuncs");
#pragma CODE_SECTION(Display_main, "ramfuncs");
#pragma CODE_SECTION(FFT, "ramfuncs");
#pragma CODE_SECTION(delay_t, "ramfuncs");
#pragma CODE_SECTION(DATA_CHANGE, "ramfuncs");
#pragma CODE_SECTION(LP_Filter_1st, "ramfuncs");
#pragma CODE_SECTION(LP_Filter_2nd, "ramfuncs");
#pragma CODE_SECTION(BP_Filter_2nd, "ramfuncs");
#pragma CODE_SECTION(BP_Filter_1st, "ramfuncs");
#pragma CODE_SECTION(Sin_Value, "ramfuncs");
#pragma CODE_SECTION(Check_Harmonic_N1, "ramfuncs");
// These are defined by the linker (see F28335.cmd)
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart; 
void MemCopy(Uint16 *SourceAddr, Uint16 *SourceEndAddr, Uint16 *DestAddr);

void InitSysCtrl(void);     //系统时钟及外设时钟初始化
void ServiceDog(void);      //开门狗使能
void DisableDog(void);      //看门狗禁止
void InitPll(Uint16, Uint16);  		//PLL初始化 时钟配置为30*10/2=150M
void InitPeripheralClocks(void);  	//外设时钟使能
void InitFlash(void);           	//FLASH使能
// These are defined by the linker (see F28335.cmd)
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart; 
void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);


interrupt void DATA_CHANGE(void);
void Display_main(void);               //主显示程序 2
float LP_Filter_2nd(float *, float *); //二阶低通滤波器
float LP_Filter_1st(float *, float *);//一阶低通滤波器
float BP_Filter_2nd(float *, float *);//二阶带通滤波器
float BP_Filter_1st(float *, float *);//一阶带通滤波器
void FFT(float dataR[],float dataI[],float w[],float I_d[],float I_q[]);  //FFT谐波计算
void Sin_Value( float theta );
void Check_Harmonic_N1(int H_n);
//通用函数
int m=0;        //
int n=0;    	//在delay_t中有使用
int i=0;
void delay_t(unsigned); 	 //100大约是8个us/频率150M
int date_flag=0;	


float r;

void main(void)
{
//	Uint16 SendChar;
	
    DINT;

#ifdef FLASH
	InitSysCtrl();  
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
#endif

	InitFlash();
	InitPieVectTable();      //PIE 已经ENABLE
    EnableInterrupts();      //PIE直接采用例程中的文件 ENable the int12.xint7
	// Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
   	EALLOW;  
   	PieVectTable.XINT1=&DATA_CHANGE;
   	EDIS;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1; // Enable PIE Gropu 1 INT4 
    IER=M_INT1;    // Enable CPU int1
    //配置GPIO30为XINT1的中断源
	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO30=0;   	
    GpioCtrlRegs.GPAPUD.bit.GPIO30=1;
	GpioCtrlRegs.GPADIR.bit.GPIO30=0;
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL=0x1E; //选择GPIO30作为XINT1的中断输入
	EDIS;
	XIntruptRegs.XINT1CR.bit.POLARITY=0;       //下降沿产生中断
	XIntruptRegs.XINT1CR.bit.ENABLE=1;         //XINT1使能

	

    // Specific clock setting for this example:
    EALLOW;
    SysCtrlRegs.HISPCP.all = 0x3; // HSPCLK = SYSCLKOUT/(2*3)=25MHz
    EDIS;
	InitAdc();         // init the ADC
	// Specific ADC setup for this example:
   AdcRegs.ADCTRL1.bit.ACQ_PS = 0xF;//0x1 //ACQ_PS用于设置采样窗时间(ACQ_PS+1)*ADC clock in ns
					    //  Simultaneous mode enabled: Sample rate = 1/[(3+ACQ_PS)*ADC clock in ns]
						//                                           =1/(4*40ns)=6.25MHz
						//If Sequential mode: Sample rate   = 1/[(2+ACQ_PS)*ADC clock in ns]
                        //                     = 1/(3*40ns) =8.3MHz (for 150 MHz SYSCLKOUT)
   AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x5;//0x0;
   AdcRegs.ADCTRL1.bit.CPS = 1;//0;
   AdcRegs.ADCTRL3.bit.SMODE_SEL = 0x1; // Setup simultaneous sampling mode
   AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        //  Cascaded mode
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // Setup conv from ADCINA0 &amp; ADCINB0
   AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // Setup conv from ADCINA1 &amp; ADCINB1
   AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; // Setup conv from ADCINA2 &amp; ADCINB2
   AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3; // Setup conv from ADCINA3 &amp; ADCINB3
   AdcRegs.ADCMAXCONV.all = 0x0003; // 4 double conv's (8 total)
   AdcRegs.ADCTRL1.bit.CONT_RUN = 1;       //每次启动AD转换只有当最大转换通道转换完成才置标志位
   AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1;       // Sequencer override feature，转换完成之后通道指针重新开始
   //AdcRegs.ADCOFFTRIM.all = 0x004A;  //AD校正



	EALLOW;
	GpioCtrlRegs.GPBMUX2.bit.GPIO56=0;       //GPIO56用于输出DIS控制信号
	GpioCtrlRegs.GPBDIR.bit.GPIO56=1;
	GpioCtrlRegs.GPBPUD.bit.GPIO56=1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO57=0;       //GPIO56用于输出DIS控制信号
	GpioCtrlRegs.GPBDIR.bit.GPIO57=1;
	GpioCtrlRegs.GPBPUD.bit.GPIO57=1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO55=0;       //用于测试用
	GpioCtrlRegs.GPBDIR.bit.GPIO55=1;
	GpioCtrlRegs.GPBPUD.bit.GPIO55=1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO49=0;       
	GpioCtrlRegs.GPBDIR.bit.GPIO49=1;
	GpioCtrlRegs.GPBPUD.bit.GPIO49=1;
    GpioCtrlRegs.GPBMUX1.all=0xFFFFFF00;   	//配置地址线（XA0~XA7），地址线第16位XA16,XWE0使能和zone0、zone7 CS（即GPIO36和GPIO37为11B）
	GpioCtrlRegs.GPCMUX2.all=0x0000FFFF;    //配置地址线（XA8~XA15）
	GpioCtrlRegs.GPAMUX2.bit.GPIO28=3;      //zone6 CS
    GpioCtrlRegs.GPAMUX2.bit.GPIO31=3;	    //XA17
	GpioCtrlRegs.GPCMUX1.all=0xFFFFFFFF;   //数据线配置，共16位数据线
	EDIS;

	EALLOW;
	XintfRegs.XTIMING7.all=0x0031224;
	EDIS;
    GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1; 


//	InitSCIC(); // Initalize SCI 

  	EINT;
	IFR = 0x0000;
	ERROR_flag=0;

	Display_main();
}
void VoltageCalibration(void)
{
	static float Ua_z = 17, Ub_z = 25, Uc_z = 9.5; //电网电压的零漂
	static float Ua_k = -0.2926 * 1.414, Ub_k = -0.2917 * 1.414, Uc_k = -0.294 * 1.414; //电网电压的增益
//	static float Ua_k = 0.2926 * 1.414, Ub_k = 0.2917 * 1.414, Uc_k = 0.294 * 1.414; //电网电压的增益	TODO 反电压

	/*电网电压定标*/
	UAADRF += Ua_z;
	UBADRF += Ub_z;
	UCADRF += Uc_z;

	UAADRF *= Ua_k;
	UBADRF *= Ub_k;
	UCADRF *= Uc_k;
}
/*****************************控制相关*********************************/
//主要的控制函数，包括采样、谐波检测电压控制等等
interrupt void DATA_CHANGE(void)          
{
   float Omiga=0, omiga_delta=0;
   float invented_power=0;                 //PLL用到
	float ia=0, ic=0;
        DINT;
		GpioDataRegs.GPBSET.bit.GPIO56=1;  //测试用
		
        // Start SEQ1
       AdcRegs.ADCTRL2.all = 0x2000;
       // Wait for int1
       while (AdcRegs.ADCST.bit.INT_SEQ1== 0){}
       AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;

       UAADRF= ( (AdcRegs.ADCRESULT0)>>4);
	   UAADRF-= ADC_Cal_Offset_A;
       UBADRF= ( (AdcRegs.ADCRESULT1)>>4);
	   UBADRF-= ADC_Cal_Offset_B;
       UCADRF= ( (AdcRegs.ADCRESULT2)>>4);
	   UCADRF-= ADC_Cal_Offset_C;
       TEMP1= ( (AdcRegs.ADCRESULT3)>>4);
       AdcRefHighActualValue= ( (AdcRegs.ADCRESULT4)>>4);
       TEMP2= ( (AdcRegs.ADCRESULT5)>>4);
       RESERVE1= ( (AdcRegs.ADCRESULT6)>>4);
       AdcRefLowActualValue= ( (AdcRegs.ADCRESULT7)>>4);
       AdcRegs.ADCTRL2.all = 0x0000;// Stop SEQ1
       

/****************软件锁相环*******************************/
       VoltageCalibration();

    Sin_Value(Omiga_t);
	ia = sinVal;	
	Sin_Value(Omiga_t + 120);
	ic = sinVal;
	invented_power = -ia * UAADRF + ic * UBADRF;
	invented_integral = invented_integral + invented_power * PLL_TS ;

	if(invented_integral > 1000.0)
		invented_integral = 1000.0;
	if(invented_integral < (-1000.0))
		invented_integral = -1000.0;

	
	Omiga = 90 * invented_power + 220 * invented_integral ;

	omiga_delta = Omiga* PLL_TS;

	
	Omiga_t = Omiga_t + omiga_delta;
	while(Omiga_t < 0)	Omiga_t += 360;
    while(Omiga_t >= 360) Omiga_t -= 360;
   
    THETA_C= Omiga_t + 18.2;// + 18.2;
	while(THETA_C < 0)	THETA_C += 360;
    while(THETA_C >= 360) THETA_C -= 360;

	/************************计算部分谐波************************************/
	if(data_flag5==1) 
	{
	     IA_d_toA[21] = IA_d[21];
         IA_q_toA[21] = IA_q[21];
         IB_d_toA[21] = IB_d[21];
         IB_q_toA[21] = IB_q[21];
         IC_d_toA[21] = IC_d[21];
         IC_q_toA[21] = IC_q[21];
		 
		 IA_d_toA[25] = IA_d[25];
         IA_q_toA[25] = IA_q[25];
         IB_d_toA[25] = IB_d[25];
         IB_q_toA[25] = IB_q[25];
         IC_d_toA[25] = IC_d[25];
         IC_q_toA[25] = IC_q[25];
	}
	  Check_Harmonic_N1(21);
		  IAh_Part_Sum = IAh;
		  IBh_Part_Sum = IBh;
		  ICh_Part_Sum = ICh;
 
	  Check_Harmonic_N1(25);
		  IAh_Part_Sum += IAh*0.5;
		  IBh_Part_Sum += IBh*0.5;
		  ICh_Part_Sum += ICh*0.5;
   /*********************************************************************/
        s=10;
		w=100;

		*( DRAM_data + 3 ) = (int)UAADRF; //电压与相角传输
		*( DRAM_data + 4 ) = (int)UBADRF;
		*( DRAM_data + 5 ) = (int)UCADRF;
		*( DRAM_data + 150 ) = (int)(THETA_C*50);
		*( DRAM_data + 151 ) = (int)(IAh_Part_Sum*100); //传输由B片计算得谐波
		*( DRAM_data + 152 ) = (int)(IBh_Part_Sum*100);
		*( DRAM_data + 153 ) = (int)(ICh_Part_Sum*100);


	/***************************************************************/

	
    
    if(data_flag5==1)
    {   
	   	*(DRAM_data+104) = 1;
	    *(DRAM_data+26) =  (int)(IA_d[1] * s);
		*(DRAM_data+27) =  (int)(IA_q[1] * s);
		*(DRAM_data+28) =  (int)(IB_d[1] * s);
		*(DRAM_data+29) =  (int)(IB_q[1] * s);
		*(DRAM_data+30) =  (int)(IC_d[1] * s);
		*(DRAM_data+31) =  (int)(IC_q[1] * s);

		*(DRAM_data+32) = (int)(IA_d[3] * s);	
		*(DRAM_data+33) = (int)(IA_q[3] * s);
		*(DRAM_data+34) = (int)(IB_d[3] * s);
		*(DRAM_data+35) = (int)(IB_q[3] * s);
		*(DRAM_data+36) = (int)(IC_d[3] * s);
		*(DRAM_data+37) = (int)(IC_q[3] * s);

		*(DRAM_data+38) = (int)(IA_d[5] * s);
		*(DRAM_data+39) = (int)(IA_q[5] * s);
		*(DRAM_data+40) = (int)(IB_d[5] * s);
		*(DRAM_data+41) = (int)(IB_q[5] * s);
		*(DRAM_data+42) = (int)(IC_d[5] * s);
		*(DRAM_data+43) = (int)(IC_q[5] * s);

		*(DRAM_data+44) = (int)(IA_d[7] * s);
		*(DRAM_data+45) = (int)(IA_q[7] * s);
		*(DRAM_data+46) = (int)(IB_d[7] * s);
		*(DRAM_data+47) = (int)(IB_q[7] * s);
		*(DRAM_data+48) = (int)(IC_d[7] * s);
		*(DRAM_data+49) = (int)(IC_q[7] * s);

		*(DRAM_data+50) = (int)(IA_d[9] * w);
		*(DRAM_data+51) = (int)(IA_q[9] * w);
		*(DRAM_data+52) = (int)(IB_d[9] * w);
		*(DRAM_data+53) = (int)(IB_q[9] * w);
		*(DRAM_data+54) = (int)(IC_d[9] * w);
		*(DRAM_data+55) = (int)(IC_q[9] * w);

		*(DRAM_data+56) = (int)(IA_d[11] * w);
        *(DRAM_data+57) = (int)(IA_q[11] * w);
		*(DRAM_data+58) = (int)(IB_d[11] * w);
		*(DRAM_data+59) = (int)(IB_q[11] * w);
		*(DRAM_data+60) = (int)(IC_d[11] * w);
		*(DRAM_data+61) = (int)(IC_q[11] * w);

		*(DRAM_data+62) = (int)(IA_d[13] * w);
		*(DRAM_data+63) = (int)(IA_q[13] * w);
		*(DRAM_data+64) = (int)(IB_d[13] * w);
		*(DRAM_data+65) = (int)(IB_q[13] * w);
		*(DRAM_data+66) = (int)(IC_d[13] * w);
		*(DRAM_data+67) = (int)(IC_q[13] * w);

		*(DRAM_data+68) = (int)(IA_d[15] * w);
		*(DRAM_data+69) = (int)(IA_q[15] * w);
		*(DRAM_data+70) = (int)(IB_d[15] * w);
		*(DRAM_data+71) = (int)(IB_q[15] * w);
		*(DRAM_data+72) = (int)(IC_d[15] * w);
		*(DRAM_data+73) = (int)(IC_q[15] * w);

		*(DRAM_data+74) = (int)(IA_d[17] * w);
		*(DRAM_data+75) = (int)(IA_q[17] * w);
		*(DRAM_data+76) = (int)(IB_d[17] * w);
		*(DRAM_data+77) = (int)(IB_q[17] * w);
		*(DRAM_data+78) = (int)(IC_d[17] * w);
		*(DRAM_data+79) = (int)(IC_q[17] * w);

		*(DRAM_data+80) = (int)(IA_d[19] * w);
		*(DRAM_data+81) = (int)(IA_q[19] * w);
		*(DRAM_data+82) = (int)(IB_d[19] * w);
		*(DRAM_data+83) = (int)(IB_q[19] * w);
		*(DRAM_data+84) = (int)(IC_d[19] * w);
		*(DRAM_data+85) = (int)(IC_q[19] * w);

	 //	*(DRAM_data+86) = (int)(IA_d[21] * w);
	//	*(DRAM_data+87) = (int)(IA_q[21] * w);
	//	*(DRAM_data+88) = (int)(IB_d[21] * w);
	//	*(DRAM_data+89) = (int)(IB_q[21] * w);
	//	*(DRAM_data+90) = (int)(IC_d[21] * w);
	//	*(DRAM_data+91) = (int)(IC_q[21] * w);
        
		*(DRAM_data+92) = (int)(IA_d[23] * w);
		*(DRAM_data+93) = (int)(IA_q[23] * w);
		*(DRAM_data+94) = (int)(IB_d[23] * w);
		*(DRAM_data+95) = (int)(IB_q[23] * w);
		*(DRAM_data+96) = (int)(IC_d[23] * w);
		*(DRAM_data+97) = (int)(IC_q[23] * w);

		//*(DRAM_data+98) = (int)(IA_d[25] * w);
	//	*(DRAM_data+99) = (int)(IA_q[25] * w);
	//	*(DRAM_data+100) = (int)(IB_d[25] * w);
	//	*(DRAM_data+101) = (int)(IB_q[25] * w);
	//	*(DRAM_data+102) = (int)(IC_d[25] * w);
	//	*(DRAM_data+103) = (int)(IC_q[25] * w);
	
		*(DRAM_data+105) = (int)(IA3 * s);
		*(DRAM_data+106) = (int)(IA5 * s);
		*(DRAM_data+107) = (int)(IA7 * s);
//		*(DRAM_data+108) = (int)(IA9 * w);
//		*(DRAM_data+109) = (int)(IA11 * w);
//		*(DRAM_data+110) = (int)(IA13 * w);
//		*(DRAM_data+111) = (int)(IA15 * w);
//		*(DRAM_data+112) = (int)(IA17_25 * w);
	
		*(DRAM_data+113) = (int)(IB3 * s);
		*(DRAM_data+114) = (int)(IB5 * s);
		*(DRAM_data+115) = (int)(IB7 * s);
//		*(DRAM_data+116) = (int)(IB9 * w);
//		*(DRAM_data+117) = (int)(IB11 * w);
//		*(DRAM_data+118) = (int)(IB13 * w);
//		*(DRAM_data+119) = (int)(IB15 * w);
//		*(DRAM_data+120) = (int)(IB17_25 * w);

		*(DRAM_data+121) = (int)(IC3 * s);
		*(DRAM_data+122) = (int)(IC5 * s);
		*(DRAM_data+123) = (int)(IC7 * s);
//		*(DRAM_data+124) = (int)(IC9 * w);
//		*(DRAM_data+125) = (int)(IC11 * w);
//		*(DRAM_data+126) = (int)(IC13 * w);
//		*(DRAM_data+127) = (int)(IC15 * w);
//		*(DRAM_data+128) = (int)(IC17_25 * w);

		*(DRAM_data+129) = (int)(I03 * s);
		*(DRAM_data+130) = (int)(I05 * w);
		*(DRAM_data+131) = (int)(I07 * w);
//		*(DRAM_data+132) = (int)(I09 * w);
//		*(DRAM_data+133) = (int)(I011 * w);
//		*(DRAM_data+134) = (int)(I013 * w);
//		*(DRAM_data+135) = (int)(I015 * w);

		data_flag5=0;
     }
	 else
	    *(DRAM_data+104) = 0;
		
		*( DRAM_data + 0x7FFE ) = 0x1111;  //置INTL为低电平

        delay_t(240);//240
	//读取DRAM中的数据
	while ( GpioDataRegs.GPADAT.bit.GPIO18 == 1 )
	{
	    ;
	}	
		UDC_R = *(DRAM_data+0);
	//	THETA_C = *(DRAM_data+1);
		CROSS_ZERO = *(DRAM_data+2);
	//	UAADRF = *(DRAM_data+3);
	//	UBADRF = *(DRAM_data+4);
	//	UCADRF = *(DRAM_data+5);
		IAADRF = *(DRAM_data+6);		//真实电流的100倍作为定标值
		IBADRF = *(DRAM_data+7);
		ICADRF = *(DRAM_data+8);
		I0ADRF = *(DRAM_data+9);
	    INT_data1 = *(DRAM_data+0x7FFF);
    //保护计算

//if((data_flag3 == 1)&&(data_flag1 == 0))
//   graph_data[graph_num] = IAADRF;//AdcRefHighActualValue_f-3513.5;//UAADRF;
//else
//   graph_data[graph_num] = 0;
//   graph_data1[graph_num] = IAADRF1;//AdcRefLowActualValue_f-2754; 
//   graph_data2[graph_num] = IBADRF1;
//   graph_num++;
//   if ( graph_num > 499 )
//   {
//       graph_num -= 500;
//   }
	    //电网缺相
		ERROR_flag = 0x0;

		if( ( (UAADRF + UBADRF + UCADRF) > 200 )||((UAADRF + UBADRF + UCADRF) < -200 ))
		ERROR_flag |= 0x10;
		//电网箱序错
		THETA_0 = THETA_1;
		THETA_1 = THETA_C;
		if(THETA_0 > THETA_1)
		{
			Count_n3++;
			if(Count_n3 > 10)
			ERROR_flag |= 0x20;
		}
		else
		{
			Count_n3=0;
		}			
       // if(ERROR_flag>0)
	   //data_flag1=2;
		
		//数据存储
		if((CROSS_ZERO == 1)&&(data_flag1 == 0))  //如果过零且THD计算完毕
			{
				data_flag3++;
			}

		if((data_flag3 == 1)&&(data_flag1 == 0))
		{
			ILa1[Count_n1] = IAADRF * 0.1;//实际电流参与FFT运算
 			ILb1[Count_n1] = IBADRF * 0.1;
			ILc1[Count_n1] = ICADRF * 0.1;
			IL01[Count_n1] = I0ADRF * 0.1;
//			graph_data1[Count_n1] = IAADRF1;			
			Count_n1++;
		}
		if(data_flag3 == 2)
		{
			Count_n1 = 0;      //记点的起始值置零
			data_flag1 = 1;    //可以进行THD计算
			data_flag3 = 0;
		}

	//数据发送
	 	if(data_flag4==1)
		{
			*(DRAM_data+9) = ERROR_flag;
			*(DRAM_data+11) = THD_IA;
			*(DRAM_data+12) = THD_IB;
			*(DRAM_data+13) = THD_IC;
			*(DRAM_data+14) = I0;//I0
			*(DRAM_data+15) = IA;
			*(DRAM_data+16) = IB;	
			*(DRAM_data+17) = IC;
			*(DRAM_data+18) = CompensateA;
			*(DRAM_data+19) = CompensateB;
			*(DRAM_data+20) = CompensateC;
			*(DRAM_data+21) = THD_IA3;
			*(DRAM_data+22) = THD_IA5;
			*(DRAM_data+23) = THD_IA7;
			*(DRAM_data+24) = THD_IA9;
			*(DRAM_data+25) = THD_IA11;
			*(DRAM_data+0x7FFE) = 0x1111;  //
			data_flag4=0;
		}


		GpioDataRegs.GPBCLEAR.bit.GPIO56=1;
   	// Acknowledge this interrupt to get more from group 1
	   	PieCtrlRegs.PIEACK.all = 0x0001;
       	EINT;

}
/*****************************显示主程序*********************************/
void Display_main(void)
{
/*			*(DRAM_data+9)=0;
			*(DRAM_data+11)=0x1111;
			*(DRAM_data+12)=0x2222;
			*(DRAM_data+13)=0x3333;
			*(DRAM_data+14)=0x4444;
			*(DRAM_data+15)=0x5555;
			*(DRAM_data+16)=0x6666;	
			*(DRAM_data+17)=0x7777;
			*(DRAM_data+18)=0;
			*(DRAM_data+19)=0;
			*(DRAM_data+20)=0;
			*(DRAM_data+21)=0;
			*(DRAM_data+22)=0;
			*(DRAM_data+23)=0;
			*(DRAM_data+24)=0;
			*(DRAM_data+25)=0;	 
*/
	while(1)
    { 
	//	GpioDataRegs.GPBSET.bit.GPIO55=1;
		if(data_flag1==1)
		{
			GpioDataRegs.GPBSET.bit.GPIO57=1;  //测试用
//		    DINT;
			for(jflag=0;jflag<128;jflag++)              //提取一个周期中的电流数值
			{
				iflag=(int)(jflag * 1.953125);//1.5625);		//刘彬：这里为什么要乘以1.56？？？

			    //MN=ILa1[0];
				//M[jflag]=ILa1[iflag];
				ILa[jflag]=ILa1[iflag];
				ILb[jflag]=ILb1[iflag];
				ILc[jflag]=ILc1[iflag];
				IL0[jflag]=IL01[iflag];
//				graph_data[jflag]=ILa1[iflag];
//				graph_data1[jflag]=ILa[jflag];
			}
			FFT(ILa,ILa_0,Wa,IA_d,IA_q);
			FFT(ILb,ILb_0,Wb,IB_d,IB_q);
			FFT(ILc,ILc_0,Wc,IC_d,IC_q);

//*****************************3、5、7、9、11次电流值计算*********************************/

           IA3=sqrt(Wa[3]);//SQRT_TABLE[(int)IA3];
           IA5=sqrt(Wa[5]);//SQRT_TABLE[(int)IA5];
           IA7=sqrt(Wa[7]);//SQRT_TABLE[(int)IA7];
           IA9=sqrt(Wa[9]);//SQRT_TABLE[(int)IA9];
           IA11=sqrt(Wa[11]);//SQRT_TABLE[(int)IA11];
           IA13=sqrt(Wa[13]);
           IA15=sqrt(Wa[15]);
           IA17_25=sqrt(Wa[17])+sqrt(Wa[19])+sqrt(Wa[21])+sqrt(Wa[23])+sqrt(Wa[25]);
           
		   IB3=sqrt(Wb[3]);//SQRT_TABLE[(int)IA3];
           IB5=sqrt(Wb[5]);//SQRT_TABLE[(int)IA5];
           IB7=sqrt(Wb[7]);//SQRT_TABLE[(int)IA7];
           IB9=sqrt(Wb[9]);//SQRT_TABLE[(int)IA9];
           IB11=sqrt(Wb[11]);//SQRT_TABLE[(int)IA11];
           IB13=sqrt(Wb[13]);
           IB15=sqrt(Wb[15]);
           IB17_25=sqrt(Wb[17])+sqrt(Wb[19])+sqrt(Wb[21])+sqrt(Wb[23])+sqrt(Wb[25]);

		   IC3=sqrt(Wc[3]);//SQRT_TABLE[(int)IA3];
           IC5=sqrt(Wc[5]);//SQRT_TABLE[(int)IA5];
           IC7=sqrt(Wc[7]);//SQRT_TABLE[(int)IA7];
           IC9=sqrt(Wc[9]);//SQRT_TABLE[(int)IA9];
           IC11=sqrt(Wc[11]);//SQRT_TABLE[(int)IA11];
           IC13=sqrt(Wc[13]);
           IC15=sqrt(Wc[15]);
           IC17_25=sqrt(Wc[17])+sqrt(Wc[19])+sqrt(Wc[21])+sqrt(Wc[23])+sqrt(Wc[25]);

		   I03=IA3+IB3+IC3;//SQRT_TABLE[(int)IA3];
           
           I05_d=IA5-0.5*IB5-0.5*IC5;
           I05_q=0.866*IB5-0.866*IC5;
           I05=sqrt(I05_d*I05_d+I05_q*I05_q);//零线5次谐波幅值
           
		   I07_d=IA7-0.5*IB7-0.5*IC7;
           I07_q=0.866*IB7-0.866*IC7;
           I07=sqrt(I07_d*I07_d+I07_q*I07_q);

           I09=IA9+IB9+IC9;//SQRT_TABLE[(int)IA3];
		   
		   I011_d=IA11-0.5*IB11-0.5*IC11;
           I011_q=0.866*IB11-0.866*IC11;
           I011=sqrt(I011_d*I011_d+I011_q*I011_q);

		   I013_d=IA13-0.5*IB13-0.5*IC13;
           I013_q=0.866*IB13-0.866*IC13;
           I013=sqrt(I013_d*I013_d+I013_q*I013_q);

		   I015=IA15+IB15+IC15;//SQRT_TABLE[(int)IA3];
           		  		  
		   data_flag5=1;

//******************************* *基波电流计算*********************************************

        	//FFT(IL0,IL0_0,W0);
			//基波有效值有效值必须小于100
			//用查表的方式来实现开方计算
			//算法实现如下：
			//假设需要查0~8192的开方那么你做了一个长度为1000的表，
			//表中的内容为0 （8192/1000）, 8192*2/1000, 8192*3/1000, 8192*4/1000......8192*1000/1000的开方值
			//那么当你计算4000的开方值时，你需要4000*1000/8192的那个指针对应的开方值即可
			//计算后默认十倍定标
           IA=Wa[1];
           IA=sqrt(IA);//SQRT_TABLE[IA];
           IB=Wb[1];//400 * 0.0001);
           IB=sqrt(IB);//SQRT_TABLE[IB];
           IC=Wc[1];//400 * 0.0001);
           IC=sqrt(IC);//SQRT_TABLE[IC];
		   I0=W0[1];//400 * 0.0001);
           I0=sqrt(I0);//SQRT_TABLE[I0];


//*****************************THD值计算*********************************/

		   IAALL=IBALL=ICALL=0;
           for(i=2;i<32;i++)
           {
           		IAALL+=Wa[i];
           		IBALL+=Wb[i]; 
				ICALL+=Wc[i];	
   		   }
		   IAALLint=IAALL;//500 * 0.0001);
		   IAALL=sqrt(IAALLint);//SQRT_TABLE[IAALLint];
		   CompensateA = IAALL;
		   IBALLint=IBALL;//500 * 0.0001);
		   IBALL=sqrt(IBALLint);//SQRT_TABLE[IBALLint];
		   CompensateB = IBALL;
		   ICALLint=ICALL;//500 * 0.0001);
		   ICALL=sqrt(ICALLint);//SQRT_TABLE[ICALLint];
		   CompensateC = ICALL;
	   if(IA > 1)
	   {
	   THD_IA=(int)(IAALL * 100 / IA);//
	   THD_IB=(int)(IBALL * 100 / IB);
	   THD_IC=(int)(ICALL * 100 / IC);
		
//*****************************3、5、7、9、11次电流THD值计算*********************************/
       THD_IA3=(int)(IA3 * 100 / IA);//* back_IA);
	   THD_IA5=(int)(IA5 * 100 / IA);//* back_IA);
	   THD_IA7=(int)(IA7 * 100 / IA);//* back_IA);
	   THD_IA9=(int)(IA9 * 100 / IA);//* back_IA);
	   THD_IA11=(int)(IA11 * 100 / IA);//* back_IA);
	   }
	   else
	   {
	   THD_IA=0;//
	   THD_IB=0;
	   THD_IC=0;	

//*****************************3、5、7、9、11次电流THD值计算*********************************/
       THD_IA3=0;//* back_IA);
	   THD_IA5=0;//* back_IA);
	   THD_IA7=0;//* back_IA);
	   THD_IA9=0;//* back_IA);
	   THD_IA11=0;//* back_IA);
	   }

		   data_flag1=0;
		   data_flag4=1;
//		   MN1=((IA_d[1] * IA_d[1])+(IA_q[1] * IA_q[1]));
//		   MN2=0.707*sqrt((IB_d[1] * IB_d[1])+(IB_q[1] * IB_q[1]));
//		   MN3=0.707*sqrt((IC_d[1] * IC_d[1])+(IC_q[1] * IC_q[1]));
		   GpioDataRegs.GPBCLEAR.bit.GPIO57=1;
		}
		delay_t(100);
	//	GpioDataRegs.GPBCLEAR.bit.GPIO55=1;
	}
}
/******************************显示数据计算*****************************/
//说明求出的谐波是各次谐波的平方
void FFT(float dataR[],float dataI[],float w[],float I_d[],float I_q[])
	{int x0,x1,x2,x3,x4,x5,x6;
	int L,j,k,b,p;
	float TR,TI,temp;
	int i,xx;

/********** following code invert sequence ************/
//按时间抽取（DIT）的FFT算法通常将原始数据倒位序存储，
//最后按正常顺序输出结果X（0），X（1），...，X（k），...。
//假设一开始，数据在数组 float dataR[128]中，我们将下标i表示为（x6x5x4x3x2x1x0），
//倒位序存放就是将原来第i个位置的元素存放到第（x0x1x2x3x4x5x6）的位置上去.
//由于C语言的位操作能力很强，可以分别提取出x6、x5、x4、x3、x2、x1、x0，再重新组合成x0、x1、x2、x3、x4、x5、x6，即是倒位序的位置。
//程序段如下（假设128点FFT�
// i为原始存放位置，最后得invert_pos为倒位序存放位置.

	for(i=0;i<128;i++)
	{ 
		x0=x1=x2=x3=x4=x5=x6=0;
		x0=i&0x01;
		x1=(i/2)&0x01;
		x2=(i/4)&0x01; 
		x3=(i/8)&0x01;
		x4=(i/16)&0x01; 
		x5=(i/32)&0x01; 
		x6=(i/64)&0x01;
		xx=x0*64+x1*32+x2*16+x3*8+x4*4+x5*2+x6;
		dataI[xx]=dataR[i];
	//	graph_data1[i]=xx;
	}

	for(i=0;i<128;i++)
	{ 
		dataR[i]=dataI[i]; dataI[i]=0; 
	}
/************** following code FFT *******************/
//蝶形公式：
//X(K) = X’(K) + X’(K+B)W PN ,
//X(K+B) = X’(K) - X’(K+B) W PN
//其中W PN= cos（2πP/N）- jsin（2πP/N）。
//设 X(K+B) = XR(K+B) + jXI(K+B),
//X(K) = XR(K) + jXI(K) ,
//有:
//XR(K)+jXI(K)= XR’(K)+jXI’(K)+[ XR’(K+B) + jXI’(K+B)]*[ cos（2πP/N）-jsin（2πP/N）];
//继续分解得到下列两式:
//XR(K)= XR’(K)+ XR’(K+B) cos（2πP/N）+ XI’(K+B) sin (2πP/N) （1）
//XI(K)= XI’(K)-XR’(K+B) sin（2πP/N）+XI’(K+B)cos (2πP/N) （2）
//需要注意的是: XR(K)、XR’(K)的存储位置相同,所以经过（1）、（2）后，该位置上的值已经改变，而下面求X(K+B)玫絏’(K)，因此在编程时要注意保存XR’(K)和XI’(K)到TR和TI两个临时变量中。
//同理: XR(K+B)+jXI(K+B)= XR’(K)+jXI’(K)- [ XR’(K+B)+jXI’(K+B)] *[ cos（2πP/N）-jsin（2πP/N）]继续分解得到下列两式:
//XR(K+B)= XR’(K)-XR’(K+B) cos（2πP/N）- XI’(K+B) sin (2πP/N) （3）
//XI(K+B)= XI’(K)+ XR’(K+B) sin（2πP/N）- XI’(K+B) cos (2πP/N) （4）
//注意：
//① 在编程时, 式（3）、（4）中的XR’(K)和 XI’(K)分别用TR和TI代替。
//② 经过式（3）后， XR(K+B)的值已变化，而式（4）中要用到该位置上的上一级值，所以在执行式（3）前要先将上一级的值XR’(K+B)保存。
//③ 在编程时, XR(K)和 XR’(K), XI(K)和 XI’(K)使用同一个变量。
 //通过以上分析，我们只要将式（1）、（2）、（3）、（4）转换成C语言语句即可。要注意变量的中间保存，详见以下程序巍?

	for(L=1;L<=7;L++) 
	{ /* for(1) */
		b=1; i=L-1;
		while(i>0) 
		{
			b=b*2; i--;
		} /* b= 2^(L-1) */
		for(j=0;j<=b-1;j++) /* for (2) */
		{
	 		p=1; i=7-L;
			while(i>0) /* p=pow(2,7-L)*j; */
			{
			p=p*2; i--;
			}
			p=p*j;
			for(k=j;k<128;k=k+2*b) /* for (3) */
			{ 
				TR=dataR[k]; 
				TI=dataI[k]; 
				temp=dataR[k+b];

				dataR[k]=dataR[k]+dataR[k+b]*COS_CONST[p]+dataI[k+b]*SIN_CONST[p];
				dataI[k]=dataI[k]-dataR[k+b]*SIN_CONST[p]+dataI[k+b]*COS_CONST[p];
				dataR[k+b]=TR-dataR[k+b]*COS_CONST[p]-dataI[k+b]*SIN_CONST[p];
				dataI[k+b]=TI+temp*SIN_CONST[p]-dataI[k+b]*COS_CONST[p];
//			while((dataR[k]>1000000)||(dataI[k]>1000000))
//			{
//			m1=k;
//			}
//			while((dataR[k+b]>1000000)||(dataI[k+b]>1000000))
//			{
//			m1=k;
//			m2=k+b;
//			}
			
			} /* END for (3) */
		} /* END for (2) */
	} /* END for (1) */



	for(i=0;i<32;i++)
	{ /* 只需要32次以下的谐波进行分析 */
	    I_d[i]=dataR[i] * 0.015625;//1/64
    	I_q[i]=dataI[i] * 0.015625;
		w[i]=I_d[i]*I_d[i]+I_q[i]*I_q[i];
	//	w[i]=w[i]/4096; // /64*64;
	}
		w[0]=w[0]/4; // /2*2;
} /* END FFT */


/*DIT FFT 算法的基本思想分析
　　我们知道N点FFT运算可以分成LOGN2 级，每一级都有N/2个碟形IT FFT的基本思想是用3层循环完成全部运算(N点FFT)。
　　第一层循环：由于N=2m需要m级计算,第一层循环对运算的级数进行控制。
 　　第二层循环：由于第L级有2L-1个蝶形因子（乘数），第二层循环根据乘数进行控制，保证对于每一个蝶形因子第三层循环要执行一次，这样，第三层循环在第二层循环控制下，每一级要?L-1次循环计算。
 　　第三层循环：由于第L级共有N/2L个群，并且同一级内不同群的乘数分布相同，当第二阊啡范骋怀耸螅谌阊芬炯吨忻扛鋈褐芯哂姓庖怀耸牡渭扑阋淮危吹谌阊访恐葱暌匆蠳/2L个碟形计算。
 　　可以得出结论：在每一级中，第三层循环完成N/2L个碟形计算；第二层循环使第三层循环进行 2L-1次，因此，第二层循环完成时，共进?L-1 *N/2L=N/2个碟形计算。实质是：第二、第三层循环完成了贚级的计算。
　　几个要注意的数据：
　　①  在第L级中，每龅蔚两个输入端相距b=2L-1个点。
　　② 同一乘数对应着相邻间隔为2L个点的N/2L个碟形。
　　③ 第L级的2L-1个碟形因子WPN 中的P，可表示为p = j*2m-L，其中j = 0，1，2，...，（2L-1-1）。
*/


/******************************初始化***************************************/
//系统时钟及外设时钟初始化
void InitSysCtrl(void)
{

   // Disable the watchdog
   DisableDog();

   // Initialize the PLL control: PLLCR and DIVSEL
   // DSP28_PLLCR and DSP28_DIVSEL are defined in DSP2833x_Examples.h
   InitPll(DSP28_PLLCR,DSP28_DIVSEL);

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
    SysCtrlRegs.WDCR= 0x0068;
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

      while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1)
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

   SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 0;   // I2C
   SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 0;   // SCI-A
   SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 0;   // SCI-B
   SysCtrlRegs.PCLKCR0.bit.SCICENCLK = 1;   // SCI-C
   SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 0;   // SPI-A
   SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = 1; // McBSP-A
   SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK = 1; // McBSP-B
   SysCtrlRegs.PCLKCR0.bit.ECANAENCLK=0;    // eCAN-A
   SysCtrlRegs.PCLKCR0.bit.ECANBENCLK=0;    // eCAN-B

   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Disable TBCLK within the ePWM
   SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 0;  // ePWM1
   SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 0;  // ePWM2
   SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 0;  // ePWM3
   SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 0;  // ePWM4
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
    while(SourceAddr < SourceEndAddr)
    { 
       *DestAddr++ = *SourceAddr++;
    }
    return;
}



/******************************通用函数*********************************/
void delay_t(unsigned a)  
{

  for(n=0;n<a;n++)					 
  {
  ;
  }
}

float LP_Filter_1st(float *y, float *x)
{  
   /*
   入口：一阶低通滤波器输入值数组首地址，x[]包含2个变量，x[0]输入新值
        x[1]输入的前一次值.
        一阶低通滤波器以前输出值数组首地址,其中y[0]表示输出的前一次值，。
   出口：一阶低通滤波器输出值。
   中间变量：y_new用于暂时存储一阶低通滤波器的当前输出值。
   使用条件：滤波器的常数对应的采样频率为9.375KHz，截止频率2Kz。
   */
    float y_new;
	y_new = ( F_1st_Cnst1 * y[0] ) + ( F_1st_Cnst2 * (x[0] + x[1]) );
	y[0] = y_new;
	x[1] = x[0];
    return y_new;
}

float LP_Filter_2nd(float *y, float *x)
{  
   /*
   入口：二阶低通滤波器输入值数组首地址，x[]包含3个变量，x[0]输入新值
        x[1]淙氲那耙淮沃担瑇[2]输入的前两次值。
        二阶低通滤波器以前输出值数组首地址,其中y[0]表示输出的前一次值，
		y[1]表示输出的前两次值。
   出口：二阶低通滤波器输出值。
   中间变量：y_new用于暂时存储二阶低通滤波器的当前输出值。
   使用条件：滤波器的常数对应的采样频率为9.375KHz，截止频率700Hz。离散化方法为双线性变化法。
   */
    float y_new;
	y_new = ( F_2nd_Cnst1 * y[0] )
		    - ( F_2nd_Cnst2 * y[1] )
			+ ( F_2nd_Cnst3 * ( x[0] + x[2] + (2 * x[1]) ) );

	y[1] = y[0];
	y[0] = y_new;
	x[2] = x[1];
	x[1] = x[0];
    return y_new;
}


float BP_Filter_2nd( float *y, float *x )
{
//入口：二阶低通滤波器输入值数组首地址，x[]包含3个变量，x[0]输入新值
        //x[1]输入的前一次值，x[2]输入的前两次值，x[3]表示输入的前三次值。
        //二阶低通滤波器以前输出值数组首地址,其中y[0]表示输出的前一次值，
		//y[1]表示输出的前两次值，依次类推。
   //出口：二阶低通滤波器输出值。
    float y_new;
	y_new = ( BP_2nd_Cnst1 * y[0] ) - ( BP_2nd_Cnst2 * y[1] )
	        + ( BP_2nd_Cnst3 * y[2] ) - ( BP_2nd_Cnst4 * y[3] )
			+ ( BP_2nd_Cnst5 * ( x[0] - 2*x[2] + x[4] ) );
	y[3] = y[2];
	y[2] = y[1];
	y[1] = y[0];
	y[0] = y_new;
	x[4] = x[3];
	x[3] = x[2];
	x[2] = x[1];
	x[1] = x[0];
	return y_new;
}

float BP_Filter_1st( float *y, float *x )
{
//入口：一阶带通滤波器输入值数组首地址，x[]包含3个变量，x[0]输入新值
        //x[1]输入的前一次值，x[2]输入的前两次值，x[3]表示输入的前三次值。
        //一阶带通滤波器其中y[0]表示输出的前一次值，
		//y[1]表示输出的前两次值，依次类推。
   //出口：一阶带通滤波器输出值。
    float y_new;
	y_new = BP_1st_Cnst1 * y[0] - BP_1st_Cnst2 * y[1] + BP_1st_Cnst3 * (x[0] - x[2]);
	y[1] = y[0];
	y[0] = y_new;
	x[2] = x[1];
	x[1] = x[0];

	return y_new;
}

//计算正弦和余弦值
void Sin_Value( float theta )
{
	int temp;
	float AB[2]={0,0}; 
	  
	if (theta>=360)
	theta-=360;
	   
	if(theta==0)				//sin(0)
	{
		AB[0]=0;
		AB[1]=1;
	}
	else if(theta==90)		//sin(90)
	{
		AB[0]=1;
		AB[1]=0;
	}
	else if(theta==180)		//sin(180)
	{
		AB[0]=0;
		AB[1]=-1;
	}
	else if(theta==270)		//sin(270)
	{
		AB[0]=-1;
		AB[1]=0;
	}
	else
	{
		if(theta<90)
		{
			temp=(int)(theta * Theta_DB);		//sin 
			AB[0]=SIN_TABLE[temp];
			temp=(int)((90-theta) * Theta_DB);	//cos=sin(90-theta)
			AB[1]=SIN_TABLE[temp];
		}
		else if(theta<180)
		{
			temp=(int)((180-theta) * Theta_DB);		//sin 
			AB[0]=SIN_TABLE[temp];
			temp=(int)((theta-90) * Theta_DB);	//cos=sin(90-theta)
			AB[1]=-SIN_TABLE[temp];			
		}
		else if(theta<270)
		{
			temp=(int)((theta-180) * Theta_DB);		//sin 
			AB[0]=-SIN_TABLE[temp];
			temp=(int)((270-theta) * Theta_DB);	//cos=sin(90-theta)
			AB[1]=-SIN_TABLE[temp];						
		}
		else if(theta<360)
		{
			temp=(int)((360-theta) * Theta_DB);		//sin 
			AB[0]=-SIN_TABLE[temp];
			temp=(int)((theta-270) * Theta_DB);	//cos=sin(90-theta)
			AB[1]=SIN_TABLE[temp];								
		}
	}
	sinVal=AB[0];
	cosVal=AB[1];
}

void Check_Harmonic_N1(int H_n)
{
	float temp =0;
	//float IAd,IAq,IBd,IBq,ICd,ICq;
    if((H_n==23)||(H_n==21))
	temp = H_n * (THETA_C + 90) + H_n * 18 * Sampl_Interval * (COM_EXT_count + 4) * 0.5 ;
    else if(H_n==25)
	temp = H_n * (THETA_C + 90) + H_n * 18 * Sampl_Interval * (COM_EXT_count + 5) * 0.5 ;
    else if(H_n==19)
	temp = H_n * (THETA_C + 90) + H_n * 18 * Sampl_Interval * (COM_EXT_count + 3) * 0.5 ;
	else if((H_n==17)||(H_n==15))
	temp = H_n * (THETA_C + 90) + H_n * 18 * Sampl_Interval * (COM_EXT_count + 1) * 0.5 ;
    else
    temp = H_n * (THETA_C + 90) + H_n * 18 * Sampl_Interval * COM_EXT_count * 0.5 ;
	while( temp >= 360 )
	{
	   temp -= 360;
	}
	Sin_Value(temp);
    if((H_n==1)||(H_n==5)||(H_n==13)||(H_n==17)||(H_n==25)||(H_n==9)||(H_n==21))
	{
	    IAh = (IA_q_toA[H_n] * cosVal + IA_d_toA[H_n] * sinVal); //IA_q IA_d是和差公式
	    IBh = (IB_q_toA[H_n] * cosVal + IB_d_toA[H_n] * sinVal); //查一下傅里叶公式
	    ICh = (IC_q_toA[H_n] * cosVal + IC_d_toA[H_n] * sinVal); 
	}
	else
	{
        IAh = (-IA_q_toA[H_n] * cosVal - IA_d_toA[H_n] * sinVal);
	    IBh = (-IB_q_toA[H_n] * cosVal - IB_d_toA[H_n] * sinVal);
	    ICh = (-IC_q_toA[H_n] * cosVal - IC_d_toA[H_n] * sinVal);
	} 
        
}
//===========================================================================
// No more.
//===========================================================================

