/*
//###########################################################################
//
// FILE:  Variable.c
//
// DESCRIPTION:
//
//         including those variables used in the Communication test program
//
//###########################################################################
// $Author: Guangxin  zhang $
// $Date: March  31, 2008$
// $Target System: DF28335 APF $
// $Version: v1.0 $
//###########################################################################

*/

//********************相位计算用到的变量*************************
int     x1=0;
int     CS=0;
float	THETA_C=0;
float  	UAADRF=0;
float	UBADRF=0;
float	UCADRF=0;
float  	UDC_R=0;
float 	IAADRF=0;
float 	IBADRF=0;
float 	ICADRF=0;
float 	I0ADRF=0;

//int 	IAADRF1=0;
//int 	IBADRF1=0;
//int 	ICADRF1=0;
//int 	I0ADRF1=0;

float graph_data[500]={0};
float graph_data1[500]={0};
float graph_data2[500] = {0};


float ILa1[250]={0};
float ILb1[250]={0};
float ILc1[250]={0};
float IL01[250]={0};

int s=0;
int w=0;
float MN1=0;
float MN2=0;
float MN3=0;
//float M[250]={0};
float ILa[128]={0};
float ILa_0[128]={0};
float IA_d[32]={0};
float IA_q[32]={0};
float Wa[32]={0};
float ILb[128]={0};
float ILb_0[128]={0};
float IB_d[32]={0};
float IB_q[32]={0};
float Wb[32]={0};
float ILc[128]={0};
float ILc_0[128]={0};
float IC_d[32]={0};
float IC_q[32]={0};
float Wc[32]={0};
float IL0[128]={0};
float IL0_0[128]={0};
float W0[32]={0};

int  Count_n1=0;   //对应数组中数据位置
int  data_flag1=0;
int  INT_data1=0;
int  ERROR_flag=0;
int  THD_IA=0;
int  THD_IB=0;
int  THD_IC=0;
int  THD_I0=0;
float  IA = 0;
float  IB = 0;
float  IC = 0;
float  I0 = 0;
int  data_flag2=0;
int  Count_n2=0;  //数组中存在点的个数
int  Count_n3=0;  //相序错用到的记点个数
int  THETA_0=0;
int  THETA_1=0;
float  IAALL=0;     //所有次谐波的有效值I=IH1*IH1+IH2*IH2+IH2*IH2+****
float  IBALL=0;
float  ICALL=0;
int  CROSS_ZERO=0;
int  jflag=0;
int  iflag=0;
int  data_flag3=0;
float   IAALLint=0;
float   IBALLint=0;
float   ICALLint=0;
int  data_flag4=0;
int  data_flag5=0;
//***********************AD采样用到的变量*********************888
int TEMP1=0;
int MON15=0;
int TEMP2=0;
int RESERVE1=0;
int MON5=0;
//*********************作图**************************
unsigned graph_num=0;
//*********************电网电压滤波及其校正后的变量***************
float UAADRF_f = 0;   //滤波后变量
float UAADRF_f_C = 0; //滤波并且校正后的变量
float UBADRF_f = 0;
float UBADRF_f_C = 0;
float UCADRF_f = 0;
float UCADRF_f_C = 0;
//**********************ADC精度校正变量***********************
float AdcRefHighActualValue=0;
float AdcRefLowActualValue=0;
float AdcCalGain=0;
float AdcCalOffset=0;
float AdcRefHighActualValue_f=0;
float AdcRefLowActualValue_f=0;
//********************3、5、7、9、11次电流值显示所用变量*************************
float  IA3=0;
float  IA5=0;
float  IA7=0;
float  IA9=0;
float  IA11=0;
float  IA13=0;
float  IA15=0;
float  IA17_25=0;
//float  IA19=0;
//float  IA21=0;
//float  IA23=0;
//float  IA25=0;
float  IB3=0;
float  IB5=0;
float  IB7=0;
float  IB9=0;
float  IB11=0;
float  IB13=0;
float  IB15=0; 
float  IB17_25=0; 

float  IC3=0;
float  IC5=0;
float  IC7=0;
float  IC9=0;
float  IC11=0;
float  IC13=0;
float  IC15=0; 
float  IC17_25=0; 

float  I03=0;
float  I05=0;
float  I05_d=0;
float  I05_q=0;
float  I07=0;
float  I07_d=0;
float  I07_q=0;
float  I09=0;
float  I011=0;
float  I011_d=0;
float  I011_q=0;
float  I013=0;
float  I013_d=0;
float  I013_q=0;
float  I015=0;  

int  THD_IA3=0;
int  THD_IA5=0;
int  THD_IA7=0;
int  THD_IA9=0;
int  THD_IA11=0;
int  THD_IA13=0;
int  THD_IA17=0;

int CompensateA = 0;//补偿电流有效值
int CompensateB = 0;
int CompensateC = 0;



//三相电压滤波前后用到的数组
float UA_x[5] = {0};
float UA_y[4] = {0};
float UB_x[5] = {0};
float UB_y[4] = {0};
float UC_x[5] = {0};
float UC_y[4] = {0};

//校正用的直流量滤波前后用到的数组
float AdcRefH_x[2] = {0};
float AdcRefH_y[1] = {3400};//为防止校正时初始值溢出，AdcRefHighActualValue_f和AdcRefLowActualValue_f初始值不能为零
float AdcRefL_x[2] = {0};
float AdcRefL_y[1] = {2683};//为防止校正时初始值溢出，AdcRefHighActualValue_f和AdcRefLowActualValue_f初始值不能为零

float sinVal = 0;
float cosVal = 0;
float Omiga_t = 0;
float   invented_integral=0;

/*******用于计算部分高次谐波***********/
 float IAh_Part_Sum = 0;
 float IBh_Part_Sum = 0;
 float ICh_Part_Sum = 0;
 float IAh=0;
 float IBh=0;
 float ICh=0;
 float COM_EXT_count=4;
float IA_d_toA[30]={0};
float IA_q_toA[30]={0};
float IB_d_toA[30]={0};
float IB_q_toA[30]={0};
float IC_d_toA[30]={0};
float IC_q_toA[30]={0};






