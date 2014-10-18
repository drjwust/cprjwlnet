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
// $Author: Han bofoster $
// $Date: May  11, 2011$
// $Target System: DF28335 APF $
// $Version: v2.0 $
//###########################################################################

*/
//********************电压环用到的变量**************************
float   x1=2.2;//零漂
float   y1=5;//幅值
int 	UDCADRF=0;             //直流侧电压补偿零漂后值
float  	UDC_R=0;            //定标后对应的实际电压值
float  	UDC_Rf1=140;	//定标并滤波后的实际电压
float	UDC_Rf2=140;
			
float	UDC_Kp=200;
float	UDC_Ki=50;
float   I_Kp=8;
float   I_Ki=0;
float	UDC_erro_old=0;
float	UDC_erro_new=0;
float	Udc_slip=140;
float	Id_slip=0; 
float	Idref=0;		//Udc_PI调节器出来的Id_ref
float   Inref=0;        //Udc积分项限幅
float   T_cout=0;
//float   Error_IA=0;
//float   Error_IB=0;
//float   Error_IC=0;
//********************相位计算用到的变量*************************
float	THETA_C=0;
float 	THETA_C_B = 0.0;
float	THETA_C_C = 0.0;
float   Omiga_t=0;
//float   Omiga_t2=0;
int     PLL=1;
float   invented_integral=0;
float  	UAADRF=0;
float	UBADRF=0;
float	UCADRF=0;

float	U1_a=0;		//基波电压
float	U1_b=0;
float	U1_c=0;

float   Ua_APF=0;
float   Ub_APF=0;
float   Uc_APF=0; 
float   U0_APF = 0;

float	LS1=0;		//三相静止坐标下的临时存储变量
float	LS2=0;
float	LS3=0;

float I_temp1=0;    //仅用一次
float I_temp2=0;
float temp3=0;
float temp4=0;

float Ih_a=0;       //各相电流值
float Ih_b=0;
float Ih_c=0;
float I1_aqf_f = 0;
float I1_bqf_f = 0;
float I1_cqf_f = 0;
float Ihw_a = 0;
float Ihw_b = 0;
float Ihw_c = 0; 
float Ih_0=0;
float If_a=0;
float If_b=0;
float If_c=0;
float In_d=0;
//*********稳压PI输出滤波时，记录当前滤波输入值（当前一个和前2个）和滤波输出值（前2个）
float Idref_Fx[3] = {0};
float Idref_Fy[2] = {0};
//***********全补偿时，记录当前滤波输入值（当前一个和前2个）和滤波输出值（前2个）***

float I1d[3] = {0};
float I1df[2] = {0};
float I1q[3] = {0};
float I1qf[2] = {0};
//*******************全补偿部分所需数组结束*******************************

	//**************分次补偿所用变量**********************(三相四线)
	float IAh = 0;
	float IAdh = 0;
	float IAqh = 0;
	float IBh = 0;
	float IBdh = 0;
	float IBqh = 0;
	float ICh = 0;
	float ICdh = 0;
	float ICqh = 0;
	float IAh_Sum = 0;
	float IBh_Sum = 0;
	float ICh_Sum = 0;

	float FilterPara1[6] = {0}; //分次补偿滤波输出值记录数组
	float FilterPara2[6] = {0};
	float FilterPara3[6] = {0};
	float FilterPara4[6] = {0};
	float FilterPara5[6] = {0};

//***********分次补偿滤波时，记录滤波输入值（当前一个和前2个）和滤波输出值（前2个）***
float In1_d[3] = {0};
float In1_df[2] = {0};
float In1_q[3] = {0};
float In1_qf[2] = {0};

float In2_d[3] = {0};
float In2_df[2] = {0};
float In2_q[3] = {0};
float In2_qf[2] = {0};

float In3_d[3] = {0};
float In3_df[2] = {0};
float In3_q[3] = {0};
float In3_qf[2] = {0};

float In4_d[3] = {0};
float In4_df[2] = {0};
float In4_q[3] = {0};
float In4_qf[2] = {0};
//***********************分次补偿滤波部分所需数组结束***************

float In_q=0;
float cosVal=0;
float sinVal=0;
float In_a=0;
float In_b=0;
float In_df=0;
float In_qf=0;
float Irm[25]={0};
float Ih_d=0;
float Ih_q=0;

float I1_a=0; //两相静止坐标系下的临时变量
float I1_b=0; 
float I1_adf=0;
float I1_aqf=0;
float I1_adf_1stOld=0;
float I1_adf_2ndOld=0;
float I1_aqf_1stOld=0;
float I1_aqf_2ndOld=0;

float I1_bdf=0;
float I1_bqf=0; 
float I1_bdf_1stOld=0;
float I1_bdf_2ndOld=0;
float I1_bqf_1stOld=0;
float I1_bqf_2ndOld=0;
float I1_cdf=0;
float I1_cqf=0;
float I1_cdf_1stOld=0;
float I1_cdf_2ndOld=0;
float I1_cqf_1stOld=0;
float I1_cqf_2ndOld=0;
float I1_d=0;
float I1_d_1stOld=0;
float I1_d_2ndOld=0;
float I1_q=0;
float I1_q_1stOld=0;
float I1_q_2ndOld=0;


int 	CROSS_IH=0; 
int 	CROSS_TOTAL=0; 
int 	CROSS_SUM=0; 
int 	CROSS_ZERO=0;
int		CROSS_FLAG=0;
int     THETA_0=0;
int     THETA_1=0; 
int 	n_cycle=0;
int 	m_cycle=0;
//int     j_cycle=0;
int 	EXT_count=2;  
float	I_H_A[361]={0};
float	I_H_B[361]={0};
float	I_H_C[361]={0};
//float   I_H_0[270]={0};
int Leading_Comp_Flag = 1; 

//float Ih5_CF;
float Ih_beta=0;
float Ih_alpha=0;
float Ih_0x = 0;
int AD_result[8]={0};
int DA_date[8]={0};
float IAADRF=0;
float IBADRF=0;
float ICADRF=0;
float I0ADRF=0;
float IAAPF=0;
float IBAPF=0;
float ICAPF=0;
float I0APF=0;
int Com_rate1=0;
/*
int com=0;
int Run_Stop_Flag=0;
int display_date_one[10]={0xAA,0x55,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
int display_date_two[10]={0xAA,0x55,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
int Parameters[10]={0};

int m=0;        //
int n=0;    	//在delay_t中有使用
int i=0;*/

float graph_data[500]={0};
int ReceivedChar=0;
float graph_data1[500]={0};
float graph_data2[500]={0};
float graph_data3[500]={0};
//float graph_date1[500]={0};
//float graph_data2[500]= {0}; 
int graph_temp1=0;
int graph_temp2=0;
int graph_temp3=0;



float UDC_Max = 0;
float Ih_Max = 0;
float U_FtoI = 0;
float UDCREF = 0;	//直线母线给定电压
float Ih_Limit = 0;		//限流输出限幅值
//int Limit_Flag = 0.0;		//限流运行标记位
//float If_Limit = 0.0;		//无功限流运行限幅
//float If_alpha = 0.0;		//无功指令值
//float If_beta = 0.0;
float Ih_d_Limit=0;
float Ih_q_Limit=0;
float I1_d_square=0;
float I1_q_square=0;
float I1_Square = 0.0;
float Ih_Limit_square = 0.0;
float I1_Value = 0.0;
float Phi_Limit = 0.0;



//通信相关
int com=0;
int com1=0;
int Run_Stop_Flag=0;
int Reactive_Power=0;
int Para_SET=0;
//int Fault_flag=0;
int Receive_date_error=0;
int Com_date[30]={0};
int Read_control[7]={0x1B,0x52,0x00,0x64,0x00,0x01,0xD3};
int display_date_one[14]={0x01,0x7C,0x01,0x7C,0x01,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int display_date_two[40]={0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x32,0x00,0x00};
int Para1_set[9]={0x1B,0x57,0x00,0xAA,0x00,0x01,0x00,0x01,0x8D};
int Para2_set[9]={0x1B,0x57,0x00,0xAA,0x00,0x01,0x00,0x02,0x8D};
int Para1_Intdate[40]={0x1B,0x57,0x00,0x96,0x00,0x05,0x00,0xC8,0x00,0xFA,0x00,0x02,0x00,0x3C,0x00,0x00,0x8D};
int Para2_Intdate[15]={0x1B,0x57,0x00,0xC8,0x00,0x04,0x00,0x64,0x00,0x32,0x01,0x40,0x00,0x8C,0x8D};
//int Current_IA[202]={0};
//int Current_IB[202]={0};
//int Current_IC[202]={0};
//int Current_I0[202]={0}; 
int C_data_count1=0;
int C_data_count2=0;
int C_data_count3=0;
int COM_UDCREF=0; //150
int COM_Ih_max=0;	//151
int C_way=0;	//152  .1=1全补。1=0分次补  。3=1 补偿基波无功
int C_Way1=0;
int C_Way2=0;
int C_Way3=0;
int COM_ALL_rate=0;	//153
int COM_EXT_count=0; //154
int COM_IHB_RATE=0;  //155
int COM_UDC_Max=0;	//156
int COM_IH1=0;		//157
int COM_IH1_RATE=0;	//158
int COM_IH2=0;		//159
int COM_IH2_RATE=0;	//160 
int COM_IH3=0;		//161
int COM_IH3_RATE=0;	//162
int COM_IH4=0;		//163
int COM_IH4_RATE=0;	//164
int COM_Kp=0;	//200
int COM_ki=0;	//201 
int COM_Udb=0;	//202
int COM_Ua=0;	//203
int COM_Tu=0;	//204
int COM_Ti=0;
//int Current_Kp=0; 	//205
int COM_Iz=0;	//206
int COM_Uz=0; 	//207
int COM_U=0;    //UABC 
int COM_UAB=0; 	//电网电压a  
int COM_UBC=0;	//电网电压b
int COM_UCA=0;	//电网电压c
int COM_IA=0;	//
int COM_IB=0;
int COM_IC=0;
int COM_I0=0;
int COM_Udc=0;
float COM_Udc1=0;
int ERROR_flag=0;
int Wrong_Flag = 0;
int Wrong_Temp = 0;
int INRL_data=0;
int Current_flag=0;
int Init_para_flag=0;
//********************THD值显示所用变量*************************
int  THD_IA=0;
int  THD_IB=0;
int  THD_IC=0;
int  THD_I0=0;
int  THD_IA3=0;
int  THD_IA5=0;
int  THD_IA7=0;
int  THD_IA9=0;
int  THD_IA11=0;
int CompensateA= 0;//输出补偿电流有效值
int CompensateB= 0;
int CompensateC= 0;
//********************电流值显示所用变量************************* 


int m=0;        //
int n=0;    	//在delay_t中有使用
int i=0;


float In1_adf=0;
float In1_aqf=0;
float Ih1_a=0;



float In1_bdf=0;
float In1_bqf=0;
float Ih1_b=0;



float In1_cdf=0;
float In1_cqf=0;
float Ih1_c=0;


float In2_adf=0;
float In2_aqf=0;
float Ih2_a=0;



float In2_bdf=0;
float In2_bqf=0;
float Ih2_b=0;



float In2_cdf=0;
float In2_cqf=0;
float Ih2_c=0;



float In3_adf=0;
float In3_aqf=0;
float Ih3_a=0;


float In3_bdf=0;
float In3_bqf=0;
float Ih3_b=0;



float In3_cdf=0;
float In3_cqf=0;
float Ih3_c=0;

float In4_adf=0;
float In4_aqf=0;
float Ih4_a=0;


float In4_bdf=0;
float In4_bqf=0;
float Ih4_b=0;


float Ih4_cdf=0;
float Ih4_cqf=0;
float Ih4_c=0;


float In5_adf=0;
float In5_aqf=0;
float Ih5_a=0;


float In5_bdf=0;
float In5_bqf=0;
float Ih5_b=0;


float Ih5_cdf=0;
float Ih5_cqf=0;
float Ih5_c=0;


float In6_adf=0;
float In6_aqf=0;
float Ih6_a=0;


float In6_bdf=0;
float In6_bqf=0;
float Ih6_b=0;


float Ih6_cdf=0;
float Ih6_cqf=0;
float Ih6_c=0;

float I_a_end=0;
float I_b_end=0;
float I_c_end=0;


int display_flag=0;

float I1_df=0;
float I1_qf=0;

int T_Flag=0;
float T_cout_U=0;
//***********************扑电网电压涉及到的变量******************************
int BiCount=0; //用二分法计算开根号的计数值
float LineVolSquare=0;  //电网相电压的平方
float BiVari=0;  //用二分法计算开平方时的中间变量
float BiMinInitial=0; //用二分法计算开平方时的最小初始值
float BiMaxInitial=1000; // 用二分法计算开平方时的最大初始值
float alpha=0;   //经3/2变换得到数值的平方
float belta=0;   //经3/2变换得到数值的平方
//**********************end****************************888
float tempIdref = 0;

//*******************零漂补偿****************************
float UAB_Fx[3] = {0};
float UAB_Fy[2] = {0};
float UBC_Fx[3] = {0};
float UBC_Fy[2] = {0};
float UCA_Fx[3] = {0};
float UCA_Fy[2] = {0};

float IA_Fx[3] = {0};
float IA_Fy[2] = {0};
float IB_Fx[3] = {0};
float IB_Fy[2] = {0};
float IC_Fx[3] = {0};
float IC_Fy[2] = {0};

float IAF_Fx[3] = {0};
float IAF_Fy[2] = {0};
float IBF_Fx[3] = {0};
float IBF_Fy[2] = {0};
float ICF_Fx[3] = {0};
float ICF_Fy[2] = {0};
float I0F_Fx[3] = {0};
float I0F_Fy[2] = {0};

float UAB_Sum = 0;
float UBC_Sum = 0;
float UCA_Sum = 0;
float IA_Sum = 0;
float IB_Sum = 0;
float IC_Sum = 0;
float IAF_Sum = 0;
float IBF_Sum = 0;
float ICF_Sum = 0;
float I0F_Sum = 0;

//float UA_Z = 0;
//float UB_Z = 0;
//float UC_Z = 0;
//float IA_Z = 0;
//float IB_Z = 0;
//float IC_Z = 0;
//float IAF_Z = 0;
//float IBF_Z = 0;
//float ICF_Z = 0;
//float I0F_Z = 0;

unsigned CountCmpar2 = 0;
unsigned CountCmpar1 = 0;

//**************************APF参数数据库**************************
struct SystemPara 
{
   const unsigned int  DramAddr;//参数在Dram中的地址
   const unsigned int  Rom_P_Addr;//参数在E2PROM的页地址
   const unsigned int  Rom_O_Addr;//参数在E2PROM的页内偏移地址
   const unsigned int  ScreenAddr;//参数在触摸屏上的地址
   const int  FactoryDefault;//参数出厂值
   const int  Max;//参数的最大值
   const int  Min;//参数的最小值           
};
struct SystemPara  UDCREF_Ref = {1,4,0x10,150,650,600,800}; //声明UDCREF_Ref为结构体类型

//***********************APF参数数据库结束***************************

//*********************EEPROM相关***************************
int Factory_Flag = 0;//出厂标志
int EEPROM_Compare = 0;//判断是否将参数写入EEPROM
//*******************EEPROM相关结束*************************

//*************将ABC坐标系下的三个PI改为pq坐标系下的两个PI所用到的参数****
float PI_alpha = 0;//alpha轴PI调节器输出
float PI_beta = 0;//beta轴PI调节器输出
float Udc_d = 0;//稳压指令d轴分量
float Udc_q = 0;//稳压指令q轴分量
float Id_apf = 0; //APF输出电流反馈量的d坐标分量
float Iq_apf = 0; //APF输出电流反馈量的q坐标分量
float Ih_d_c = 0; //dq坐标系下谐波补偿指令,d轴
float Ih_q_c = 0; //dq坐标系下谐波补偿指令,q轴
float Ih_d_c_new = 0; //补偿电流d轴当前指令
float Ih_q_c_new = 0; //补偿电流q轴当前指令
//float I_H_d_c[361] = {0}; //用于存储以前周期的dq轴的补偿指令
//float I_H_q_c[361] = {0};
int   NewCount = 0; //当前抽取的采样点的标号
float Us_d = 0; //电网相电压的d轴分量
float Us_q = 0; //电网相电压的q轴分量
float ModuComdA = 0;//用于载波的调制波（A相）
float ModuComdB = 0;//用于载波的调制波（B相）
float ModuComdC = 0;//用于载波的调制波（C相）
float ModuComd0 = 0;//用于载波的调制波（0相）
float ModuComdA1 = 0; //用于重复控制输出
float ModuComdB1 = 0;
float ModuComdC1 = 0;
//******************************结束***************************************

//************reboot 自动重启相关参数**************
unsigned int lack_phase_check=0;//用于缺相检测
float cycle_point=250;//每个电压周期的采样点数
float tempp3=0;//缺相检测用到的变量
float Rebt_cout_Add=0;
float Rebt_cout_Add1=0;//测试用
int Rebt_Cout=0;  //
int Reboot_flag=0;//控制在每次上电后，重启模式下只对模拟板复位一次
int Ltime_Lack_Phase=0;//记录上次缺相信息
int Lack_Phase=0;//当次缺相信息
int L_Reboot_Mode_Set=0;//EEPROM中记录的上次的重启设定信息
int Reboot_Mode_Set=0;//用于设置自动运行模式
float Delay_Rebt_Add=0;
float Delay_Rebt_Add1=0;
int WR_Error_Count=0;
int Reset_Count=0;//用于判断电网是否完全掉电，完全掉电情况下控制在自启动模式下仅复位一次模拟板
float Fault_flag0 = 0;
float Fault_flag = 0;
float L_Fault_flag = 0;
int Fault_Count = 0;//记录电网出现不完全掉电的次数
float Delay_Rebt_Add2 = 0;
float Delay_Rebt_Add3 = 0;
int Reboot_Run = 0;//用于程序中与
int Reboot_Run_Lamp = 0;
int Initial_Reset = 0;//控制上电时对模拟板复位一次

int Reboot_Count_Down=0;
int PowerFailRecord = 0;//当前电网故障记录
int L_PowerFailRecord = 0;//前一次电网故障记录
int FaultCtrlVari = 0;  //FaultCtrlVari用于作为控制变量，目的是方便在主循环中判断电网故障
int PF_Flag = 0;
//**************************end*********************************//

//************************重复控制器中用到的变量*******************
float alpha_New_error = 0; // alpha,beta坐标系下电流环的当前误差
float beta_New_error = 0;

int Repiti_Count = 0; //重复控制器中一个周期内的当前采样点数
int Repiti_lead = 3; //重复控制器的超前点数
int CycleErrorHit = 0;
float Qz = 0.95; //重复控制的Q(z)部分
float Kr = 0.6;  //重复控制器补偿器部分参数
float Repiti_alpha_CycleOut[251] = {0}; //alpha,beta坐标系下重复控制器上个周期的输出值
float Repiti_beta_CycleOut[251] = {0};
float alpha_Cycle_error[251] = {0}; //alpha,beta坐标系下重复控制器上个周期的误差值
float beta_Cycle_error[251] = {0};
float Repiti_alpha_NewOut = 0; //alpha,beta坐标系下重复控制器的输出值
float Repiti_beta_NewOut = 0;
int Repiti_Sign = 0;//重复控制标志位
//******************************结束*******************************

//***********************电网电压显示值计算用到的变量*****************
float UA_Display = 0;
float UB_Display = 0;
float UC_Display = 0;
//************************结束****************************************

//**************计算电网电压显示用到的变量****************************
float DisplayLS1 = 0;
float DisplayLS2 = 0;
//*********************结束*******************************************


//************************直流侧电压给定指令时间计数***********
int UDC_Count = 0;
//************************结束**********************************


//************************Filter_All****************************
float Filter_All_X1[3] = {0};
float Filter_All_X2[3] = {0};
float Filter_All_X3[3] = {0};
float Filter_All_X4[3] = {0};
float Filter_All_X5[3] = {0};
float Filter_All_X6[3] = {0};
float Filter_All_Y1[3] = {0};
float Filter_All_Y2[3] = {0};
float Filter_All_Y3[3] = {0};
float Filter_All_Y4[3] = {0};
float Filter_All_Y5[3] = {0};
float Filter_All_Y6[3] = {0};
//***************************END********************************

//************************测试***********************
float Dly_Flag = 0.0;
float Dly_Para = 0.0;
float Dly_Test = 1.0;
//**************************结束******************************

//***************分次补偿在alpha和beta坐标系下做PI调节器用到的变量********
float I_apf_alpha = 0;
float I_apf_beta = 0;
float I_apf_0 = 0;
float Udc_alpha = 0;
float Udc_beta = 0;
float Comd_alpha = 0;
float Comd_beta = 0;
float Comd_0 = 0;
//************************结束***************************

//********************计算功率因数*************************
float Power_Factor = 99.0;//功率因数
float Factor = 0.0; 
float I_fundmental = 0.0;
float I_lade_d = 1.0; //用于计算负载功率因数
float I_lade_q = 1.0;
float I_lade_r = 1.0;
float tan_theta[];

// ********************end*************************
