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
//********************µçÑ¹»·ÓÃµ½µÄ±äÁ¿**************************
int     Reset=0;
float   CS=0;
float   x1=1;//ÁãÆ¯
float   y1=1;//·ùÖµ
float   x2=1;
//float   y2=1;
float   A1=0;
float   A3=0;
float   A5=0;
float   A7=0;
float   A9=0;
float   A11=0;
float   A13=0;
float   A15=0;
float   A17=0;
float   A19=0;
float   A21=0;
float   A23=0;
float   A25=0;
int     graph=2;
int     FFT_Flag=0;
int     FFT=1;
int 	UDCADRF=0;             //Ö±Á÷²àµçÑ¹²¹³¥ÁãÆ¯ºóÖµ
float  	UDC_R=0;            //¶¨±êºó¶ÔÓ¦µÄÊµ¼ÊµçÑ¹Öµ
float  	UDC_Rf1=140;
float	UDC_Rf2=140;
			
float	UDC_Kp=200;
float	UDC_Ki=50;
float   I_Kp=8;
float   I_Ki=0;
float	UDC_erro_old=0;
float	UDC_erro_new=0;
float	Udc_slip=140;
float	Id_slip=0; 
float	Idref=0;		//Udc_PIµ÷½ÚÆ÷³öÀ´µÄId_ref
float   Inref=0;        //Udc»ı·ÖÏîÏŞ·ù
float   T_cout=0;
//float   Error_IA=0;
//float   Error_IB=0;
//float   Error_IC=0;
//********************ÏàÎ»¼ÆËãÓÃµ½µÄ±äÁ¿*************************
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

float	U1_a=0;		//»ù²¨µçÑ¹
float	U1_b=0;
float	U1_c=0;

float   Ua_APF=0;
float   Ub_APF=0;
float   Uc_APF=0; 
float   U0_APF = 0;

float	LS1=0;		//ÈıÏà¾²Ö¹×ø±êÏÂµÄÁÙÊ±´æ´¢±äÁ¿
float	LS2=0;
float	LS3=0;

float I_temp1=0;    //½öÓÃÒ»´Î
float I_temp2=0;
float temp3=0;
float temp4=0;

float Ih_a=0;       //¸÷ÏàµçÁ÷Öµ
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
float If_0=0;
float In_d=0;


float I1d[3] = {0};
float I1df[2] = {0};
float I1q[3] = {0};
float I1qf[2] = {0};
float IA_d[30]={0};
float IA_q[30]={0};
float IB_d[30]={0};
float IB_q[30]={0};
float IC_d[30]={0};
float IC_q[30]={0};
float IA3=0;
float IA5=0;
float IA7=0;
float IA9=0;
float IA11=0;
float IA13=0;
float IA15=0;
float IA17_25=0;
float IB3=0;
float IB5=0;
float IB7=0;
float IB9=0;
float IB11=0;
float IB13=0;
float IB15=0;
float IB17_25=0;
float IC3=0;
float IC5=0;
float IC7=0;
float IC9=0;
float IC11=0;
float IC13=0;
float IC15=0;
float IC17_25=0;
float I03=0;
float I05=0;
float I07=0;
float I09=0;
float I011=0;
float I013=0;
float I015=0;
float IAh_Vector=0;
float IBh_Vector=0;
float ICh_Vector=0;
float I0h_Vector=0;
int flag_0=0;
float IA1=0;
float IB1=0;
float IC1=0;
float IA_Vector=0;
float IB_Vector=0;
float IC_Vector=0;
float I0_Vector=0;
float I_Limit=0;
float I01_d=0;
float I01_q=0;
float I01=0;
//*******************È«²¹³¥²¿·ÖËùĞèÊı×é½áÊø*******************************


float reactive_filter_buffer[3][4]={0};

	//**************·Ö´Î²¹³¥ËùÓÃ±äÁ¿**********************(ÈıÏàËÄÏß)
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
	float IAh_Sum1 = 0;
	float IBh_Sum1 = 0;
	float ICh_Sum1 = 0;
	float IAh_Part_Sum = 0;
	float IBh_Part_Sum = 0;
	float ICh_Part_Sum = 0;

	float FilterPara1[6] = {0}; //·Ö´Î²¹³¥ÂË²¨Êä³öÖµ¼ÇÂ¼Êı×é
	float FilterPara2[6] = {0};
//	float FilterPara3[6] = {0};
//	float FilterPara4[6] = {0};
//	float FilterPara5[6] = {0};

//***********·Ö´Î²¹³¥ÂË²¨Ê±£¬¼ÇÂ¼ÂË²¨ÊäÈëÖµ£¨µ±Ç°Ò»¸öºÍÇ°2¸ö£©ºÍÂË²¨Êä³öÖµ£¨Ç°2¸ö£©***
//float In1_d[3] = {0};
//float In1_df[2] = {0};
//float In1_q[3] = {0};
//float In1_qf[2] = {0};

//float In2_d[3] = {0};
//float In2_df[2] = {0};
//float In2_q[3] = {0};
//float In2_qf[2] = {0};

//float In3_d[3] = {0};
//float In3_df[2] = {0};
//float In3_q[3] = {0};
//float In3_qf[2] = {0};

//float In4_d[3] = {0};
//float In4_df[2] = {0};
//float In4_q[3] = {0};
//float In4_qf[2] = {0};
//***********************·Ö´Î²¹³¥ÂË²¨²¿·ÖËùĞèÊı×é½áÊø***************

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

float I1_a=0; //Á½Ïà¾²Ö¹×ø±êÏµÏÂµÄÁÙÊ±±äÁ¿
float I1_b=0; 
float I1_adf=0;
float I1_aqf=0;
float I1_bdf=0;
float I1_bqf=0; 
float I1_cdf=0;
float I1_cqf=0;
float I1_d=0;
float I1_q=0;



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
int n=0;    	//ÔÚdelay_tÖĞÓĞÊ¹ÓÃ
int i=0;*/

//float graph_data[300]={0};
int ReceivedChar=0;
//float graph_data1[300]={0};
//float graph_data2[300]={0};
float graph_data[500]={0};
float graph_data1[500]={0};
float graph_data2[500]= {0}; 




float UDC_Max = 0;
float Ih_Max = 0;
float U_FtoI = 0;
float UDCREF = 0;
float Ih_Limit = 0;		//ÏŞÁ÷Êä³öÏŞ·ùÖµ
//int Limit_Flag = 0.0;		//ÏŞÁ÷ÔËĞĞ±ê¼ÇÎ»
//float If_Limit = 0.0;		//ÎŞ¹¦ÏŞÁ÷ÔËĞĞÏŞ·ù
//float If_alpha = 0.0;		//ÎŞ¹¦Ö¸ÁîÖµ
//float If_beta = 0.0;
float Ih_d_Limit=0;
float Ih_q_Limit=0;
float I1_d_square=0;
float I1_q_square=0;
float I1_Square = 0.0;
float Ih_Limit_square = 0.0;
float I1_Value = 0.0;
float Phi_Limit = 0.0;



//Í¨ĞÅÏà¹Ø
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
int C_way=0;	//152  .1=1È«²¹¡£1=0·Ö´Î²¹  ¡£3=1 ²¹³¥»ù²¨ÎŞ¹¦
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
int COM_UAB=0; 	//µçÍøµçÑ¹a  
int COM_UBC=0;	//µçÍøµçÑ¹b
int COM_UCA=0;	//µçÍøµçÑ¹c
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
//********************THDÖµÏÔÊ¾ËùÓÃ±äÁ¿*************************
int  THD_IA=0;
int  THD_IB=0;
int  THD_IC=0;
int  THD_I0=0;
int  THD_IA3=0;
int  THD_IA5=0;
int  THD_IA7=0;
int  THD_IA9=0;
int  THD_IA11=0;
int CompensateA= 0;//Êä³ö²¹³¥µçÁ÷ÓĞĞ§Öµ
int CompensateB= 0;
int CompensateC= 0;
//********************µçÁ÷ÖµÏÔÊ¾ùÓÃ±äÁ¿************************* 


int m=0;        //
int n=0;    	//ÔÚdelay_tÖĞÓĞÊ¹ÓÃ
int i=0;

int display_flag=0;

float I1_df=0;
float I1_qf=0;

int T_Flag=0;
float T_cout_U=0;
//***********************ÆËµçÍøµçÑ¹Éæ¼°µ½µÄ±äÁ¿******************************
int BiCount=0; //ÓÃ¶ş·Ö·¨¼ÆËã¿ª¸ùºÅµÄ¼ÆÊıÖµ
float LineVolSquare=0;  //µçÍøÏàµçÑ¹µÄÆ½·½
float BiVari=0;  //ÓÃ¶ş·Ö·¨¼ÆËã¿ªÆ½·½Ê±µÄĞ¼ä±äÁ¿
float BiMinInitial=0; //ÓÃ¶ş·Ö·¨¼ÆËã¿ªÆ½·½Ê±µÄ×îĞ¡³õÊ¼Öµ
float BiMaxInitial=1000; // ÓÃ¶ş·Ö·¨¼ÆËã¿ªÆ½·½Ê±µÄ×î´ó³õÊ¼Öµ
float alpha=0;   //¾­3/2±ä»»µÃµ½ÊıÖµµÄÆ½·½
float belta=0;   //¾­3/2±ä»»µÃµ½ÊıÖµµÄÆ½·½
//**********************end****************************888
float tempIdref = 0;

//*******************ÁãÆ¯²¹³¥****************************
/*float UAB_Fx[3] = {0};
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
*/
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

//**************************APF²ÎÊıÊı¾İ¿â**************************
struct SystemPara 
{
   const unsigned int  DramAddr;//²ÎÊıÔÚDramÖĞµÄµØÖ·
   const unsigned int  Rom_P_Addr;//²ÎÊıÔÚE2PROMµÄÒ³µØÖ·
   const unsigned int  Rom_O_Addr;//²ÎÊıÔÚE2PROMµÄÒ³ÄÚÆ«ÒÆµØÖ·
   const unsigned int  ScreenAddr;//²ÎÊıÔÚ´¥ÃşÆÁÉÏµÄµØÖ·
   const int  FactoryDefault;//²ÎÊı³ö³§Öµ
   const int  Max;//²ÎÊıµÄ×î´óÖµ
   const int  Min;//²ÎÊıµÄ×îĞ¡Öµ           
};
struct SystemPara  UDCREF_Ref = {1,4,0x10,150,650,600,800}; //ÉùÃ÷UDCREF_RefÎª½á¹¹ÌåÀàĞÍ

//***********************APF²ÎÊıÊı¾İ¿â½áÊø***************************

//*********************EEPROMÏà¹Ø***************************
int Factory_Flag = 0;//³ö³§±êÖ¾
int EEPROM_Compare = 0;//ÅĞ¶ÏÊÇ·ñ½«²ÎÊıĞ´ÈëEEPROM
//*******************EEPROMÏà¹Ø½áÊø*************************

//*************½«ABC×ø±êÏµÏÂµÄÈı¸öPI¸ÄÎªpq×ø±êÏµÏÂµÄÁ½¸öPIËùÓÃµ½µÄ²ÎÊı****
float PI_alpha = 0;//alphaÖáPIµ÷½ÚÆ÷Êä³ö
float PI_beta = 0;//betaÖáPIµ÷½ÚÆ÷Êä³ö
/*float Udc_d = 0;//ÎÈÑ¹Ö¸ÁîdÖá·ÖÁ¿
float Udc_q = 0;//ÎÈÑ¹Ö¸ÁîqÖá·ÖÁ¿
float Id_apf = 0; //APFÊä³öµçÁ÷·´À¡Á¿µÄd×ø±ê·ÖÁ¿
float Iq_apf = 0; //APFÊä³öµçÁ÷·´À¡Á¿µÄq×ø±ê·ÖÁ¿
float Ih_d_c = 0; //dq×ø±êÏµÏÂĞ³²¨²¹³¥Ö¸Áî,dÖá
float Ih_q_c = 0; //dq×ø±êÏµÏÂĞ³²¨²¹³¥Ö¸Áî,qÖá
float Ih_d_c_new = 0; //²¹³¥µçÁ÷dÖáµ±Ç°Ö¸Áî
float Ih_q_c_new = 0; //²¹³¥µçÁ÷qÖáµ±Ç°Ö¸Áî
*/
//float I_H_d_c[361] = {0}; //ÓÃÓÚ´æ´¢ÒÔÇ°ÖÜÆÚµÄdqÖáµÄ²¹³¥Ö¸Áî
//float I_H_q_c[361] = {0};
int   NewCount = 0; //µ±Ç°³éÈ¡µÄ²ÉÑùµãµÄ±êºÅ
float Us_d = 0; //µçÍøÏàµçÑ¹µÄdÖá·ÖÁ¿
float Us_q = 0; //µçÍøÏàµçÑ¹µÄqÖá·ÖÁ¿
float ModuComdA = 0;//ÓÃÓÚÔØ²¨µÄµ÷ÖÆ²¨£¨AÏà£©
float ModuComdB = 0;//ÓÃÓÚÔØ²¨µÄµ÷ÖÆ²¨£¨BÏà£©
float ModuComdC = 0;//ÓÃÓÚÔØ²¨µÄµ÷ÖÆ²¨£¨CÏà£©
float ModuComd0 = 0;//ÓÃÓÚÔØ²¨µÄµ÷ÖÆ²¨£¨0Ïà£©
float ModuComdA1 = 0; //ÓÃÓÚÖØ¸´¿ØÖÆÊä³ö
float ModuComdB1 = 0;
float ModuComdC1 = 0;
//******************************½áÊø***************************************

//************reboot ×Ô¶¯ÖØÆôÏà¹Ø²ÎÊı**************
unsigned int lack_phase_check=0;//ÓÃÓÚÈ±Ïà¼ì²â
float cycle_point=250;//Ã¿¸öµçÑ¹ÖÜÆÚµÄ²ÉÑùµãÊı
float tempp3=0;//È±Ïà¼ì²âÓÃµ½µÄ±äÁ¿
float Rebt_cout_Add=0;
float Rebt_cout_Add1=0;//²âÊÔÓÃ
int Rebt_Cout=0;  //
int Reboot_flag=0;//¿ØÖÆÔÚÃ¿´ÎÉÏµçºó£¬ÖØÆôÄ£Ê½ÏÂÖ»¶ÔÄ£Äâ°å¸´Î»Ò»´Î
int Ltime_Lack_Phase=0;//¼ÇÂ¼ÉÏ´ÎÈ±ÏàĞÅÏ¢
int Lack_Phase=0;//µ±´ÎÈ±ÏàĞÅÏ¢
int L_Reboot_Mode_Set=0;//EEPROMÖĞ¼ÇÂ¼µÄÉÏ´ÎµÄÖØÆôÉè¶¨ĞÅÏ¢
int Reboot_Mode_Set=0;//ÓÃÓÚÉèÖÃ×Ô¶¯ÔËĞĞÄ£Ê½
float Delay_Rebt_Add=0;
float Delay_Rebt_Add1=0;
int WR_Error_Count=0;
int Reset_Count=0;//ÓÃÓÚÅĞ¶ÏµçÍøÊÇ·ñÍêÈ«µôµç£¬ÍêÈ«µôµçÇé¿öÏÂ¿ØÖÆÔÚ×ÔÆô¶¯Ä£Ê½ÏÂ½ö¸´Î»Ò»´ÎÄ£Äâ°å
float Fault_flag0 = 0;
float Fault_flag = 0;
float L_Fault_flag = 0;
int Fault_Count = 0;//¼ÇÂ¼µçÍø³öÏÖ²»ÍêÈ«µôµçµÄ´ÎÊı
float Delay_Rebt_Add2 = 0;
float Delay_Rebt_Add3 = 0;
int Reboot_Run = 0;//ÓÃÓÚ³ÌĞòÖĞÓë
int Reboot_Run_Lamp = 0;
int Initial_Reset = 0;//¿ØÖÆÉÏµçÊ±¶ÔÄ£Äâ°å¸´Î»Ò»´Î

int Reboot_Count_Down=0;
int PowerFailRecord = 0;//µ±Ç°µçÍø¹ÊÕÏ¼ÇÂ¼
int L_PowerFailRecord = 0;//Ç°Ò»´ÎµçÍø¹ÊÕÏ¼ÇÂ¼
int FaultCtrlVari = 0;  //FaultCtrlVariÓÃÓÚ×÷Îª¿ØÖÆ±äÁ¿£¬Ä¿µÄÊÇ·½±ãÔÚÖ÷Ñ­»·ÖĞÅĞ¶ÏµçÍø¹ÊÕÏ
//int PF_Flag = 0;
//**************************end*********************************//

//************************ÖØ¸´¿ØÖÆÆ÷ÖĞÓÃµ½µÄ±äÁ¿*******************
//float alpha_New_error = 0; // alpha,beta×ø±êÏµÏÂµçÁ÷»·µÄµ±Ç°Îó²î
//float beta_New_error = 0;

//int Repiti_Count = 0; //ÖØ¸´¿ØÖÆÆ÷ÖĞÒ»¸öÖÜÆÚÄÚµÄµ±Ç°²ÉÑùµãÊı
//int Repiti_lead = 3; //ÖØ¸´¿ØÖÆÆ÷µÄ³¬Ç°µãÊı
//int CycleErrorHit = 0;
//float Qz = 0.95; //ÖØ¸´¿ØÖÆµÄQ(z)²¿·Ö
//float Kr = 0.6;  //ÖØ¸´¿ØÖÆÆ÷²¹³¥Æ÷²¿·Ö²ÎÊı
//float Repiti_alpha_CycleOut[251] = {0}; //alpha,beta×ø±êÏµÏÂÖØ¸´¿ØÖÆÆ÷ÉÏ¸öÖÜÆÚµÄÊä³öÖµ
//float Repiti_beta_CycleOut[251] = {0};
//float alpha_Cycle_error[251] = {0}; //alpha,beta×ø±êÏµÏÂÖØ¸´¿ØÖÆÆ÷ÉÏ¸öÖÜÆÚµÄÎó²îÖµ
//float beta_Cycle_error[251] = {0};
float Repiti_alpha_NewOut = 0; //alpha,beta×ø±êÏµÏÂÖØ¸´¿ØÖÆÆ÷µÄÊä³öÖµ
float Repiti_beta_NewOut = 0;
int Repiti_Sign = 0;//ÖØ¸´¿ØÖÆ±êÖ¾Î»
//******************************½áÊø*******************************

//***********************µçÍøµçÑ¹ÏÔÊ¾Öµ¼ÆËãÓÃµ½µÄ±äÁ¿*****************
float UA_Display = 0;
float UB_Display = 0;
float UC_Display = 0;
//************************½áÊø****************************************

//**************¼ÆËãµçÍøµçÑ¹ÏÔÊ¾ÓÃµ½µÄ±äÁ¿****************************
float DisplayLS1 = 0;
float DisplayLS2 = 0;
//*********************½áÊø*******************************************


//************************Ö±Á÷²àµçÑ¹¸ø¶¨Ö¸ÁîÊ±¼ä¼ÆÊı***********
int UDC_Count = 0;
//************************½áÊø**********************************


//************************Filter_All****************************
/*float Filter_All_X1[3] = {0};
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
*/
//***************************END********************************

//**************************½áÊø******************************

//***************·Ö´Î²¹³¥ÔÚalphaºÍbeta×ø±êÏµÏÂ×öPIµ÷½ÚÆ÷ÓÃµ½µÄ±äÁ¿********
float I_apf_alpha = 0;
float I_apf_beta = 0;
float I_apf_0 = 0;
float Udc_alpha = 0;
float Udc_beta = 0;
float Comd_alpha = 0;
float Comd_beta = 0;
float Comd_0 = 0;
//************************½áÊø***************************

//********************¼ÆËã¹¦ÂÊÒòÊı*************************
float Power_Factor = 99.0;//¹¦ÂÊÒòÊı
float Factor = 0.0; 
float I_fundmental = 0.0;
float I_lade_d = 1.0; //ÓÃÓÚ¼ÆËã¸ºÔØ¹¦ÂÊÒòÊı
float I_lade_q = 1.0;
float I_lade_r = 1.0;
// ********************end*************************
//********************¼ì²âÎŞ¹¦µçÁ÷*************//

float IA_Past[61]={0};
float IB_Past[61]={0};
float IC_Past[61]={0};
int iqflag=0;
		
float IAADRF_Past=0;
float IBADRF_Past=0;
float ICADRF_Past=0;
// ********************end*************************

