#ifndef Variables_H_
#define Variables_H_


typedef struct tagCOMREG_PARA {
	unsigned char		Runcode;
	unsigned char		Mode;
	Uint16		FreRef;
	Uint16		I_Kp;
	Uint16		I_Ki;
	int16		Id_Ref;
	int16		Iq_Ref;
	Uint16		Omiga_Kp;
	Uint16		Omiga_Ki;
	int16		Omiga_Ref;
	Uint16		Omiga_Max;
	int16		Pos_Differ;
	Uint16		UDC_Prtct;
	Uint16		I_Prtct;
	Uint16		Omiga_Prtct;
	Uint16		Temp_Warning;
	Uint16		Temp_Prtct;
	Uint16		FaultCode;
	Uint16		switch_1;		//Ô¤Áô
	Uint16		switch_2;		//Ô¤Áô
	Uint16		switch_3;		//Ô¤Áô
	Uint16		switch_4;		//Ô¤Áô
	Uint16		Res_5;		//Ô¤Áô
	Uint16		Res_6;		//Ô¤Áô
	Uint16		Res_7;		//Ô¤Áô
	Uint16		Res_8;		//Ô¤Áô
	Uint16		Res_9;		//Ô¤Áô
	Uint16		Res_10;		//Ô¤Áô
} COMREG_PARA;

#define COMReg_Para_DEFAULT{  0, \
						  	2, \
						  	50, \
						  	1000, \
						  	500, \
							0, \
							0, \
						  	300, \
							100,\
							0,\
							0,\
							0,\
							650,\
							860,\
							4000,\
							75,\
							95,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
							0,\
						   }

extern COMREG_PARA	COMReg_Para;

typedef struct{
	Uint16	MI20;
	Uint16	MI15;
	Uint16	MI10;
	Uint16	MI5;
	Uint16	ZERO;
	Uint16	PO5;
	Uint16	PO10;
	Uint16	PO15;
	Uint16	PO20;
	Uint16	PO25;
	Uint16	PO30;
	Uint16	PO35;
	Uint16	PO40;
	Uint16	PO45;
	Uint16	PO50;
	Uint16	PO55;
	Uint16	PO60;
	Uint16	PO65;
	Uint16	PO70;
	Uint16	PO75;
	Uint16	PO80;
	Uint16	PO85;
	Uint16	PO90;
	Uint16	PO95;
	Uint16	PO100;
	Uint16	PO105;
	Uint16	PO110;
	Uint16	PO115;
	Uint16	PO120;
	Uint16	PO125;
	Uint16	PO130;
	Uint16	PO135;
	Uint16	PO140;
	Uint16	PO145;
	Uint16	PO150;
} TEMPCUR_REG;

#define TEMPCURVE_DEFAULT{	1017,\
						  	1120,\
						  	1239,\
						  	1374,\
						  	1522,\
							1683,\
							1852,\
						  	2026,\
							2201,\
							2375,\
							2542,\
							2702,\
							2852,\
							2991,\
							3118,\
							3233,\
							3336,\
							3428,\
							3511,\
							3581,\
							3648,\
							3704,\
							3754,\
							3798,\
							3837,\
							3871,\
							3901,\
							3927,\
							3950,\
							3971,\
							3989,\
							4005,\
							4020,\
							4033,\
							4044,\
							}

extern TEMPCUR_REG	Tempcurve;

typedef struct{
	int16 I_V_DSP;
	int16 I_U_DSP;
	int16 UV_2_DSP;
	int16 VW_2_DSP;
	int16 Udc_DSP;
	int16 IpA_DSP;
	int16 IpB_DSP;
	Uint16	Vref2V048;
	Uint16	AGND;
	int16	I_V_DSP0;
	int16	I_U_DSP0;
	int16   UV_2_DSP0;
	int16   VW_2_DSP0;
	int16	Udc_DSP0;
	int16   IpA_DSP0;
	int16   IpB_DSP0;
	Uint16	ADtimes;
}ADResult_REG;

#define ADRESULT_DEFAULT{0,\
						0,\
						0,\
						0,\
						0,\
						0,\
						0,\
						0,\
						0,\
						2048,\
						2048,\
						2048,\
						2048,\
						0,\
						2048,\
						2048,\
						0,\
						}
extern ADResult_REG ADResult;

#endif /*Variables_H_*/

//===========================================================================
// End of file.
//===========================================================================
