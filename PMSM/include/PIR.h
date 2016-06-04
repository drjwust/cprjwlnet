/* =================================================================================
File name:       PIR.H 
===================================================================================*/


#ifndef __PIR_H__
#define __PIR_H__

typedef struct {  float  Ref;   			// Input: reference set-point
				  float  Fbk;   			// Input: feedback
				  float  Out;   			// Output: controller output 
				  float  Kp;				// Parameter: proportional loop gain
				  float  Ki;			    // Parameter: integral gain
				  float  Kr;
				  float  out_1;				// Data: previous output
				  float  out_2;				// Data: previous previous output
				  float  out_3;				//
				  float  error;
				  float  error_1;			//Data:previous error
				  float  error_2;
				  float  error_3;
				} PIR_CONTROLLER;


/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/                     


#define CURRENTPIR_CONTROLLER_DEFAULTS {		\
						   0, 			\
                           0, 			\
						   0, 			\
			   			   5,\
                           500,	\
                           100,	\
                           0.0,	\
                           0.0, \
                           0.0,	\
                           0.0, 	\
                           0.0,	\
                           0.0,	\
			   0.0,\
              			  }
/*------------------------------------------------------------------------------
	PIR_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PIR_MACRO(v)												\
																\
	/* error term */ 									\
	v.error = v.Ref - v.Fbk;						\
	v.Out=1/(1+w0*w0*Ts*Ts)*((3+w0*w0*Ts*Ts)*v.out_1-3*v.out_2+v.out_3+\
	  (v.Kp+v.Kp*w0*w0*Ts*Ts+Ts*v.Kr+v.Ki*Ts+v.Ki*w0*w0*Ts*Ts*Ts)*v.error\
	  -(3*v.Kp+v.Kp*w0*w0*Ts*Ts+2*Ts*v.Kr+2*Ts*v.Ki)*v.error_1+(3*v.Kp+Ts*v.Kr+Ts*v.Ki)*v.error_2-v.Kp*v.error_3);\
v.out_3=v.out_2;\
v.out_2=v.out_1;\
v.out_1=v.Out;\
v.error_3=v.error_2;\
v.error_2=v.error_1;\
v.error_1=v.error;



#endif // __PIR_H__

