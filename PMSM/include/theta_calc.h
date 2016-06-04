/* =================================================================================
File name:        THETA_CALC.H  
===================================================================================*/

#ifndef __THETA_CALC_H__
#define __THETA_CALC_H__
typedef struct { float  Freq; 		// Input: Ramp frequency (pu) 	
		 	     float  StepAngleMax;	// Parameter: Maximum step angle (pu)		
	 	 	     float  Angle;		// Variable: Step angle (pu)					  
			     float  Out;  	 	// Output: Ramp signal (pu) 				 
	  	  	   } THETA;	            

/*------------------------------------------------------------------------------
      Object Initializers
------------------------------------------------------------------------------*/                       
#define THETA_CALC_DEFAULTS {0,		\
						  0,		\
						  0,		\
						  0,		\
                         }

/*------------------------------------------------------------------------------
	THETA_CALC Definition
------------------------------------------------------------------------------*/                                               

#define THETA_CALC(v)								\
	v.Angle += v.StepAngleMax * v.Freq;				\
	if(v.Angle > 1)									\
		v.Angle -= 1;								\
	else if(v.Angle < 0)							\
		v.Angle += 1;								\
	v.Out = v.Angle;

#endif // __THETA_CALC_H__
