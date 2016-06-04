/* =================================================================================
File name:        RAMP.H  
===================================================================================*/


#ifndef __RAMP_H__
#define __RAMP_H__

typedef struct { float    TargetValue; 	// Input: Target input
				 float    Step;
				 float    Out;	// Output: Target output				 
				 float	Tmp;			// Variable: Temp variable
		  	   } RAMP;	            


/*-----------------------------------------------------------------------------
Default initalizer for the RMPCNTL object.
-----------------------------------------------------------------------------*/                     
#define RAMP_DEFAULTS    {  0, 		 \
                            0,		 \
                            0,       \
                            0,       \
                   		  }

/*------------------------------------------------------------------------------
 	RAMP Controller Macro Definition
------------------------------------------------------------------------------*/

#define RAMP_MACRO(v)					\
	if(v.Out < v.TargetValue)			\
	{									\
		v.Tmp = v.Out + v.Step;			\
		if(v.Tmp > v.TargetValue)			\
			v.Out = v.TargetValue;		\
		else							\
			v.Out = v.Tmp;				\
	}									\
	else if(v.Out > v.TargetValue)		\
	{									\
		v.Tmp = v.Out - v.Step;			\
		if(v.Tmp < v.TargetValue)			\
			v.Out = v.TargetValue;		\
		else							\
			v.Out = v.Tmp;				\
	}																					


#endif // __RAMP_H__
