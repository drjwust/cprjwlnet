/* =================================================================================
File name:       PI.H 
===================================================================================*/


#ifndef __PI_H__
#define __PI_H__

typedef struct {  float  Ref;   			// Input: reference set-point
				  float  Fbk;   			// Input: feedback
				  float  Out;   			// Output: controller output 
				  float  Kp;				// Parameter: proportional loop gain
				  float  Ki;			    // Parameter: integral gain
				  float  Umax;			// Parameter: upper saturation limit
				  float  Umin;			// Parameter: lower saturation limit
				  float  up;				// Data: proportional term
				  float  ui;				// Data: integral term
				  float  v1;				// Data: pre-saturated controller output
				  float  i1;				// Data: integrator storage: ui(k-1)
				} PI_CONTROLLER;


/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/                     

#define OMIGAPI_CONTROLLER_DEFAULTS {		\
						   0, 			\
                           0, 			\
						   0, 			\
                           300,	\
                           100,	\
                           21,	\
                           (-21), \
                           0.0,	\
                           0.0, 	\
                           0.0,	\
                           0.0,	\
              			  }

#define CURRENTPI_CONTROLLER_DEFAULTS {		\
						   0, 			\
                           0, 			\
						   0, 			\
                           1000,	\
                           500,	\
                           1.0,	\
                           (-1.0), \
                           0.0,	\
                           0.0, 	\
                           0.0,	\
                           0.0,	\
              			  }
/*------------------------------------------------------------------------------
	PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PI_MACRO(v)												\
																\
	/* proportional term */ 									\
	v.up = v.Kp*(v.Ref - v.Fbk);						\
																\
	/* integral term */ 										\
	v.ui = (v.Out == v.v1)?(v.Ki*v.up+ v.i1) : v.i1;	\
	v.i1 = v.ui;												\
																\
	/* control output */ 										\
	v.v1 = v.up + v.ui;											\
	if(v.v1>v.Umax) v.Out=v.Umax;								\
	else if(v.v1<v.Umin) v.Out=v.Umin;							\
	else v.Out=v.v1;



#endif // __PI_H__

