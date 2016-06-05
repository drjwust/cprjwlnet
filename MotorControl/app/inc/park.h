/* =================================================================================
File name:       PARK.H 
===================================================================================*/

#ifndef __PARK_H__
#define __PARK_H__

typedef struct {  float  Alpha;  		// Input: stationary d-axis stator variable 
				  float  Beta;	 	// Input: stationary q-axis stator variable 
				  float  Angle;		// Input: rotating angle  
				  float  Ds;			// Output: rotating d-axis stator variable 
				  float  Qs;			// Output: rotating q-axis stator variable
				  float  Sine;
				  float  Cosine; 	 
		 	 	} PARK;	            

/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/                     
#define PARK_DEFAULTS {   0, \
                          0, \
                          0, \
                          0, \
                          0, \
						  0, \
                          0, \
              			  }

/*------------------------------------------------------------------------------
	PARK Transformation Macro Definition
------------------------------------------------------------------------------*/


#define PARK_MACRO(v)											\
																\
	v.Ds = v.Alpha*v.Cosine + v.Beta*v.Sine;	\
    v.Qs = v.Beta*v.Cosine - v.Alpha*v.Sine;

#endif // __PARK_H__
