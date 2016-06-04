/* =================================================================================
File name:       IPARK.H   
===================================================================================*/

#ifndef __IPARK_H__
#define __IPARK_H__

typedef struct {  float  Alpha;  		// Output: stationary d-axis stator variable
				  float  Beta;		// Output: stationary q-axis stator variable
				  float  Angle;		// Input: rotating angle (pu)
				  float  Ds;			// Input: rotating d-axis stator variable
				  float  Qs;			// Input: rotating q-axis stator variable
				  float  Sine;		// Input: Sine term
				  float  Cosine;		// Input: Cosine term
		 	    } IPARK;	            

/*-----------------------------------------------------------------------------
Default initalizer for the IPARK object.
-----------------------------------------------------------------------------*/                     
#define IPARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
						  0, \
                          0, \
              		   }

/*------------------------------------------------------------------------------
	Inverse PARK Transformation Macro Definition
------------------------------------------------------------------------------*/


#define IPARK_MACRO(v)										\
															\
v.Alpha = v.Ds*v.Cosine - v.Qs*v.Sine;		\
v.Beta  = v.Qs*v.Cosine + v.Ds*v.Sine;

#endif // __IPARK_H__

