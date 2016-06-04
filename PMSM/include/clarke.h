/* =================================================================================
File name:       CLARKE.H  
===================================================================================*/


#ifndef __CLARKE_H__
#define __CLARKE_H__

typedef struct {  float  As;  		// Input: phase-a stator variable
				  float  Bs;			// Input: phase-b stator variable
				  float  Cs;			// Input: phase-c stator variable  
				  float  Alpha;		// Output: stationary d-axis stator variable 
				  float  Beta;		// Output: stationary q-axis stator variable
		 	 	} CLARKE;	            

/*-----------------------------------------------------------------------------
	Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define CLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0, \
              			} 

/*------------------------------------------------------------------------------
	CLARKE Transformation Macro Definition
------------------------------------------------------------------------------*/

//  1/sqrt(3) = 0.57735026918963
#define  ONEbySQRT3   0.57735026918963    /* 1/sqrt(3) */
	

// Clarke transform macro (with 2 currents)
//==========================================
#define CLARKE_MACRO(v)										\
v.Alpha = v.As;												\
v.Beta = (v.As +2*v.Bs)*ONEbySQRT3;


// Clarke transform macro (with 3 currents)
//==========================================
#define CLARKE1_MACRO(v)									\
v.Alpha = v.As;											    \
v.Beta  = (v.Bs - v.Cs)*ONEbySQRT3;

#endif // __CLARKE_H__

