#ifndef Parameters_H_
#define Parameters_H_

/*------------------------------------------------------------------------------
 Following is the list of the Build Level choices.
 ------------------------------------------------------------------------------*/

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI_CONSTANT 3.14159265358979
#define SYSTEM_FREQUENCY 150
// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10

// Define the electrical motor parametes (Estun Servomotor)
#define RS 		2.35		    	    // Stator resistance (ohm) 
#define RR   			               	// Rotor resistance (ohm) 
#define LS   	0.0065					// Stator inductance (H) 
#define LR   			  				// Rotor inductance (H) 	
#define LM   			   				// Magnatizing inductance (H)
#define POLE_PAIR  	4						// Number of pole pairs

// Define the base quantites
#define BASE_VOLTAGE    346.4        // Base peak phase voltage (volt), Vdc/sqrt(3)
#define BASE_CURRENT    890            // Base peak phase current (amp), Max. measurable peak curr.
#define BASE_TORQUE     		      // Base torque (N.m)
#define BASE_FLUX       			  // Base flux linkage (volt.sec/rad)
#define BASE_FREQ      	200           // Base electrical frequency (Hz) 
#define BASE_OMIGA		(60*BASE_FREQ/POLE_PAIR)
#define	Pulse_Num		4096
#define Max_Pulse       4095
#define Omiga_COFF		0.048828125		//1转 对应 4096	单位为rpm(定标后) 1s*60/50ms/(3*4096)  //Pulse_num                                                                 
#define	I_ABC_COFF		0.012523089446166	//+-25.64A 最大值对应+-2047.5+2047.5
#define	U_UDC_COFF		0.192354653893115	//

#define		PWM_STAT_NOFAULT		0x00
#define 	PWM_STAT_STOP			0X01
#define		PWM_STAT_OVERCUR		0x02
#define 	PWM_STAT_OVERVOL		0x04
#define		PWM_STAT_TEMPHI			0x08
#define 	PWM_STAT_OVERTEMP		0x0F

#endif /*Parameters_H_*/

//===========================================================================
// End of file.
//===========================================================================
