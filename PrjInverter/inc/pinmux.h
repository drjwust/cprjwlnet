//*****************************************************************************
//
//  pinmux.h - Created using TI Pinmux  on 2015/12/5 at 上午11:48:58.
//
//*****************************************************************************
//
//  These values will provide the functionality requested when written into
//  the registers for which the #defines are named.  For example, using the
//  controlSUITE device support header files, use the defines like in this
//  sample function:
//
//  void samplePinMuxFxn(void)
//  {
//      EALLOW;
//      //
//      // Write generated values to mux registers
//      //
//      GpioCtrlRegs.GPAMUX1.all  = GPAMUX1_VALUE;
//      GpioCtrlRegs.GPAMUX2.all  = GPAMUX2_VALUE;
//      GpioCtrlRegs.GPBMUX1.all  = GPBMUX1_VALUE;
//        . . .
//      EDIS;
//  }
//
//  NOTE:  These GPIO control registers are only available on CPU1.
//
//*****************************************************************************

//
// Port A mux register values
//
// Pin 162 (GPIO2) to EPWM2A (mode 1)
// Pin 163 (GPIO3) to EPWM2B (mode 1)
// Pin 164 (GPIO4) to EPWM3A (mode 1)
// Pin 165 (GPIO5) to EPWM3B (mode 1)
// Pin 166 (GPIO6) to EPWM4A (mode 1)
// Pin 167 (GPIO7) to EPWM4B (mode 1)
// Pin 1 (GPIO10) to EPWM6A (mode 1)
// Pin 2 (GPIO11) to EPWM6B (mode 1)
// Pin 4 (GPIO12) to EPWM7A (mode 1)
// Pin 5 (GPIO13) to EPWM7B (mode 1)
// Pin 6 (GPIO14) to EPWM8A (mode 1)
// Pin 7 (GPIO15) to EPWM8B (mode 1)
#define GPAMUX1_VALUE		0x55505550
#define GPAMUX2_VALUE		0x00000000
#define GPAGMUX1_VALUE		0x00000000
#define GPAGMUX2_VALUE		0x00000000

//
// Port B mux register values
//
#define GPBMUX1_VALUE		0x00000000
#define GPBMUX2_VALUE		0x00000000
#define GPBGMUX1_VALUE		0x00000000
#define GPBGMUX2_VALUE		0x00000000

//
// Port C mux register values
//
#define GPCMUX1_VALUE		0x00000000
#define GPCMUX2_VALUE		0x00000000
#define GPCGMUX1_VALUE		0x00000000
#define GPCGMUX2_VALUE		0x00000000

//
// Port D mux register values
//
#define GPDMUX1_VALUE		0x00000000
#define GPDMUX2_VALUE		0x00000000
#define GPDGMUX1_VALUE		0x00000000
#define GPDGMUX2_VALUE		0x00000000

//
// Port E mux register values
//
#define GPEMUX1_VALUE		0x00000000
#define GPEMUX2_VALUE		0x00000000
#define GPEGMUX1_VALUE		0x00000000
#define GPEGMUX2_VALUE		0x00000000

//*****************************************************************************
//
// Function prototype for function to write values above into their
// corresponding registers. This function is found in pinmux.c. Its use is
// completely optional.
//
//*****************************************************************************

void GPIO_setPinMuxConfig(void);
