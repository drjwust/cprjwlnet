//###########################################################################
// FILE:   can_loopback_interrupts.c
// TITLE:  Example to demonstrate basic CAN setup and use.
//
//! \addtogroup cpu01_example_list
//! <h1>CAN External Loopback with Interrupts (can_loopback_interrupts)</h1>
//!
//! This example shows the basic setup of CAN in order to transmit and receive
//! messages on the CAN bus.  The CAN peripheral is configured to transmit 
//! messages with a specific CAN ID.  A message is then transmitted once per 
//! second, using a simple delay loop for timing.  The message that is sent is
//! a 4 byte message that contains an incrementing pattern.  A CAN interrupt
//! handler is used to confirm message transmission and count the number of
//! messages that have been sent.
//!
//! This example sets up the CAN controller in External Loopback test mode.
//! Data transmitted is visible on the CAN0TX pin and can be received with
//! an appropriate mailbox configuration.
//!
//! This example uses the following interrupt handlers:\n
//! - INT_CAN0 - CANIntHandler
//!
//
//###########################################################################
// $TI Release: F2837xD Support Library v160 $
// $Release Date: Mon Jun 15 13:36:23 CDT 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include <includes.h>

//*****************************************************************************
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//*****************************************************************************
interrupt void CANIntHandler(void)
{
	unsigned long ulStatus;

	// Read the CAN interrupt status to find the cause of the interrupt
	ulStatus = CANIntStatus(CANA_BASE, CAN_INT_STS_CAUSE);

	// If the cause is a controller status interrupt, then get the status
	if (ulStatus == CAN_INT_INT0ID_STATUS)
	{
		// Read the controller status.  This will return a field of status
		// error bits that can indicate various errors.  Error processing
		// is not done in this example for simplicity.  Refer to the
		// API documentation for details about the error status bits.
		// The act of reading this status will clear the interrupt.  If the
		// CAN peripheral is not connected to a CAN bus with other CAN devices
		// present, then errors will occur and will be indicated in the
		// controller status.
		ulStatus = CANStatusGet(CANA_BASE, CAN_STS_CONTROL);

		//Check to see if an error occurred.
		if (((ulStatus & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 7)
				&& ((ulStatus & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 0))
		{
			// Set a flag to indicate some errors may have occurred.
		}
	}

	// Check if the cause is message object 1, which what we are using for
	// sending messages.
	else if (ulStatus == 1)
	{
		// Getting to this point means that the TX interrupt occurred on
		// message object 1, and the message TX is complete.  Clear the
		// message object interrupt.
		CANIntClear(CANA_BASE, 1);
		// Since the message was sent, clear any error flags.
	}

	// Check if the cause is message object 1, which what we are using for
	// receiving messages.
	else if (ulStatus == 2)
	{

		FlagRxCAN = 1;
		// Get the received message
		CANMessageGet(CANA_BASE, 2, &sRXCANMessage, true);


		// Getting to this point means that the TX interrupt occurred on
		// message object 1, and the message TX is complete.  Clear the
		// message object interrupt.
		CANIntClear(CANA_BASE, 2);

		// Since the message was sent, clear any error flags.
	}

	// Otherwise, something unexpected caused the interrupt.  This should
	// never happen.
	else
	{
		// Spurious interrupt handling can go here.
	}

	//canaRegs.CAN_GLB_INT_CLR.bit.INT0_FLG_CLR = 1;
	CANGlobalIntClear(CANA_BASE, CAN_GLB_INT_CANINT0);
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

//*****************************************************************************
// Configure the CAN and enter a loop to transmit periodic CAN messages.
//*****************************************************************************
void CAN_Init(void)
{
	//GPIO30 -  CANRXA
	GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 3);
	//GPIO31 - CANTXA
	GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 3);
	GPIO_SetupPinOptions(18, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);

	// Initialize the CAN controller
	CANInit(CANA_BASE);

	// Setup CAN to be clocked off the PLL output clock
	CANClkSourceSelect(CANA_BASE, 0); /* 500kHz CAN-Clock */

	// Set up the bit rate for the CAN bus.  This function sets up the CAN
	// bus timing for a nominal configuration.  You can achieve more control
	// over the CAN bus timing by using the function CANBitTimingSet() instead
	// of this one, if needed.
	// In this example, the CAN bus is set to 500 kHz.  In the function below,
	// the call to SysCtlClockGet() is used to determine the clock rate that
	// is used for clocking the CAN peripheral.  This can be replaced with a
	// fixed value if you know the value of the system clock, saving the extra
	// function call.  For some parts, the CAN peripheral is clocked by a fixed
	// 8 MHz regardless of the system clock in which case the call to
	// SysCtlClockGet() should be replaced with 8000000.  Consult the data
	// sheet for more information about CAN peripheral clocking.
	CANBitRateSet(CANA_BASE, 200000000, 100000);

	// Enable interrupts on the CAN peripheral.  This example uses static
	// allocation of interrupt handlers which means the name of the handler
	// is in the vector table of startup code.  If you want to use dynamic
	// allocation of the vector table, then you must also call CANIntRegister()
	// here.
	CANIntEnable(CANA_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

//	// Enable test mode and select external loopback
//	HWREG(CANA_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
//	HWREG(CANA_BASE + CAN_O_TEST) = CAN_TEST_EXL;

	// Enable the CAN for operation.
	CANEnable(CANA_BASE);

	CANGlobalIntEnable(CANA_BASE, CAN_GLB_INT_CANINT0);

    *(unsigned long *)ucRXMsgData = 0;
    sRXCANMessage.ui32MsgID = 1;                      // CAN message ID - use 1
    sRXCANMessage.ui32MsgIDMask = 0X2000;                  // no mask needed for TX
    sRXCANMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;  // enable interrupt on RX
    sRXCANMessage.ui32MsgLen = sizeof(ucRXMsgData);   // size of message is 8
    sRXCANMessage.pucMsgData = ucRXMsgData;           // ptr to message content
    CANMessageSet(CANA_BASE, 2, &sRXCANMessage, MSG_OBJ_TYPE_RX);

}

