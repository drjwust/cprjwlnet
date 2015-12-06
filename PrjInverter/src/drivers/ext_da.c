/*
 * ext_da.c
 *
 *  Created on: 2015Äê12ÔÂ4ÈÕ
 *      Author: Administrator
 */
#include <includes.h>
#define ASRAM_CS4_START_ADDR 0X00380000		//todo daiding

void ExtDA_Init(void)
{
	//Configure to run EMIF1 on full Rate (EMIF1CLK = CPU1SYSCLK)
	  EALLOW;
	  ClkCfgRegs.PERCLKDIVSEL.bit.EMIF1CLKDIV = 0x0;
	  EDIS;

	  EALLOW;
	///Grab EMIF1 For CPU1
	  Emif1ConfigRegs.EMIF1MSEL.all = 0x93A5CE71;

	//Disable Access Protection (CPU_FETCH/CPU_WR/DMA_WR)
	  Emif1ConfigRegs.EMIF1ACCPROT0.all = 0x0;

	// Commit the configuration related to protection. Till this bit remains set
	// content of EMIF1ACCPROT0 register can't be changed.
	  Emif1ConfigRegs.EMIF1COMMIT.all = 0x1;

	  // Lock the configuration so that EMIF1COMMIT register can't be changed any more.
	  Emif1ConfigRegs.EMIF1LOCK.all = 0x1;
	  EDIS;

	//Configure GPIO pins for EMIF1
	  setup_emif1_pinmux_async_16bit(0);

	//Configure the access timing for CS2 space
	  Emif1Regs.ASYNC_CS4_CR.all =  (EMIF_ASYNC_ASIZE_16  	| // 16Bit Memory Interface
			  	  	  	  	  	  	 EMIF_ASYNC_TA_1 		| // Turn Around time of 2 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_RHOLD_1 	| // Read Hold time of 1 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_RSTROBE_4 	| // Read Strobe time of 4 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_RSETUP_1 	| // Read Setup time of 1 Emif Clock
									 EMIF_ASYNC_WHOLD_8 	| // Write Hold time of 1 Emif Clock
									 EMIF_ASYNC_WSTROBE_16 	| // Write Strobe time of 1 Emif Clock
									 EMIF_ASYNC_WSETUP_8 	| // Write Setup time of 1 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_EW_DISABLE 	| // Extended Wait Disable.
			  	  	  	  	  	  	 EMIF_ASYNC_SS_DISABLE    // Strobe Select Mode Disable.
			  	  	  	  	  	  	);
}

void ExtDA_Output(char channel, int value)
{
	*((int* )ASRAM_CS4_START_ADDR + channel) = value;
}

