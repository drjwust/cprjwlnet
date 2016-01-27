/*
 * main.c
 */
#include <includes.h>
void LED_Blinky(char x);
void ExtIO_Init(void);

int main(void)
{

	APF_Init();
	BSP_Init();
	ExtIO_Init();
	ExtDA_Init();
	SetOC1Value(10);
	SetOC2Value(10);
	SetOVValue(50);

//	LED_Blinky(48);
	for (;;)
		;
	return 0;
}

void LED_Blinky(char x)
{
	unsigned int i = 0;
	for (;;)
	{
		if (GPIO_ReadPin(54))
			GPIO_WritePin(86, 1);
		else
			GPIO_WritePin(86, 0);
		//
		// Turn on LED
		//
		GPIO_WritePin(x, 0);
//		GPIO_WritePin(87, 0);
		//
		// Delay for a bit.
		//
		DELAY_US(1000);
//		ExtDA_Output(1,i++%4000);
		//
		// Turn off LED
		//
		GPIO_WritePin(x, i++);
//		GPIO_WritePin(87, 1);
		//
		// Delay for a bit.
		//
		DELAY_US(1000);
	}
}

void ExtIO_Init(void)
{

	//Output setup
	GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);	//D33
	GPIO_SetupPinOptions(41, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(48, GPIO_MUX_CPU1, 0);	//D34
	GPIO_SetupPinOptions(48, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(51, GPIO_MUX_CPU1, 0);	//DO3->LED1
	GPIO_SetupPinOptions(51, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);	//DO4->LED2
	GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(58, GPIO_MUX_CPU1, 0);	//TP50
	GPIO_SetupPinOptions(58, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(59, GPIO_MUX_CPU1, 0);	//TP51
	GPIO_SetupPinOptions(59, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(86, GPIO_MUX_CPU1, 0);	//DO1->继电器1
	GPIO_SetupPinOptions(86, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(87, GPIO_MUX_CPU1, 0);	//DO2->继电器2
	GPIO_SetupPinOptions(87, GPIO_OUTPUT, GPIO_PUSHPULL);

	GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);	//ExtDAC CS
	GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(32, 0);

	//Input Setup
	GPIO_SetupPinMux(53, GPIO_MUX_CPU1, 0);	//DIN1
	GPIO_SetupPinOptions(53, GPIO_INPUT, GPIO_PUSHPULL); //TODO 待定
	GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 0);	//DIN2
	GPIO_SetupPinOptions(54, GPIO_INPUT, GPIO_PUSHPULL); //TODO 待定

}
