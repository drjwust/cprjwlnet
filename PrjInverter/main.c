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
	ExtDA_Init();

//	LED_Blinky(48);
	for (;;)
	{
		if (MainTskTrigger)
		{
			float udc, state;

			MainTskTrigger = 0;		//清触发标志
			udc = (UdcA + UdcB + UdcC) / 3;
			state = CheckPwrState(Upcc_ab, Upcc_bc, Upcc_ca, udc);
			if (state)
			{
				GPIO_WritePin(GPIO_PWRON, 1);
			}
		}
	}
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
