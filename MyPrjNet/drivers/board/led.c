/*
 * File      : led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */
#include <rtthread.h>
#include <stm32f4xx_conf.h>

#define led1_rcc                    RCC_AHB1Periph_GPIOC
#define led1_gpio                   GPIOC
#define led1_pin                    (GPIO_Pin_0)

#define led2_rcc                    RCC_AHB1Periph_GPIOA
#define led2_gpio                   GPIOA
#define led2_pin                    (GPIO_Pin_0)

void rt_hw_led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(led1_rcc | led2_rcc | RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = led1_pin;
	GPIO_Init(led1_gpio, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = led2_pin;
	GPIO_Init(led2_gpio, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

}

void rt_hw_led_set_state(uint8_t led,uint8_t state)
{
	switch (led)
	{
	case 1:
		GPIO_WriteBit(led1_gpio,led1_pin,state);
		break;
	case 2:
		GPIO_WriteBit(led2_gpio,led2_pin,state);
		break;
	}
}

void rt_hw_led_off(rt_uint32_t n)
{
	switch (n)
	{
	case 1:
		GPIO_SetBits(led1_gpio, led1_pin);
		break;
	case 2:
		GPIO_SetBits(led2_gpio, led2_pin);
		break;
	case 3:
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		break;
	}
}


void rt_hw_led_on(rt_uint32_t n)
{
	switch (n)
	{
	case 1:
		GPIO_ResetBits(led1_gpio, led1_pin);
		break;
	case 2:
		GPIO_ResetBits(led2_gpio, led2_pin);
		break;
	case 3:
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		break;
	}
}

#ifdef RT_USING_FINSH
#include <finsh.h>
static rt_uint8_t led_inited = 0;
void led(rt_uint32_t led, rt_uint32_t value)
{
	/* init led configuration if it's not inited. */
	if (!led_inited)
	{
		led_inited = 1;
	}

	if (led == 0)
	{
		/* set led status */
		switch (value)
		{
		case 0:
			rt_hw_led_off(0);
			break;
		case 1:
			rt_hw_led_on(0);
			break;
		default:
			break;
		}
	}

	if (led == 1)
	{
		/* set led status */
		switch (value)
		{
		case 0:
			rt_hw_led_off(1);
			break;
		case 1:
			rt_hw_led_on(1);
			break;
		default:
			break;
		}
	}
}
FINSH_FUNCTION_EXPORT(led, set led[0 - 1] on[0] or off[1].)
#endif

