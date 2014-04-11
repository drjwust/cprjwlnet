#include <includes.h>

/*
 TOUCH INT: PA3
 */
#define IS_TOUCH_UP()     GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)

/*
 7  6 - 4  3      2     1-0
 s  A2-A0 MODE SER/DFR PD1-PD0
 */
/* bit[1:0] power-down */
#define POWER_MODE0     (0) /* Power-Down Between Conversions. When */
/* each conversion is finished, the converter */
/* enters a low-power mode. At the start of the */
/* next conversion, the device instantly powers up */
/* to full power. There is no need for additional */
/* delays to ensure full operation, and the very first */
/* conversion is valid. The Y? switch is on when in */
/* power-down.*/
#define POWER_MODE1     (1) /* Reference is off and ADC is on. */
#define POWER_MODE2     (2) /* Reference is on and ADC is off. */
#define POWER_MODE3     (3) /* Device is always powered. Reference is on and */
/* ADC is on. */
/* bit[2] SER/DFR */
#define DIFFERENTIAL    (0<<2)
#define SINGLE_ENDED    (1<<2)
/* bit[3] mode */
#define MODE_12BIT      (0<<3)
#define MODE_8BIT       (1<<3)
/* bit[6:4] differential mode */
#define MEASURE_X       (((1<<2) | (0<<1) | (1<<0))<<4)
#define MEASURE_Y       (((0<<2) | (0<<1) | (1<<0))<<4)
#define MEASURE_Z1      (((0<<2) | (1<<1) | (1<<0))<<4)
#define MEASURE_Z2      (((1<<2) | (0<<1) | (0<<0))<<4)
/* bit[7] start */
#define START           (1<<7)

///* X Y change. */
//#define TOUCH_MSR_X     (START | MEASURE_X | MODE_12BIT | DIFFERENTIAL | POWER_MODE0)
//#define TOUCH_MSR_Y     (START | MEASURE_Y | MODE_12BIT | DIFFERENTIAL | POWER_MODE0)

#define TOUCH_MSR_Y  0x90   //读X轴坐标指令 addr:1
#define TOUCH_MSR_X  0xD0   //读Y轴坐标指令 addr:3

#define MIN_X_DEFAULT   334
#define MAX_X_DEFAULT   3794
#define MIN_Y_DEFAULT   3747
#define MAX_Y_DEFAULT   166
#define SAMP_CNT 8                              //the adc array size
#define SAMP_CNT_DIV2 4                         //the middle of the adc array
#define SH   10                                 // Valve value
struct rtgui_touch_device
{
	struct rt_device parent;

	rt_uint16_t x, y;

	rt_bool_t calibrating;
	rt_touch_calibration_func_t calibration_func;

	rt_uint16_t min_x, max_x;
	rt_uint16_t min_y, max_y;

	struct rt_spi_device * spi_device;
	struct rt_event event;
};
static struct rtgui_touch_device *touch = RT_NULL;

static void touch_int_cmd(FunctionalState NewState);

#define X_WIDTH 240
#define Y_WIDTH 320

#define   CS_0()          GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define   CS_1()          GPIO_SetBits(GPIOB,GPIO_Pin_9)

uint8_t SPI_WriteByte(unsigned char data)
{
	//Wait until the transmit buffer is empty
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	// Send the byte
	SPI_I2S_SendData(SPI2, data);

	//Wait until a data is received
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;
	// Get the received data
	data = SPI_I2S_ReceiveData(SPI2);

	// Return the shifted data
	return data;
}

//SPI写数据
static void WriteDataTo7843(unsigned char num)
{
	SPI_WriteByte(num);
}

static void rtgui_touch_calculate(void)
{
	if (touch != RT_NULL)
	{
		/* read touch */
		{
			uint8_t i, j, k, min;
			uint16_t temp;
			rt_uint16_t tmpxy[2][SAMP_CNT];
			for (i = 0; i < SAMP_CNT; i++)
			{
				CS_0();
				WriteDataTo7843(TOUCH_MSR_X);
				tmpxy[0][i] = (SPI_WriteByte(0x00) & 0x7F) << 5;
				tmpxy[1][i] |= (SPI_WriteByte(TOUCH_MSR_Y) >> 3) & 0x1F;

				tmpxy[1][i] = (SPI_WriteByte(0x00) & 0x7F) << 5;
				tmpxy[1][i] |= (SPI_WriteByte(0x00) >> 3) & 0x1F;

				WriteDataTo7843(1 << 7); /* 打开中断 */
				CS_1();
			}

			/* calculate average */
			{
				rt_uint32_t total_x = 0;
				rt_uint32_t total_y = 0;
				for (k = 0; k < 2; k++)
				{
					// sorting the ADC value
					for (i = 0; i < SAMP_CNT - 1; i++)
					{
						min = i;
						for (j = i + 1; j < SAMP_CNT; j++)
						{
							if (tmpxy[k][min] > tmpxy[k][j])
								min = j;
						}
						temp = tmpxy[k][i];
						tmpxy[k][i] = tmpxy[k][min];
						tmpxy[k][min] = temp;
					}
					//check value for Valve value
					if ((tmpxy[k][SAMP_CNT_DIV2 + 1]
							- tmpxy[k][SAMP_CNT_DIV2 - 2]) > SH)
					{
						return;
					}
				}
				total_x = tmpxy[0][SAMP_CNT_DIV2 - 2]
						+ tmpxy[0][SAMP_CNT_DIV2 - 1] + tmpxy[0][SAMP_CNT_DIV2]
						+ tmpxy[0][SAMP_CNT_DIV2 + 1];
				total_y = tmpxy[1][SAMP_CNT_DIV2 - 2]
						+ tmpxy[1][SAMP_CNT_DIV2 - 1] + tmpxy[1][SAMP_CNT_DIV2]
						+ tmpxy[1][SAMP_CNT_DIV2 + 1];
				//calculate average value
				touch->x = total_x >> 2;
				touch->y = total_y >> 2;
				rt_kprintf("touch->x:%d touch->y:%d\r\n", touch->x, touch->y);
			} /* calculate average */
		} /* read touch */

		/* if it's not in calibration status  */
		if (touch->calibrating != RT_TRUE)
		{
			if (touch->max_x > touch->min_x)
			{
				touch->x = (touch->x - touch->min_x) * X_WIDTH
						/ (touch->max_x - touch->min_x);
			}
			else
			{
				touch->x = (touch->min_x - touch->x) * X_WIDTH
						/ (touch->min_x - touch->max_x);
			}

			if (touch->max_y > touch->min_y)
			{
				touch->y = (touch->y - touch->min_y) * Y_WIDTH
						/ (touch->max_y - touch->min_y);
			}
			else
			{
				touch->y = (touch->min_y - touch->y) * Y_WIDTH
						/ (touch->min_y - touch->max_y);
			}
		}
	}
}

static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the EXTI0 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void touch_int_cmd(FunctionalState NewState)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Configure  EXTI  */
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_InitStructure.EXTI_LineCmd = NewState;

	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_Init(&EXTI_InitStructure);
}

static void EXTI_Configuration(void)
{
	/* PA3 touch INT */
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);

	/* Configure  EXTI  */
	touch_int_cmd(ENABLE);
}

/* RT-Thread Device Interface */
static rt_err_t rtgui_touch_init(rt_device_t dev)
{
	NVIC_Configuration();
	EXTI_Configuration();

	/* PG15 touch CS */
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		CS_1();
	}

	CS_0();
	WriteDataTo7843(1 << 7); /* 打开中断 */
	CS_1();

	return RT_EOK;
}

static rt_err_t rtgui_touch_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch (cmd)
	{
	case RT_TOUCH_CALIBRATION:
		touch->calibrating = RT_TRUE;
		touch->calibration_func = (rt_touch_calibration_func_t) args;
		break;

	case RT_TOUCH_NORMAL:
		touch->calibrating = RT_FALSE;
		break;

	case RT_TOUCH_CALIBRATION_DATA:
	{
		struct calibration_data* data;
		data = (struct calibration_data*) args;

		//update
		touch->min_x = data->min_x;
		touch->max_x = data->max_x;
		touch->min_y = data->min_y;
		touch->max_y = data->max_y;
	}
		break;
	}

	return RT_EOK;
}
static void touch_thread_entry(void *parameter)
{
	rt_bool_t touch_down = RT_FALSE;
	rt_uint32_t event_value;
	struct rtgui_event_mouse emouse;
	static struct _touch_previous
	{
		rt_uint32_t x;
		rt_uint32_t y;
	} touch_previous;

	RTGUI_EVENT_MOUSE_BUTTON_INIT(&emouse);
	emouse.wid = RT_NULL;

	while (1)
	{
		if (rt_event_recv(&touch->event, 1,
		RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
		RT_WAITING_FOREVER, &event_value) == RT_EOK)
		{
			while (1)
			{
				if (IS_TOUCH_UP())
				{
					/* touch up */
					emouse.button = (RTGUI_MOUSE_BUTTON_LEFT
							| RTGUI_MOUSE_BUTTON_UP);

					/* use old value */
					emouse.x = touch->x;
					emouse.y = touch->y;

					if (touch_down != RT_TRUE)
					{
						touch_int_cmd(ENABLE);
						break;
					}

					if ((touch->calibrating == RT_TRUE)
							&& (touch->calibration_func != RT_NULL))
					{
						/* callback function */
						touch->calibration_func(emouse.x, emouse.y);

					}
					else
					{
						rtgui_server_post_event(&emouse.parent,
								sizeof(struct rtgui_event_mouse));
					}
					rt_kprintf("touch up: (%d, %d)\n", emouse.x, emouse.y);

					/* clean */
					touch_previous.x = touch_previous.y = 0;
					touch_down = RT_FALSE;

					touch_int_cmd(ENABLE);
					break;
				} /* touch up */
				else /* touch down or move */
				{
					if (touch_down == RT_FALSE)
					{
						rt_thread_delay(RT_TICK_PER_SECOND / 10);
					}
					else
					{
						rt_thread_delay(5);
					}

					if (IS_TOUCH_UP())
						continue;

					/* calculation */
					rtgui_touch_calculate();

					/* send mouse event */
					emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
					emouse.parent.sender = RT_NULL;

					emouse.x = touch->x;
					emouse.y = touch->y;
//                    _set_mouse_position(emouse.x, emouse.y);
					/* init mouse button */
					emouse.button = (RTGUI_MOUSE_BUTTON_LEFT
							| RTGUI_MOUSE_BUTTON_DOWN);

					/* send event to server */
					if (touch->calibrating != RT_TRUE)
					{
#define previous_keep      8
						/* filter. */
						if ((touch_previous.x > touch->x + previous_keep)
								|| (touch_previous.x < touch->x - previous_keep)
								|| (touch_previous.y > touch->y + previous_keep)
								|| (touch_previous.y < touch->y - previous_keep))
						{
							touch_previous.x = touch->x;
							touch_previous.y = touch->y;
							rtgui_server_post_event(&emouse.parent,
									sizeof(struct rtgui_event_mouse));
							if (touch_down == RT_FALSE)
							{
								touch_down = RT_TRUE;
								rt_kprintf("touch down: (%d, %d)\n", emouse.x,
										emouse.y);
							}
							else
							{
								rt_kprintf("touch motion: (%d, %d)\n", emouse.x,
										emouse.y);
							}
						}
					}
					else
					{
						touch_down = RT_TRUE;
					}
				} /* touch down or move */
			} /* read touch */
		} /* event recv */
	} /* thread while(1) */
}

void EXTI9_5_IRQHandler(void)
{
	/* disable interrupt */
	touch_int_cmd(DISABLE);

	rt_event_send(&touch->event, 1);

	EXTI_ClearITPendingBit(EXTI_Line8);
}

rt_err_t rtgui_touch_hw_init()
{
	struct rt_thread * touch_thread;

	/* SPI2 config */
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		SPI_InitTypeDef SPI_InitStructure;

		/* Enable SPI2 Periph clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC,
				ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);

		/* Configure SPI2 pins: PB10-SCK, PC2-MISO and PC3-MOSI */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

		/*!< SPI SCK pin configuration */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/*!< SPI MOSI pin configuration */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		/*!< SPI MISO pin configuration */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_Init(SPI2, &SPI_InitStructure);

		/* Enable SPI_MASTER */
		SPI_Cmd(SPI2, ENABLE);
		SPI_CalculateCRC(SPI2, DISABLE);
	}

	touch = (struct rtgui_touch_device*) rt_malloc(
			sizeof(struct rtgui_touch_device));
	if (touch == RT_NULL)
		return RT_ENOMEM; /* no memory yet */

	/* clear device structure */
	rt_memset(&(touch->parent), 0, sizeof(struct rt_device));

	rt_event_init(&touch->event, "touch", RT_IPC_FLAG_FIFO);

//    touch->spi_device = spi_device;
	touch->calibrating = RT_FALSE;

	touch->min_x = MIN_X_DEFAULT;
	touch->max_x = MAX_X_DEFAULT;
	touch->min_y = MIN_Y_DEFAULT;
	touch->max_y = MAX_Y_DEFAULT;

//	rt_kprintf("MIN X: %d\t\tMIN Y:%D\t\nMAX X:%d\t\tMAX Y:%D\n", touch->min_x,
//			touch->min_y, touch->max_x,touch->max_y);

	/* init device structure */
	touch->parent.type = RT_Device_Class_Unknown;
	touch->parent.init = rtgui_touch_init;
	touch->parent.control = rtgui_touch_control;
	touch->parent.user_data = RT_NULL;

	/* register touch device to RT-Thread */
	rt_device_register(&(touch->parent), "touch", RT_DEVICE_FLAG_RDWR);

	touch_thread = rt_thread_create("touch", touch_thread_entry, RT_NULL, 1024,
	RTGUI_SVR_THREAD_PRIORITY - 1, 1);
	if (touch_thread != RT_NULL)
		rt_thread_startup(touch_thread);

	return RT_EOK;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void touch_t(rt_uint16_t x, rt_uint16_t y)
{
	struct rtgui_event_mouse emouse;
	emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
	emouse.parent.sender = RT_NULL;

	emouse.x = x;
	emouse.y = y;
	/* init mouse button */
	emouse.button = (RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_DOWN);
	rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));

	rt_thread_delay(2);
	emouse.button = (RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_UP);
	rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));

	rt_kprintf("MIN X: %d\t\tMIN Y:%d\t\nMAX X:%d\t\tMAX Y:%d\n", touch->min_x,
			touch->min_y, touch->max_x,touch->max_y);
}

FINSH_FUNCTION_EXPORT(touch_t, x & y);
#endif
