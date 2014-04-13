/**
 ******************************************************************************
 * @file    IO_Toggle/stm32f4xx_it.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    19-September-2011
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <includes.h>

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
 * @{
 */

/** @addtogroup IO_Toggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
//void HardFault_Handler(void)
//{
//    // definition in libcpu/arm/cortex-m4/context_*.S
//}
/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
//void PendSV_Handler(void)
//{
//    // definition in libcpu/arm/cortex-m4/context_*.S
//}
/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
//void SysTick_Handler(void)
//{
//    // definition in boarc.c
//}
/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
 {
 }*/

void USART1_IRQHandler(void)
{
#ifdef RT_USING_UART1
	extern struct rt_device uart1_device;
	extern void rt_hw_serial_isr(struct rt_device *device);

	/* enter interrupt */
	rt_interrupt_enter();

	rt_hw_serial_isr(&uart1_device);

	/* leave interrupt */
	rt_interrupt_leave();
#endif
}

void USART2_IRQHandler(void)
{
#ifdef RT_USING_UART2
	extern struct rt_device uart2_device;
	extern void rt_hw_serial_isr(struct rt_device *device);

	/* enter interrupt */
	rt_interrupt_enter();

	rt_hw_serial_isr(&uart2_device);

	/* leave interrupt */
	rt_interrupt_leave();
#endif
}

void USART3_IRQHandler(void)
{
#ifdef RT_USING_UART3
	extern struct rt_device uart3_device;
	extern void rt_hw_serial_isr(struct rt_device *device);

	/* enter interrupt */
	rt_interrupt_enter();

	rt_hw_serial_isr(&uart3_device);

	/* leave interrupt */
	rt_interrupt_leave();
#endif
}

void UART4_IRQHandler(void)
{
#ifdef RT_USING_UART4
	extern struct rt_device uart4_device;
	extern void rt_hw_serial_isr(struct rt_device *device);

	/* enter interrupt */
	rt_interrupt_enter();

	rt_hw_serial_isr(&uart4_device);

	/* leave interrupt */
	rt_interrupt_leave();
#endif
}

void USART6_IRQHandler(void)
{
#ifdef RT_USING_UART6
	extern struct rt_device uart6_device;
	extern void rt_hw_serial_isr(struct rt_device *device);

	/* enter interrupt */
	rt_interrupt_enter();

	rt_hw_serial_isr(&uart6_device);

	/* leave interrupt */
	rt_interrupt_leave();
#endif
}

void TIM7_IRQHandler(void)
{
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, 0);
	GPIO_WriteBit(GPIOA, GPIO_Pin_0,	//只是用于测试是否产生TIM中断的
			1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0));
}

//void EXTI9_5_IRQHandler(void)
//{
//	static int8_t channel, n, complete = 0;
//	rt_interrupt_enter();
//	if (EXTI_GetFlagStatus(EXTI_Line5) == SET)	//
//	{
//		EXTI_ClearITPendingBit(EXTI_Line5);
//		//FIXME 取走ADC1的数据
//		for (channel = 0; channel < 8; channel++)
//		{
//			AD_Sample[channel][n] = read16(ARM2_ADC_1_BASE);
//		}
//		complete++;
//	}
//	if (EXTI_GetFlagStatus(EXTI_Line6) == SET)	//
//	{
//		EXTI_ClearITPendingBit(EXTI_Line6);
//		//FIXME 取走ADC2的数据
//		for (channel = 8; channel < 15; channel++)
//		{
//			AD_Sample[channel][n] = read16(ARM2_ADC_1_BASE);
//		}
//		complete++;
//	}
//	if (EXTI_GetFlagStatus(EXTI_Line7) == SET)	//
//	{
//		EXTI_ClearITPendingBit(EXTI_Line7);
//		//FIXME 取走ADC3的数据
//		for (channel = 16; channel < 23; channel++)
//		{
//			AD_Sample[channel][n] = read16(ARM2_ADC_1_BASE);
//		}
//		complete++;
//	}
////	GPIO_WriteBit(GPIOA,GPIO_Pin_0,1 - GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_0));
//
//	if (complete == 3)
//	{
//		complete = 0;
//		rt_hw_ad7606_end_sample();	//采样结束之后，使CONST变为低，为下一次转换做准备
//		n++;
//		if (n == 64)
//			n = 0;
////		GPIO_WriteBit(GPIOA, GPIO_Pin_0,1);
////				);
//	}
//	rt_interrupt_leave();
//}
#ifdef RT_USING_UART6
void DMA2_Stream6_IRQHandler(void)
{
	extern struct rt_device uart6_device;
	extern void rt_hw_serial_dma_tx_isr(rt_device_t);

	rt_interrupt_enter();
//	if (DMA_GetITStatus(DMA2_Stream6, DMA_FLAG_TCIF6))
	{
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream6, DMA_FLAG_TCIF6);
		rt_hw_serial_dma_tx_isr(&uart6_device);
	}
	rt_interrupt_leave();
}
#endif

#ifdef RT_USING_UART3
void DMA1_Stream3_IRQHandler(void)
{
	extern struct rt_device uart3_device;
	extern void rt_hw_serial_dma_tx_isr(rt_device_t);

	rt_interrupt_enter();
	if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
	{
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
		rt_hw_serial_dma_tx_isr(&uart3_device);
	}
	rt_interrupt_leave();
}
#endif

void EXTI2_IRQHandler(void)
{
	rt_interrupt_enter();
	if (EXTI_GetFlagStatus(EXTI_Line2) == SET)	//
	{
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
	rt_interrupt_leave();
}

void EXTI9_5_IRQHandler(void)
{
	rt_interrupt_enter();
	if (EXTI_GetFlagStatus(EXTI_Line6) == SET)	//
	{
		EXTI_ClearITPendingBit(EXTI_Line6);
		rt_sem_release(&rf_sem);
	}
	if (EXTI_GetFlagStatus(EXTI_Line8) == SET)	//
	{
//		Pen_IRQHandler();
	}
	rt_interrupt_leave();
}

#if defined(RT_USING_DFS) && STM32_USE_SDIO
/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
//    extern int SD_ProcessIRQSrc(void);

    /* enter interrupt */
    rt_interrupt_enter();

    /* Process All SDIO Interrupt Sources */
    if( SD_ProcessIRQSrc() == 2)
		rt_kprintf("SD Error\n");

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
