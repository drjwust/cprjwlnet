/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#define APP_GLOBALS

#include <includes.h>

void thread_init_entry(void* parameter)
{

	/* LwIP Initialization */
#ifdef RT_USING_LWIP
	{
		extern void lwip_sys_init(void);

		/* register ethernetif device */
		eth_system_device_init();

		rt_hw_stm32_eth_init();
		/* re-init device driver */
		rt_device_init_all();

		/* init lwip system */
		lwip_sys_init();
		rt_kprintf("TCP/IP initialized!\n");
	}
#endif


//FS
#ifdef RT_USING_DFS
	{
		/* init the device filesystem */
		dfs_init();

#ifdef RT_USING_DFS_ELMFAT
		/* init the elm chan FatFs filesystam*/
		elm_init();

		/* mount sd card fat partition 1 as root directory */
		if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
		{
			rt_kprintf("File System initialized!\n");
		}
#endif
	}
#endif

	user_data_init();

#ifdef RT_USING_RTGUI
	{
		rt_device_t lcd;

		/* init lcd */
		rt_hw_lcd_init();

		rtgui_touch_hw_init();

		rt_device_init_all();
		rt_device_t device;
		struct rt_device_rect_info info;

		device = rt_device_find("lcd");
		if (device != RT_NULL)
		{
			info.width = 240;
			info.height = 320;
			/* set graphic resolution */
			rt_device_control(device, RTGRAPHIC_CTRL_SET_MODE, &info);
		}

		/* re-set graphic device */
		rtgui_graphic_set_device(device);

		/* GUI系统初始化 */
		rtgui_system_server_init();

//		calibration_init();

//		application_init();
		my_application_init();
	}
#endif /* #ifdef RT_USING_RTGUI */


//GUI
	while (1)
	{
//		rt_hw_led_on(1);
		rt_thread_delay(300);
//		rt_hw_led_off(1);
//		rt_thread_delay(300);
//		rt_kprintf("Are You OK?\n");
	}
}

int rt_application_init(void)
{
	rt_thread_t init_thread;


	//------- init led1 thread
	init_thread = rt_thread_create("init", thread_init_entry, RT_NULL, 2048 * 4,
			8, 20);
	if (init_thread != RT_NULL)
	{
		rt_thread_startup(init_thread);
	}

	rf_thread_init();

	return 0;
}
/*@}*/
