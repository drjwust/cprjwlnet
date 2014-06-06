/*
 * my_view4.c
 *
 *  Created on: 2014年4月3日
 *      Author: Administrator
 */
#include <includes.h>


static void button_return_menu(struct rtgui_object *object,
		struct rtgui_event *event)
{
	rtgui_notebook_set_current(the_notebook, RTGUI_WIDGET(Container[1]));
}


static rt_bool_t _error_page_refresh_event_handler(struct rtgui_object *object,
		rtgui_event_t *event)
{
	if (event->type == RTGUI_EVENT_PAINT)
	{
		struct rtgui_label *label;
		NODE_DATA *item, **items;
		int fd,len;
		char show_str[50],*ps,*pstr;

		items = listctrl_errornode->items;
		item = items[listctrl_errornode->current_item];
		{
			/* don't cache the label because it is dynamically created */
			label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_ERR_NUM_ID));
			rt_sprintf(show_str, "%d", item->num);

			rtgui_label_set_text(label, show_str);
			rtgui_widget_update(RTGUI_WIDGET(label));
		}
		{
			/* don't cache the label because it is dynamically created */
			label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_ERR_STATE_ID));
			rt_sprintf(show_str, "%d", item->state);
			rtgui_label_set_text(label, show_str);
			rtgui_widget_update(RTGUI_WIDGET(label));
		}
		{
			/* don't cache the label because it is dynamically created */
			label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_ERR_TEMP_ID));
			rt_sprintf(show_str, "%d", item->temperature);
			rtgui_label_set_text(label, show_str);
			rtgui_widget_update(RTGUI_WIDGET(label));
		}
		{
			/* don't cache the label because it is dynamically created */
			label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_ERR_SIGNAL_ID));
			rt_sprintf(show_str, "%d", item->signal_intensity);
			rtgui_label_set_text(label, show_str);
			rtgui_widget_update(RTGUI_WIDGET(label));
		}
		{
			/* don't cache the label because it is dynamically created */
			label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_ERR_R_ALARM_ID));
			rt_sprintf(show_str, "%d", item->relative_alarm_value);
			rtgui_label_set_text(label, show_str);
			rtgui_widget_update(RTGUI_WIDGET(label));
		}
		{
			/* don't cache the label because it is dynamically created */
			label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_ERR_A_ALARM_ID));
			rt_sprintf(show_str, "%d", item->abs_alram_value);
			rtgui_label_set_text(label, show_str);
			rtgui_widget_update(RTGUI_WIDGET(label));
		}
		{
			fd = open("/config.ini",O_RDONLY,0);
			lseek(fd,item->address,DFS_SEEK_SET);
			f_gets(fd,show_str);
			strtok_r(show_str," = ",&pstr);
			ps = strtok_r(NULL," = ",&pstr);
			len = strlen(ps);
			ps[len-2] = '\0';
			/* don't cache the label because it is dynamically created */
			label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_ERR_ADDRESS_ID));
			rtgui_label_set_text(label, ps);
			rtgui_widget_update(RTGUI_WIDGET(label));
		}
		{
			lseek(fd,item->name,DFS_SEEK_SET);
			f_gets(fd,show_str);
			strtok_r(show_str," = ",&pstr);
			ps = strtok_r(NULL," = ",&pstr);
			close(fd);
			len = strlen(ps);
			ps[len-2] = '\0';
			/* don't cache the label because it is dynamically created */
			label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_ERR_POPERITY_ID));
			rtgui_label_set_text(label, ps);
			rtgui_widget_update(RTGUI_WIDGET(label));
		}
	}
		/* 调用默认的事件处理函数 */
		return rtgui_container_event_handler(object, event);
}

rtgui_container_t *my_view4(void)
{
	rtgui_rect_t rect;
	rtgui_container_t *container;
	rtgui_label_t *label;
	struct rtgui_widget *widget;
	rtgui_button_t *button;
	struct rtgui_staticline *line;
	char show_str[30],*ps,*pstr;
	NODE_DATA **items, *item;
	int fd,len;

	items = listctrl_errornode->items;
	item = items[NodeList->current];
	/*创建这个页面的容器，用于挂载其它控件*/
	container = rtgui_container_create();
	if (container == RT_NULL)
		return RT_NULL;
	rtgui_object_set_event_handler(RTGUI_OBJECT(container),
			_error_page_refresh_event_handler);

	/*将这个页面的容器放入到notebook中，以便于翻页*/
	rtgui_notebook_add(the_notebook, "error node", RTGUI_WIDGET(container));
	{
		/*获取页面的绘图区域*/
		rtgui_widget_get_rect(RTGUI_WIDGET(container), &rect);
		rtgui_widget_rect_to_device(RTGUI_WIDGET(container), &rect);
		rect.x1 += 5;
		rect.x2 -= 5;
		rect.y1 += 10;
		rect.y2 = rect.y1 + 20;
		label = rtgui_label_create("欢迎使用无线测温系统");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_CENTER_HORIZONTAL;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));

		rect.x1 = 5;
		rect.x2 = 235;
		rect.y1 = 30, rect.y2 = 35;
		line = rtgui_staticline_create(RTGUI_HORIZONTAL);
		/* 设置静态线的位置信息 */
		rtgui_widget_set_rect(RTGUI_WIDGET(line), &rect);
		/* 添加静态线到视图中 */
		rtgui_container_add_child(container, RTGUI_WIDGET(line));
	}
	{
		rect.x1 = 0, rect.x2 = 50;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 5, 45);
		label = rtgui_label_create("节点:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
	}

	{
		rect.x1 = 0, rect.x2 = 50;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 65, 45);
		rt_sprintf(show_str, "%d", item->num);
		label = rtgui_label_create(show_str);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_ERR_NUM_ID);
	}

	{
		rect.x1 = 0, rect.x2 = 50;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 125, 45);
		label = rtgui_label_create("温度:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
	}

	{
		rect.x1 = 0, rect.x2 = 40;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 175, 45);
		rt_sprintf(show_str, "%d", item->temperature);
		label = rtgui_label_create(show_str);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_ERR_TEMP_ID);
	}

	{
		rect.x1 = 0, rect.x2 = 50;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 5, 70);
		label = rtgui_label_create("状态:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
	}

	{
		rect.x1 = 0, rect.x2 = 50;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 65, 70);
		rt_sprintf(show_str, "%d", item->state);
		label = rtgui_label_create(show_str);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_ERR_STATE_ID);
	}

	{
		rect.x1 = 0, rect.x2 = 50;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 125, 70);
		label = rtgui_label_create("信号:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
	}

	{
		rect.x1 = 0, rect.x2 = 40;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 175, 70);
		rt_sprintf(show_str, "%d", item->signal_intensity);
		label = rtgui_label_create(show_str);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_ERR_SIGNAL_ID);
	}

	{
		rect.x1 = 0, rect.x2 = 100;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 15, 95);
		label = rtgui_label_create("相对报警温度:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
	}

	{
		rect.x1 = 0, rect.x2 = 40;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 120, 95);
		rt_sprintf(show_str, "%d", item->relative_alarm_value);
		label = rtgui_label_create(show_str);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_ERR_R_ALARM_ID);
	}

	{
		rect.x1 = 0, rect.x2 = 100;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 15, 120);
		label = rtgui_label_create("绝对报警温度:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
	}

	{
		rect.x1 = 0, rect.x2 = 40;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 120, 120);
		rt_sprintf(show_str, "%d", item->abs_alram_value);
		label = rtgui_label_create(show_str);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_ERR_A_ALARM_ID);
	}

	{
		rect.x1 = 0, rect.x2 = 50;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 5, 145);
		label = rtgui_label_create("地址:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
	}

	{
		rect.x1 = 0, rect.x2 = 180;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 65, 145);
		fd = open("/config.ini",O_RDONLY,0);
		lseek(fd,item->address,DFS_SEEK_SET);
		f_gets(fd,show_str);
		strtok_r(show_str," = ",&pstr);
		ps = strtok_r(NULL," = ",&pstr);
		len = strlen(ps);
		ps[len-2] = '\0';
		label = rtgui_label_create(ps);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_ERR_ADDRESS_ID);
	}

	{
		rect.x1 = 0, rect.x2 = 50;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 5, 170);
		label = rtgui_label_create("名称:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
	}

	{
		rect.x1 = 0, rect.x2 = 180;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 20;
		rtgui_rect_moveto(&rect, 65, 170);
		lseek(fd,item->name,DFS_SEEK_SET);
		f_gets(fd,show_str);
		close(fd);
		strtok_r(show_str," = ",&pstr);
		ps = strtok_r(NULL," = ",&pstr);
		len = strlen(ps);
		ps[len-2] = '\0';
//		rt_kprintf("%s", ps);
		label = rtgui_label_create(ps);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT;	//中心对齐
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_ERR_POPERITY_ID);
	}

	{
		rect.x1 = 0;
		rect.x2 = 40;
		rect.y1 = 0;
		rect.y2 = rect.y1 + 30;
		rtgui_rect_moveto(&rect, 195, 280);
		/* 创建一个button控件 */
		button = rtgui_button_create("返回");
		/* 设置onbutton动作到button_view_state函数 */
		rtgui_button_set_onbutton(button, button_return_menu);
		/* 设置button的位置 */
		rtgui_widget_set_rect(RTGUI_WIDGET(button), &rect);
		rtgui_container_add_child(container, RTGUI_WIDGET(button));
	}

	return container;
}
