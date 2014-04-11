/*
 * main_menu.c
 *
 *  Created on: 2014年4月1日
 *      Author: Administrator
 */

#include <includes.h>

static struct rtgui_timer *timer;
static char str_show[50];
static time_t now;
static struct tm *tmp;


static void _label_update(struct rtgui_timer *timer, void *parameter)
{
	struct rtgui_label *label;
	static uint16_t i = 0;

	if (i++ == 10)
	{
		i = 0;
		/* don't cache the label because it is dynamically created */
		label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_TIME_ID));

		now = time(0);
		tmp = localtime(&now);

		rt_sprintf(str_show, "%d年%d月%d日  %d点%d分", tmp->tm_year + 1900,
				tmp->tm_mon + 1, tmp->tm_mday, tmp->tm_hour, tmp->tm_min);

		/* 设置标签文本并更新控件 */
		rtgui_label_set_text(label, str_show);
		rtgui_widget_update(RTGUI_WIDGET(label));
	}

	/* don't cache the label because it is dynamically created */
	label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_ERROR_NODE_ID));

	rt_sprintf(str_show, "%d", NodeList->error_num);
	/* 设置标签文本并更新控件 */
	rtgui_label_set_text(label, str_show);
	rtgui_widget_update(RTGUI_WIDGET(label));

	/* don't cache the label because it is dynamically created */
	label = RTGUI_LABEL(rtgui_get_self_object(OBJ_LABEL_FIND_NODE_ID));

	/* 设置标签文本并更新控件 */
	rt_sprintf(str_show, "%d", NodeInNets);
	rtgui_label_set_text(label, str_show);
	rtgui_widget_update(RTGUI_WIDGET(label));

	NodeInNets++;
}

static void button_view_error(struct rtgui_object *object,
		struct rtgui_event *event)
{
	if (NodeList->error_num != 0)
	{
		rtgui_notebook_set_current(the_notebook, RTGUI_WIDGET(Container[1]));
	}
	else
	{
		rtgui_win_t *win;
		rtgui_label_t *label;
		rtgui_rect_t rect =
		{ 0, 0, 200, 50 };

		rtgui_rect_moveto(&rect, 20, 100);

		/* 创建一个窗口 */
		win = rtgui_win_create(main_win, "提示", &rect,
		RTGUI_WIN_STYLE_DEFAULT);

		rect.x1 += 20;
		rect.x2 -= 5;
		rect.y1 += 5;
		rect.y2 = rect.y1 + 20;

		label = rtgui_label_create("节点正常,没有错误!");
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		rtgui_container_add_child(RTGUI_CONTAINER(win), RTGUI_WIDGET(label));

		/* 模态显示窗口 */
		rtgui_win_show(win, RT_TRUE);

		/* 删除非自动删除窗口 */
		rtgui_win_destroy(win);
	}
}

static void button_view_state(struct rtgui_object *object,
		struct rtgui_event *event)
{
	rtgui_notebook_set_current(the_notebook, RTGUI_WIDGET(Container[2]));
}

static rt_bool_t _page_refresh_event_handler(struct rtgui_object *object,
		rtgui_event_t *event)
{
	struct rtgui_widget *widget = RTGUI_WIDGET(object);

	if (event->type == RTGUI_EVENT_SHOW)
	{
		rtgui_container_event_handler(object, event);
		rtgui_timer_start(timer);
	}
	else if (event->type == RTGUI_EVENT_HIDE)
	{
		rtgui_container_event_handler(object, event);
		rtgui_timer_stop(timer);
	}
	else
	{
		/* 调用默认的事件处理函数 */
		return rtgui_container_event_handler(object, event);
	}

	return RT_FALSE;
}

rtgui_container_t * my_view1(void)
{
	rtgui_rect_t rect;
	rtgui_container_t *container;
	rtgui_label_t *label;
	rtgui_listctrl_t *box;
	struct rtgui_widget *widget;
	struct rtgui_dc *dc;
	rtgui_button_t *button;

	/*创建这个页面的容器，用于挂载其它控件*/
	container = rtgui_container_create();
	if (container == RT_NULL)
		return RT_NULL;
	rtgui_object_set_event_handler(RTGUI_OBJECT(container),
			_page_refresh_event_handler);
	/*将这个页面的容器放入到notebook中，以便于翻页*/
	rtgui_notebook_add(the_notebook, "main menu", RTGUI_WIDGET(container));

	/*获取页面的绘图区域*/
	rtgui_widget_get_rect(RTGUI_WIDGET(container), &rect);
	rtgui_widget_rect_to_device(RTGUI_WIDGET(container), &rect);
	rect.x1 += 5;
	rect.x2 -= 5;
	rect.y1 += 10;
	rect.y2 = rect.y1 + 20;
	label = rtgui_label_create("欢迎使用无线测温系统");
	RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_CENTER_HORIZONTAL;	//左对齐
//	RTGUI_WIDGET_FOREGROUND(label) = blue;	//前景色为绿色
	rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
	/* container是一个container控件，调用add_child方法添加这个label控件 */
	rtgui_container_add_child(container, RTGUI_WIDGET(label));

	{
		time(&now);
		tmp = localtime(&now);
		rect.x1 += 5;
		rect.x2 -= 5;
		rect.y1 = rect.y2 + 5;
		rect.y2 = rect.y1 + 20;
		rt_sprintf(str_show, "%d年%d月%d日  %d点%d分", tmp->tm_year + 1900,
				tmp->tm_mon + 1, tmp->tm_mday, tmp->tm_hour, tmp->tm_min);
		label = rtgui_label_create(str_show);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//右对齐
		RTGUI_WIDGET_FOREGROUND(label) = blue;	//前景色为绿色
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_TIME_ID);
	}

	{
		rect.x1 = 5;
		rect.x2 = 120;
		rect.y1 = rect.y2 + 5;
		rect.y2 = rect.y1 + 20;
		label = rtgui_label_create("测温节点个数:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT;	//左对齐
		RTGUI_WIDGET_FOREGROUND(label) = blue;	//前景色为绿色
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));

		rect.x1 = 140;
		rect.x2 = 180;
		rt_sprintf(str_show, "%d", NodeList->count);
		label = rtgui_label_create(str_show);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//左对齐
		RTGUI_WIDGET_FOREGROUND(label) = blue;	//前景色为绿色
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
	}

	{
		rect.x1 = 5;
		rect.x2 = 120;
		rect.y1 = rect.y2 + 5;
		rect.y2 = rect.y1 + 20;
		label = rtgui_label_create("已搜索到节点:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT;	//左对齐
		RTGUI_WIDGET_FOREGROUND(label) = blue;	//前景色为绿色
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));

		rect.x1 = 140;
		rect.x2 = 180;
		rt_sprintf(str_show, "%d", NodeInNets);
		label = rtgui_label_create(str_show);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//左对齐
		RTGUI_WIDGET_FOREGROUND(label) = blue;	//前景色为绿色
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_FIND_NODE_ID);
	}

	{
		rect.x1 = 5;
		rect.x2 = 120;
		rect.y1 = rect.y2 + 5;
		rect.y2 = rect.y1 + 20;
		label = rtgui_label_create("异常节点个数:");
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT;	//左对齐
		if (NodeList->error_num == 0)
			RTGUI_WIDGET_FOREGROUND(label) = blue;	//前景色为绿色
		else
			RTGUI_WIDGET_FOREGROUND(label) = red;	//前景色为红色
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));

		rect.x1 = 140;
		rect.x2 = 180;
		rt_sprintf(str_show, "%d", NodeList->error_num);
		label = rtgui_label_create(str_show);
		RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT;	//左对齐
		if (ErrorNodeNums == 0)
			RTGUI_WIDGET_FOREGROUND(label) = blue;	//前景色为绿色
		else
			RTGUI_WIDGET_FOREGROUND(label) = red;	//前景色为红色
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* container是一个container控件，调用add_child方法添加这个label控件 */
		rtgui_container_add_child(container, RTGUI_WIDGET(label));
		rtgui_object_set_id(RTGUI_OBJECT(label), OBJ_LABEL_ERROR_NODE_ID);
	}

	/*获取页面的绘图区域*/
	rtgui_widget_get_rect(RTGUI_WIDGET(container), &rect);
	rtgui_widget_rect_to_device(RTGUI_WIDGET(container), &rect);
	rect.x1 = 0;
	rect.x2 = 80;
	rect.y1 = 0;
	rect.y2 = rect.y1 + 30;
	rtgui_rect_moveto(&rect, 5, 130);
	/* 创建一个button控件 */
	button = rtgui_button_create("浏览错误");
	/* 设置onbutton动作到button_view_error函数 */
	rtgui_button_set_onbutton(button, button_view_error);
	/* 设置button的位置 */
	rtgui_widget_set_rect(RTGUI_WIDGET(button), &rect);
	rtgui_container_add_child(container, RTGUI_WIDGET(button));

	/*获取页面的绘图区域*/
	rtgui_widget_get_rect(RTGUI_WIDGET(container), &rect);
	rtgui_widget_rect_to_device(RTGUI_WIDGET(container), &rect);
	rect.x1 = 0;
	rect.x2 = 80;
	rect.y1 = 0;
	rect.y2 = rect.y1 + 30;
	rtgui_rect_moveto(&rect, 5, 170);
	/* 创建一个button控件 */
	button = rtgui_button_create("查看状态");
	/* 设置onbutton动作到button_view_state函数 */
	rtgui_button_set_onbutton(button, button_view_state);
	/* 设置button的位置 */
	rtgui_widget_set_rect(RTGUI_WIDGET(button), &rect);
	rtgui_container_add_child(container, RTGUI_WIDGET(button));

	/*获取页面的绘图区域*/
	rtgui_widget_get_rect(RTGUI_WIDGET(container), &rect);
	rtgui_widget_rect_to_device(RTGUI_WIDGET(container), &rect);
	rect.x1 = 0;
	rect.x2 = 80;
	rect.y1 = 0;
	rect.y2 = rect.y1 + 30;
	rtgui_rect_moveto(&rect, 5, 210);
	/* 创建一个button控件 */
	button = rtgui_button_create("系统设置");
	/* 设置button的位置 */
	rtgui_widget_set_rect(RTGUI_WIDGET(button), &rect);
	rtgui_container_add_child(container, RTGUI_WIDGET(button));

	/* 创建一个定时器 */
	timer = rtgui_timer_create(500, RT_TIMER_FLAG_PERIODIC, _label_update,
	RT_NULL);
	rtgui_timer_start(timer);

	return container;
}
