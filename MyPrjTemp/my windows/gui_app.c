#include <includes.h>


static void my_application_entry(void *parameter)
{
    struct rtgui_app *app;
    struct rtgui_rect rect;

    app = rtgui_app_create("gui_demo");
    if (app == RT_NULL)
        return;

    /* create a full screen window */
    rtgui_graphic_driver_get_rect(rtgui_graphic_driver_get_default(), &rect);

    main_win = rtgui_win_create(RT_NULL, "demo_win", &rect,
                                RTGUI_WIN_STYLE_NO_BORDER | RTGUI_WIN_STYLE_NO_TITLE);
    if (main_win == RT_NULL)
    {
        rtgui_app_destroy(app);
        return;
    }


    /* create a no title notebook that we can switch demo on it easily. */
    the_notebook = rtgui_notebook_create(&rect, RTGUI_NOTEBOOK_NOTAB);
    if (the_notebook == RT_NULL)
    {
        rtgui_win_destroy(main_win);
        rtgui_app_destroy(app);
        return;
    }

    rtgui_container_add_child(RTGUI_CONTAINER(main_win), RTGUI_WIDGET(the_notebook));

    /*初始化各个页面的视图*/
    Container[0] = my_view1();
    Container[1] = my_view2();
    Container[2] = my_view3();
    Container[3] = my_view4();
    Container[4] = my_view5();

    rtgui_win_show(main_win, RT_FALSE);

    /* 执行工作台事件循环 */
    rtgui_app_run(app);

    rtgui_app_destroy(app);
}

void my_application_init()
{
    static rt_bool_t inited = RT_FALSE;

    if (inited == RT_FALSE) /* 避免重复初始化而做的保护 */
    {
        rt_thread_t tid;

        tid = rt_thread_create("wb",
                               my_application_entry, RT_NULL,
                               2048 * 2, 25, 10);

        if (tid != RT_NULL)
            rt_thread_startup(tid);

        inited = RT_TRUE;
    }
}


