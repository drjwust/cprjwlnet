/*
 * my_view.h
 *
 *  Created on: 2014Äê4ÔÂ3ÈÕ
 *      Author: Administrator
 */

#ifndef MY_VIEW_H_
#define MY_VIEW_H_

#include <includes.h>


APP_EXT enum {
	OBJ_LABEL_TIME_ID,
	OBJ_LABEL_ERROR_NODE_ID,
	OBJ_LABEL_FIND_NODE_ID,

	OBJ_LABEL_NUM_ID,
	OBJ_LABEL_STATE_ID,
	OBJ_LABEL_TEMP_ID,
	OBJ_LABEL_SIGNAL_ID,
	OBJ_LABEL_R_ALARM_ID,
	OBJ_LABEL_A_ALARM_ID,
	OBJ_LABEL_ADDRESS_ID,
	OBJ_LABEL_POPERITY_ID,

	OBJ_LABEL_ERR_NUM_ID,
	OBJ_LABEL_ERR_STATE_ID,
	OBJ_LABEL_ERR_TEMP_ID,
	OBJ_LABEL_ERR_SIGNAL_ID,
	OBJ_LABEL_ERR_R_ALARM_ID,
	OBJ_LABEL_ERR_A_ALARM_ID,
	OBJ_LABEL_ERR_ADDRESS_ID,
	OBJ_LABEL_ERR_POPERITY_ID,
};

APP_EXT struct rtgui_notebook *the_notebook;

APP_EXT struct rtgui_win *main_win;

APP_EXT rtgui_container_t * Container[5];

APP_EXT rtgui_listctrl_t *listctrl_errornode;
APP_EXT rtgui_listctrl_t *listctrl_allnode;



#endif /* MY_VIEW_H_ */
