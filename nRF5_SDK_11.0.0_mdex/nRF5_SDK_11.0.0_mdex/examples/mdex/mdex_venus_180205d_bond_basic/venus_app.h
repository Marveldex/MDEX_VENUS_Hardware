#ifndef _VENUS_APP_H_
#define	_VENUS_APP_H_



//===============================================================================================================
//		VENUS APP - Variables, Declarations
//===============================================================================================================

//===============================================================================================================
//		VENUS APP - Functions Definitions
//===============================================================================================================

uint32_t get_battery_level_100(void);


void task_timeout_handler_scan_32ch(void * p_context);

void task_timer_config(void);


void app_main_timer_work(void);
void app_main_start(void);


#endif // _VENUS_APP_H_
