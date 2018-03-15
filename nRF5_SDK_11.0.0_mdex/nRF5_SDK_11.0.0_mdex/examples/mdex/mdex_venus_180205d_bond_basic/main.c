/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */
 
/*******************************************************************************************
	The upper, the later
	
	[TODO]
	- DFU (Firmware upgrade)

	[Almost done, but LATER]
	- Delete Whitelist by button ==> Looking not necessary
	- Answer timer tick via nus request.
	- Save working : Idle (Check per 10 sec) - Wait (check per 2 sec) - Work (check per 1 sec).
	
	[DONE]
	- Divide and rule : ble, peri, adc
	- Power save : scan 1 time per sec, LED 0 off
	- Process of receiving message (i.e. turn on 16)
	- Error ==> dipsw 1 on makes all data zero. android app bug fixed
	- LED on with bond ==> make it blinky to ready to connect. call ble_timeout_handler_PairDone to system reset
	- Same to Venus 1.1.0c with ADV mode
	- Pin assign adaptive to Venus
	- Merge to Venus. adaptive BSP: Ram, Clock.
		==> RAM reduces to AA(AC;256KB/32KB --> AA;256KB/16KB). 
		==> ROM = Code + RO + RW. 		RAM = RW + ZI. 
		==> S130: Rom = 104KB , Ram = 0x13C8(5KB)
		==> for DFU mode : Application = 0x0001B000 - 0x0003C000 (132KB), DFU = 0x0003C000 - 0x00040000 (16KB), MBR = 0x00000000 - 0x00001000 (4KB)
			http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v11.0.0%2Fbledfu_architecture.html
		==> Venus app : 20,600B + 4700B
	- Broadcast permanently
	- Basic libraries : Timer/ handler, Delay, UART/ handler, GPIO/ handler, ADC/ handler, NUS
	- HID keyboard
	- ADC, LED of GPO by Button of GPI. GPIO should be separated from the LED groups and Button groups.
	- I2C/ handler / wakeup(X)
	- Sensing. Adopts 4067 decoder and FSR sensor. (ADC, GPIO)

	[ABANDON]
	- HID mode; Gamepad. Mode switch gives inconvenience to the users. And takes too much time.
	- Sleep. and wakeup with 3 axis. (TWI, 3axis sensor)
	- UICR, RTCK, RealTimeCounter, Flash, RAM retention, WDT, Temperature(O), TWI
		- RTCK(LFCLK; 32768): Venus doesn't have it. 
		- RTC: No need.
		- WDT : No use. It needs LFCLK
		- Flash : example doesn't work
		- UICR : Difficult
		- Microsecond timer : Millisecond achieved. Micro use delay with while(duration--);
	- Android ==> Arrow button to control the game. with Floating button for mode changing.
	- Multi service : Nus + HID. Dip switch selectable working
	- 
	
*******************************************************************************************/
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "boards.h"

#include "softdevice_handler.h"
#include "ble_advertising.h"
#include "ble_nus.h"

#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"

#include "app_timer.h"

#include "venus_peri.h"
#include "venus_ble.h"
#include "venus_app.h"


/*
MAC (Galaxy Note5)	: 	0x84 0xE4 0xF6 0x6F 0xA8 0x78
MAC (Nexus 5)		: 	0x5E 0xC5 0xC3 0x2B 0x9E 0x65
MAC (Galaxy Note8)	:	0xCA 0xAB 0xFB 0x34 0xC6 0x49.

USE_PCA10028_SIMUL_PINMAP
*/
//===============================================================================================================
//	<--		VENUS APP Ends here
//===============================================================================================================

#define	BATTERY_LEVEL_OFF		1

void gotoSystemPowerOff()
{
	NRF_POWER->SYSTEMOFF = 1;
}

uint32_t g_battery_cutoff_count = 0;

void check_battery_to_off(int threshold_battery_off)
{
	//	sometimes mischeck battery level. It means even though  battery level is high enough, sometimes result is zero. 
	//	To prevent this mischeck, check battery 10 times.
	if(get_battery_level_100() < threshold_battery_off) {
		g_battery_cutoff_count++;
	}
	else {
		g_battery_cutoff_count = 0;
	}

	if( 10 < g_battery_cutoff_count ) {
		gotoSystemPowerOff();
	}
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void main_config(void)
{
//	uint32_t err_code;
	bool erase_bonds;

	//--------------------------------------------------------------
	//		BSP - Keep the order of function call
	//--------------------------------------------------------------

	// Step 1) Initialize drivers.
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	uart_init();
	gpio_init();

	//	Step 2) Initialize BSP
	buttons_leds_init(&erase_bonds); // order should be after gpio_init

	//	Step 3) Initialize BLE
	ble_stack_init();
	device_manager_init(erase_bonds);
	gap_params_init();
	services_init();
	//advertising_init();
	advertising_permanently_init();
	conn_params_init();

}


/**@brief Application main function.
 */
int main(void)
{
	uint32_t err_code;

	//--------------------------------------------------------------
	//	Initialize(Config) bsp here...
	//	consider board's pin setting...
	main_config();


	//--------------------------------------------------------------
	//	Power ON, hello message. LED on and off for 1500ms
	{
		printf("\r\nUART Start!\r\n");
		{
			app_led_turn_on(true); // LED ON
			nrf_delay_ms(1000);
			app_led_turn_on(false); // LED OFF
		}
	}

	//--------------------------------------------------------------
	//	Starts venus application
	{
		peri_main_start();
		app_main_start();

		if(get_dipsw_state(DIP_0) == false) { // case nus service mode
			err_code = ble_adv_start(ADV_REQ_CONNECT);
			APP_ERROR_CHECK(err_code);
		}
		else {	//	case adv mode
			err_code = ble_adv_start(ADV_ONLY);
			APP_ERROR_CHECK(err_code);
		}
	}

	//--------------------------------------------------------------
    // Enter main loop.
    for (;;)
    {
		__WFI(); // Wait for interrupts... It's not predicable. Doesn't wait just for Timer1 but also Timer0 of BLE stack and other interrupts.

        power_manage();
		check_battery_to_off(BATTERY_LEVEL_OFF);
		
		app_main_timer_work();
    }
}


/** 
 * @}
 */
