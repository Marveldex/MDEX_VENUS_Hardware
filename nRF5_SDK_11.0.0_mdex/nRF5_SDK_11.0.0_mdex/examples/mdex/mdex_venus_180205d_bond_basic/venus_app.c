#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "boards.h"

#include "nrf_drv_gpiote.h"
#include "nrf_adc.h"
#include "nrf_delay.h"

#include "app_error.h"
#include "app_trace.h"
//#include "app_button.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"

#include "ble_nus.h"
#include "venus_peri.h"
#include "venus_ble.h"
#include "venus_app.h"


//===============================================================================================================
//		VENUS APP - Variables, Declarations
//===============================================================================================================

#define	NUS_PACKET_MAX_SIZE	20
uint8_t	g_ucNUS_PacketBuffer[NUS_PACKET_MAX_SIZE];

//	battery
uint32_t 	g_uiBATTERY_level_100 = 100;
unsigned char g_uc_battery_high = 0;
unsigned char g_uc_battery_low = 0;

//	32ch adc
#define	VENUS_SCAN_CELL_NUM	32
uint8_t	g_ucCellData32ch[VENUS_SCAN_CELL_NUM];

unsigned int g_adc_32ch_sum = 0;
unsigned int g_adc_32ch_valid_count = 0;

#define	ADC_TRIM_VALUE_7bit	127

#define	THRESHOLD_VALID_CELL_CUTOFF	5

volatile bool g_timer_flag_blinky_on = false;




//===============================================================================================================
//		VENUS APP - Functions Definitions
//===============================================================================================================
uint32_t get_battery_level_100(){
	return g_uiBATTERY_level_100;
}

//-----------------------------------------------------------------
//			Nus Packet receiver / Parser
//-----------------------------------------------------------------

void on_nus_receive_parse(unsigned char * received_data, unsigned int length)
{
	if(received_data[0] == 0xFE) { // syncword
		switch(received_data[1]) {
			case 0x00 : // none
				break;
			case 0x01: // set motor state
				if( 0 < (received_data[2] & 0x02) ) {
					nrf_gpio_pin_set(GPIO_P17); // turn on left motor
				}
				else {
					nrf_gpio_pin_clear(GPIO_P17); // turn off left motor
				}

				if( 0 < (received_data[2] & 0x01) ) {
					nrf_gpio_pin_set(GPIO_P16);  // turn on right motor
				}
				else {
					nrf_gpio_pin_clear(GPIO_P16); // turn off right motor
				}
				break;
		}
		
	}

	
	//	test - motor set

#define MSG_GPIO_TURN_ON_LEFT		"turn on left"
#define MSG_GPIO_TURN_OFF_LEFT		"turn off left"
#define MSG_GPIO_TURN_ON_RIGHT		"turn on right"
#define MSG_GPIO_TURN_OFF_RIGHT		"turn off right"

#define MSG_GPIO_TURN_LEFT_ON		"turn left on"
#define MSG_GPIO_TURN_LEFT_OFF		"turn left off"
#define MSG_GPIO_TURN_RIGHT_ON		"turn right on"
#define MSG_GPIO_TURN_RIGHT_OFF		"turn right off"

//	left
#define MSG_GPIO_p17_ON				"p17 1"
#define MSG_GPIO_P17_ON				"P17 1"
#define MSG_GPIO_p17_OFF			"p17 0"
#define MSG_GPIO_P17_OFF			"P17 0"

//	right
#define MSG_GPIO_p16_ON				"p16 1"
#define MSG_GPIO_P16_ON				"P16 1"
#define MSG_GPIO_p16_OFF			"p16 0"
#define MSG_GPIO_P16_OFF			"P16 0"


	if( (memcmp(MSG_GPIO_TURN_ON_LEFT, received_data, 12) == 0) ||
		(memcmp(MSG_GPIO_TURN_LEFT_ON, received_data, 12) == 0) ||
		(memcmp(MSG_GPIO_p17_ON, received_data, 5) == 0) ||
		(memcmp(MSG_GPIO_P17_ON, received_data, 5) == 0) )
	{
		nrf_gpio_pin_set(GPIO_P17); // turn on LEFT motor
	}
	else if( (memcmp(MSG_GPIO_TURN_OFF_LEFT, received_data, 13) == 0) ||
			(memcmp(MSG_GPIO_TURN_LEFT_OFF, received_data, 13) == 0) ||
			(memcmp(MSG_GPIO_p17_OFF, received_data, 5) == 0) ||
			(memcmp(MSG_GPIO_P17_OFF, received_data, 5) == 0) )
	{
		nrf_gpio_pin_clear(GPIO_P17); // turn off LEFT motor
	}
	else if( (memcmp(MSG_GPIO_TURN_ON_RIGHT, received_data, 13) == 0) ||
			(memcmp(MSG_GPIO_TURN_RIGHT_ON, received_data, 13) == 0) ||
			(memcmp(MSG_GPIO_p16_ON, received_data, 5) == 0) ||
			(memcmp(MSG_GPIO_P16_ON, received_data, 5) == 0) )
	{
		nrf_gpio_pin_set(GPIO_P16); // turn on RIGHT motor
	}
	else if( (memcmp(MSG_GPIO_TURN_OFF_RIGHT, received_data, 14) == 0) ||
			(memcmp(MSG_GPIO_TURN_RIGHT_OFF, received_data, 14) == 0) ||
			(memcmp(MSG_GPIO_p16_OFF, received_data, 5) == 0) ||
			(memcmp(MSG_GPIO_P16_OFF, received_data, 5) == 0) )
	{
		nrf_gpio_pin_clear(GPIO_P16); // turn off RIGHT motor
	}
}


//-----------------------------------------------------------------
//			MEASURE SEAT PRESSURE
//-----------------------------------------------------------------

void reorder_seat_16ch_VENUS(uint8_t * buffer_16ch)
{
	//	Order is distorted.
	//	Scan order : 0, 1, 2, 3, 8, 9,  10, 11, 4, 5, 6, 7, 12, 13, 14, 15

	uint32_t ui_buffer_temp[NUM_CELLS_in4067];
	
	ui_buffer_temp[0] 		= buffer_16ch[0];
	ui_buffer_temp[1] 		= buffer_16ch[1];
	ui_buffer_temp[2] 		= buffer_16ch[2];
	ui_buffer_temp[3]		= buffer_16ch[3];
	ui_buffer_temp[4] 		= buffer_16ch[8];
	ui_buffer_temp[5] 		= buffer_16ch[9];
	ui_buffer_temp[6] 		= buffer_16ch[10];
	ui_buffer_temp[7] 		= buffer_16ch[11];
	ui_buffer_temp[8] 		= buffer_16ch[4];
	ui_buffer_temp[9] 		= buffer_16ch[5];
	ui_buffer_temp[10] 		= buffer_16ch[6];
	ui_buffer_temp[11]		= buffer_16ch[7];
	ui_buffer_temp[12] 		= buffer_16ch[12];
	ui_buffer_temp[13] 		= buffer_16ch[13];
	ui_buffer_temp[14] 		= buffer_16ch[14];
	ui_buffer_temp[15] 		= buffer_16ch[15];

	for(int i = 0 ; i < NUM_CELLS_in4067 ; i++){
		buffer_16ch[i] = ui_buffer_temp[i];
	}
}

void reorder_compensate_seat_32ch(void)
{
	reorder_seat_16ch_VENUS(g_ucCellData32ch);
	reorder_seat_16ch_VENUS(g_ucCellData32ch + NUM_CELLS_in4067);
}


void measure_adc_seat_32ch(void)
{
	unsigned int adc_raw_value = 0;

	//	reset buffer
	memset(g_ucCellData32ch, 0, VENUS_SCAN_CELL_NUM);

	//--------------------------------------------------------------------------------
	//	scan main block ==> 0~15
	nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_2);
	for(unsigned int ic4067_outpin_id = 0 ; ic4067_outpin_id < NUM_CELLS_in4067 ; ic4067_outpin_id++){
		config_4067_single_ch(ic4067_outpin_id);
		
		adc_raw_value = ADC_IRQ_ConfigAndGetValue_10bit_Delay81us();
		g_ucCellData32ch[ic4067_outpin_id] = adc_raw_value >> 2;  // make 10bit to 8bit
	}

	//--------------------------------------------------------------------------------
	//	scan sub block ==> 16~31
	nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_3);
	for(unsigned int ic4067_outpin_id = 0 ; ic4067_outpin_id < NUM_CELLS_in4067 ; ic4067_outpin_id++) {
		config_4067_single_ch(ic4067_outpin_id);
		
		adc_raw_value = ADC_IRQ_ConfigAndGetValue_10bit_Delay81us();
		g_ucCellData32ch[NUM_CELLS_in4067 + ic4067_outpin_id] = adc_raw_value >> 2;  // make 10bit to 8bit
	}
	
	
	//--------------------------------------------------------------------------------
	//	Compensate 6 large cells of seat sensor ==> 4 /5
#define COMPENSATE_LARGE_CELL(v)	(v * 4 / 5)

	if(g_DIPSWITCH_state[DIP_1] == true) {	// true : seat, false : 32ch
		for(unsigned int cell_index = 0 ; cell_index < VENUS_SCAN_CELL_NUM ; cell_index++) {
			if( (cell_index == 6) 	|| (cell_index == 12) 	|| (cell_index == 14) || 
				(cell_index == 16) 	|| (cell_index == 18)	|| (cell_index == 24) ) {
					
				g_ucCellData32ch[cell_index] = COMPENSATE_LARGE_CELL(g_ucCellData32ch[cell_index]); //	reduce large cell's value.
				//g_ucCellData32ch[cell_index] = 100 + cell_index; //	verify cell index
			}
		}
	}

	//--------------------------------------------------------------------------------
	//	Trimming to 7bit, ==> max 127
	for(unsigned int cell_index = 0 ; cell_index < VENUS_SCAN_CELL_NUM ; cell_index++) {
		if( ADC_TRIM_VALUE_7bit < g_ucCellData32ch[cell_index] ){
			g_ucCellData32ch[cell_index] = ADC_TRIM_VALUE_7bit;
		}
	}

	//--------------------------------------------------------------------------------
	//	Summation
	g_adc_32ch_sum = 0;
	g_adc_32ch_valid_count = 0;
	
	for(unsigned int cell_index = 0 ; cell_index < VENUS_SCAN_CELL_NUM ; cell_index++) {
		//	sum overall adc values
		g_adc_32ch_sum += g_ucCellData32ch[cell_index];

		if( THRESHOLD_VALID_CELL_CUTOFF < g_ucCellData32ch[cell_index]) {
			g_adc_32ch_valid_count++;
		}
	}

}


void measure_adc_battery_level()
{
	unsigned int adc_val_battery = 0;
	nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_7);

	//	get adc value - half of battery voltage
	adc_val_battery = ADC_IRQ_ConfigAndGetValue_10bit_Delay81us();
	
	//	battery raw data, sehjin12
	g_uc_battery_high = adc_val_battery / 100 + 100;
	g_uc_battery_low = adc_val_battery % 100;

	//	compensation
	{
#define	BATTERY_ADC_10bit_ZERO			408
#define	BATTERY_ADC_10bit_CUTOFF		475
#define	BATTERY_ADC_10bit_FULL			575
		
#define	BATTERY_LEVEL_DISPLAY_MAX		100

		g_uiBATTERY_level_100 = adc_val_battery - BATTERY_ADC_10bit_CUTOFF;		
	}

	//	trimming adc value
	if(100 < g_uiBATTERY_level_100) {
		g_uiBATTERY_level_100 = 100;
	}

}		

//-----------------------------------------------------------------
//			MAKE NUS PACKET
//-----------------------------------------------------------------

void venus_packet_send(unsigned char * packet_data)
{
	nus_send_data_20(packet_data, NUS_PACKET_MAX_SIZE);
}

void venus_nus_packet_assemble_send(IC4067BLOCK_ID block_id)
{
	uint8_t * data_32ch = g_ucCellData32ch;
	
	memset(g_ucNUS_PacketBuffer, 0, NUS_PACKET_MAX_SIZE);
	
	if(block_id == ID_BLOCK_MAIN) {	//	main 16 channel of 32
		data_32ch = g_ucCellData32ch;
	}
	else { // sub 16 channel of 32
		data_32ch = g_ucCellData32ch + NUM_CELLS_in4067;
	}

	//	offset 0 : Header
	if( (g_DIPSWITCH_state[DIP_1] == false) && (block_id == ID_BLOCK_MAIN) ) {
		g_ucNUS_PacketBuffer[0] = 'M';
	}
	else if( (g_DIPSWITCH_state[DIP_1] == false) && (block_id == ID_BLOCK_SUB) ){
		g_ucNUS_PacketBuffer[0] = 'S';
	}
	else if( (g_DIPSWITCH_state[DIP_1] == true) && (block_id == ID_BLOCK_MAIN) ){
		g_ucNUS_PacketBuffer[0] = 'L';
	}
	else if( (g_DIPSWITCH_state[DIP_1] == true) && (block_id == ID_BLOCK_SUB) ){
		g_ucNUS_PacketBuffer[0] = 'R';
	}
	else{
		g_ucNUS_PacketBuffer[0] = 'X';
	}

	
	//	offset 1 ~ 16 : Scanned data
#define	LENGTH_OF_HEADER			1	
	for(int offset = 0 ; offset < NUM_CELLS_in4067 ; offset++){
		g_ucNUS_PacketBuffer[offset + LENGTH_OF_HEADER] = data_32ch[offset];
	}
	
	//	sehjin12 battery
	//g_ucNUS_PacketBuffer[15] = g_uc_battery_high;
	//g_ucNUS_PacketBuffer[16] = g_uc_battery_low;
	
	//	offset 17 : battery
	g_ucNUS_PacketBuffer[NUM_CELLS_in4067 + LENGTH_OF_HEADER] = g_uiBATTERY_level_100;
	
	//	offset 18 : empty

	//	offset 19 : dip switch state bit field
	g_ucNUS_PacketBuffer[19] = 0; // initialize
	g_ucNUS_PacketBuffer[19] = (g_DIPSWITCH_state[0] << 7) | (g_DIPSWITCH_state[1] << 6) | (g_DIPSWITCH_state[2] << 5);
	
	//	send packet
	venus_packet_send(g_ucNUS_PacketBuffer);
}


void work_gatt_uart(uint32_t loop_count)
{
	bool blinky_work = g_timer_flag_blinky_on;
	g_timer_flag_blinky_on = false;
	
	//	LED message, Blinky while scanning.
	if(blinky_work == true) {
		app_led_turn_on(true);
	}
	
	measure_adc_battery_level();
	measure_adc_seat_32ch();
	if(g_DIPSWITCH_state[DIP_1] == true){	//	if 32 channel mode X, seat mode O
		reorder_compensate_seat_32ch();
	}
	
	//	make and send packet
	venus_nus_packet_assemble_send(ID_BLOCK_MAIN);
	venus_nus_packet_assemble_send(ID_BLOCK_SUB);

	//	turn off led
	if(blinky_work == true) {
		app_led_turn_on(false);
	}
}

//-----------------------------------------------------------------
//			Update Advertisement
//-----------------------------------------------------------------

void venus_adv_manufact_data_update()
{
	memset(g_adv_data_packet, 0x00, PACKET_ADV_LEN);
	g_adv_data_packet[0] = 10;
	g_adv_data_packet[1] = g_adc_32ch_sum / 100;
	g_adv_data_packet[2] = g_adc_32ch_sum % 100;
	g_adv_data_packet[3] = g_adc_32ch_valid_count;
	g_adv_data_packet[4] = g_uiBATTERY_level_100;

	advertisement_update_packet(g_adv_data_packet, PACKET_ADV_LEN);
}


void work_adv_update(uint32_t loop_count)
{
	measure_adc_battery_level();
	measure_adc_seat_32ch();

	venus_adv_manufact_data_update();
}


//===============================================================================================================
//		VENUS APP Basic functions
//===============================================================================================================
APP_TIMER_DEF(m_timer_id_scan32ch);		/**< Venus timer - scan 32ch. */
#define	TIMER_IVL_SCAN32CH_SLOW		APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) 	/**< Timer interval (ms, ticks). 1~3500 tested*/
#define TIMER_IVL_SCAN32CH_FAST		APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) 	/**< Timer interval (ms, ticks). 1~3500 tested*/

APP_TIMER_DEF(m_timer_id_nus_blinky);		/**< Venus timer - scan 32ch. */
#define	TIMER_IVL_NUS_BLINKY		APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) 	/**< Timer interval (ms, ticks). 1~3500 tested*/

//APP_TIMER_DEF(m_timer_id_app_test);		/**< Venus timer - scan 32ch. */
//#define	TIMER_IVL_APP_TEST			APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER) 	/**< Timer interval (ms, ticks). 1~3500 tested*/

void task_timeout_handler_app_test(void * p_context)
{
	static uint32_t g_timer_counter_app_test = 0;

	g_timer_counter_app_test++;
}


void task_timeout_handler_nus_blinky(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	
	g_timer_flag_blinky_on = true;
}

bool g_timer_flag_scan_32ch = false;

void task_timeout_handler_scan_32ch(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	
	g_timer_flag_scan_32ch = true;
}


void task_timer_config(void)
{
    // Initialize timer module.
	//	This function should be called after "APP_TIMER_INIT"
	//	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false); // ==> called in main()

	uint32_t err_code;

	// Create timers.
    err_code = app_timer_create(&m_timer_id_scan32ch,
                                APP_TIMER_MODE_REPEATED,
                                task_timeout_handler_scan_32ch);
    APP_ERROR_CHECK(err_code);

	if(get_dipsw_state(DIP_0_0UART_1ADV) == false) {
		err_code = app_timer_create(&m_timer_id_nus_blinky,
									APP_TIMER_MODE_REPEATED,
									task_timeout_handler_nus_blinky);
		APP_ERROR_CHECK(err_code);
	}

/*	
    err_code = app_timer_create(&m_timer_id_app_test,
                                APP_TIMER_MODE_REPEATED,
                                task_timeout_handler_app_test);
    APP_ERROR_CHECK(err_code);
*/
}

void app_main_timer_work(void)
{
	if(g_timer_flag_scan_32ch == false)
		return;
	//	fall through only when g_timer_flag_scan_32ch == true

	g_timer_flag_scan_32ch = false;
	
	static uint32_t app_main_loop_count = 0;

	if(get_dipsw_state(DIP_0_0UART_1ADV) == false) {
		if(is_ble_service_connected() == true) {
			//	nus service
			work_gatt_uart(app_main_loop_count);
		}
	}
	else {
		//	adv update
		work_adv_update(app_main_loop_count);
	}

	app_main_loop_count++;

}

void app_main_start(void)
{
	config_HW();
	
	task_timer_config();

	uint32_t err_code;

	// Start application timers.
	err_code = app_timer_start(m_timer_id_scan32ch, TIMER_IVL_SCAN32CH_FAST, NULL);
	APP_ERROR_CHECK(err_code);

	if(get_dipsw_state(DIP_0_0UART_1ADV) == false) {
		// Start nus blinky timers.
		err_code = app_timer_start(m_timer_id_nus_blinky, TIMER_IVL_NUS_BLINKY, NULL);
		APP_ERROR_CHECK(err_code);
	}

	//err_code = app_timer_start(m_timer_id_app_test, TIMER_IVL_APP_TEST, NULL);
	//APP_ERROR_CHECK(err_code);

}


