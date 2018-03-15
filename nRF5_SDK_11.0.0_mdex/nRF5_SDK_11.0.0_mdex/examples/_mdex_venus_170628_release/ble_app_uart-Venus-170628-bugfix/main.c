/** @file
 * @brief    VENUS application main file.
 */
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include "nrf_adc.h"
#include "nrf_gpio.h"
#include "bsp.h"
#include "app_util_platform.h"

#include "service_51822.h"
#include "main.h"
 
//	FURNISTEM 비너스 예전 버전용 (딥스위치 없는 보드) without dip switch for furnistem 
 //#define	FORCE_ADV_MODE_FURNISTEM	


//	<<  Program Softdevice (S130) of Nordic 51822AA for this project  >>
//	 C:/Keil_v5/ARM/nRF5_SDK_11.0.0/components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex
 

/*
	<< TODO >>
		BLE bonding mode


	<< UPDATE >>
	170713 : 
		Version 1.1.1c
		CHANGE from ALL of advertising_init(); to 	advertising_init_forever(); of Data communication mode

		
	170619 : 
		Version -> 1.1.1b
		Bug fix
		CHANGE from advertising_init(); to 	advertising_init_forever(); of Data communication mode

	170510 : 
		Version 1.1.1 
		Converting ADV / UART by DIS switch
		매크로 버그 수정 : 
		#define	FORCE_ADV_MODE_FURNISTEM	TRUE // ==> 빌드 옵션으로 오류.
		#define	FORCE_ADV_MODE_FURNISTEM

	170412 : 04a1
		Battery level working good
		Set ADC resolution 10bit
		ADC display ranges : 0 ~ 127. Seems like 7bit
		Reads GPIO of DIP SWITCH.
		Receive commands via BLE from Phone
		Reduces ADC value of Row 0 ==> 80%
		Changes LED working scenario ==> 0: Send, 1: Receive
		
	170407 : 02c3
		Battery working mode, 10bit
		
	170211 : 0.2c2
		ADC resolution : 10bit to 8bit
		Change protocol (adc value : 0~ 77)
		Batttery test mode (not working)
		
	170101 : 0.2c1
		Text coordination. Variables are conceptualized. BLE functions are separated to service_51822.c
		Data - TX and RX
		Battery - it's not correct. It dies at 67%.

	161230 : 0.2ba
		Timer1 - timer1_loop_main is added
		Power save, hibernation
	
	161210 : Battery level display --> 0.2a
		Power - power down by low battery

	<< TODO >>
		Auto reconnection
		PERI - check input of reset button by IRQ.
		Seat - find COM, COC, change protocol consist of COM, COC
		OTA firmware update
		> Connection - re conn, auto conn, quick disconn
		> RTC, Timer global balance setting (https://devzone.nordicsemi.com/question/54481/timer-prescaler/)
		Buffer reordering
		Dont' send packet while empty
*/


///////////////////////////////////////////////////////////////////////////////////
//
//			<--		USER DECLARATION
//			--> 	HW CONFIG FUNCTIONS
//
///////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------
//			POWER MANAGEMENT
//-----------------------------------------------------------------

void gotoSystemPowerOff()
{
	NRF_POWER->SYSTEMOFF = 1;
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


//-----------------------------------------------------------------
//			CONFIG  -  TIMER
//-----------------------------------------------------------------
const nrf_drv_timer_t TIMER_INSTANCE_SCAN_ADC = NRF_DRV_TIMER_INSTANCE(1); // <<--- TIMER ID

/**
 * @brief Handler for timer events.
 */
void timer_mainloop_handler(nrf_timer_event_t event_type, void* p_context)
{
	static uint32_t handler_count = 0;
	
	switch(event_type)
	{
		case NRF_TIMER_EVENT_COMPARE0: // <<--- TIMER ID
			break;
		
		case NRF_TIMER_EVENT_COMPARE1:
			timer1_loop_main();
			break;
		
		default :
			break;
	}
	handler_count++;
}
/**
 * @brief Function for timer IRQ test
 */
void config_timer1_mainloop(void)
{
	//	Parameter 'time_ms' is not working.
	//uint32_t time_ms = 500; //Time(in miliseconds) between consecutive compare events.
	//	sehjin12 test for battery off
	uint32_t time_ms = 100; //Time(in miliseconds) between consecutive compare events.

	uint32_t time_ticks;
	uint32_t err_code = NRF_SUCCESS;

	//Configure TIMER_INSTANCE_SCAN_ADC for generating simple light effect - leds on board will invert his state one after the other.
	err_code = nrf_drv_timer_init(&TIMER_INSTANCE_SCAN_ADC, NULL, timer_mainloop_handler);
	APP_ERROR_CHECK(err_code);

	time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_INSTANCE_SCAN_ADC, time_ms);

	nrf_drv_timer_extended_compare(&TIMER_INSTANCE_SCAN_ADC, NRF_TIMER_CC_CHANNEL1, 
			time_ticks, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true); // <<--- TIMER ID

	nrf_drv_timer_enable(&TIMER_INSTANCE_SCAN_ADC);
}


//-----------------------------------------------------------------
//			CONFIG AND CONTROL  -  LED
//-----------------------------------------------------------------

void turn_led_on(int id)
{
	uint32_t led_to_turn = (1 << g_LED_HW_pin_map[id]);
	LEDS_ON(led_to_turn);
}

void turn_led_off(int id)
{
	uint32_t led_to_turn = (1 << g_LED_HW_pin_map[id]);
	LEDS_OFF(led_to_turn);
}

/**
 * @brief Toggle LED
 */
void toggle_led(int id)
{
	uint32_t led_to_turn = (1 << g_LED_HW_pin_map[id]);
	LEDS_INVERT(led_to_turn);
}

void blink_led_save(int id)
{
	uint32_t led_to_turn = (1 << g_LED_HW_pin_map[id]);

	for(int i = 0 ; i < 4000 ; i++){
		LEDS_ON(led_to_turn);
		for(int j = 0 ; j < 4; j++){
		}
		LEDS_OFF(led_to_turn);
		for(int j = 0 ; j < 8; j++){
		}
	}
}

void blink_double_led_save()
{
	uint32_t led_to_turn_0 = (1 << g_LED_HW_pin_map[0]);
	uint32_t led_to_turn_1 = (1 << g_LED_HW_pin_map[1]);

	for(int i = 0 ; i < 4000 ; i++){
		LEDS_ON(led_to_turn_0);
		LEDS_ON(led_to_turn_1);
		for(int j = 0 ; j < 4; j++){
		}
		LEDS_OFF(led_to_turn_0);
		LEDS_OFF(led_to_turn_1);
		for(int j = 0 ; j < 8; j++){
		}
	}
}


void config_venus_led_pins(void)
{    
	//Configure all leds on board.
	LEDS_CONFIGURE(LEDS_MASK_ALL);
	LEDS_OFF(LEDS_MASK_ALL);

	//	old style
	nrf_gpio_cfg_output(LED_0);

}

//-----------------------------------------------------------------
//			CONFIG  -  ADC
//-----------------------------------------------------------------
#define NRF_ADC_CONFIG_DEFAULT_8bit { NRF_ADC_CONFIG_RES_8BIT,               \
                                 NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD, \
                                 NRF_ADC_CONFIG_REF_VBG }

void ADC_IRQHandler(void)
{
	nrf_adc_conversion_event_clean();
	g_uiADC_IRQ_HW_read = nrf_adc_result_get();
}


unsigned int ADC_IRQ_ConfigAndGetValue_10bit_Delay81us(void)
{
	nrf_delay_us(1);
	g_uiADC_IRQ_HW_read = 0;  
	nrf_adc_start();
	nrf_delay_us(80); //adc 10 bit  convert time 68uS  ,,, 8bit 32uS

	return (g_uiADC_IRQ_HW_read); 
}

unsigned int ADC_IRQ_ConfigAndGetValue_8bit_Delay33us_not_use(void)
{
	nrf_delay_us(1);
	g_uiADC_IRQ_HW_read = 0;  
	nrf_adc_start();
	nrf_delay_us(32); //adc 8 bit  convert time 68uS  ,,, 8bit 32uS

	return (g_uiADC_IRQ_HW_read); 
}


void config_adc_each_port(int adc_index)
{
	//	check if wrong port
	switch(adc_index){
		case ADC_PIN_MAIN:
		case ADC_PIN_SUB:
		case ADC_PIN_BATTERY:
			break;
		default:
			return;
	}
	
    const nrf_adc_config_t nrf_adc_config = NRF_ADC_CONFIG_DEFAULT;

    // Initialize and configure ADC
	//nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
	{
		//nrf_adc_config_t * config = &nrf_adc_config;
		
		uint32_t config_reg = 0;
		
		config_reg |= ((uint32_t)nrf_adc_config.resolution << ADC_CONFIG_RES_Pos) & ADC_CONFIG_RES_Msk;
		config_reg |= ((uint32_t)nrf_adc_config.scaling << ADC_CONFIG_INPSEL_Pos) & ADC_CONFIG_INPSEL_Msk;
		config_reg |= ((uint32_t)nrf_adc_config.reference << ADC_CONFIG_REFSEL_Pos) & ADC_CONFIG_REFSEL_Msk;
		
		if (nrf_adc_config.reference & ADC_CONFIG_EXTREFSEL_Msk)
		{
			config_reg |= nrf_adc_config.reference & ADC_CONFIG_EXTREFSEL_Msk;
		}
		
		/* select input */
		nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_DISABLED);
		
		/* set new configuration keeping selected input */
		NRF_ADC->CONFIG = config_reg | (NRF_ADC->CONFIG & ADC_CONFIG_PSEL_Msk);
	}
	
	switch(adc_index){
		case ADC_PIN_MAIN:
			nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_2);
			break;
		case ADC_PIN_SUB:
			nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_3);
			break;
		case ADC_PIN_BATTERY:
			nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_7);
			break;
	}

	nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
	NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
	NVIC_EnableIRQ(ADC_IRQn);
}

void config_venus_adc_block(void)
{
	config_adc_each_port(ADC_PIN_BATTERY);	// adc for mercury/venus battery A7

	config_adc_each_port(ADC_PIN_MAIN); 		// adc for mercury/venus main board : A2
	config_adc_each_port(ADC_PIN_SUB); 		// adc for mercury/venus shield board : A3
}

//-----------------------------------------------------------------
//			CONFIG  -  4067
//-----------------------------------------------------------------

void config_venus_4067(void)
{
	nrf_gpio_cfg_output(DECODER_4067_S0);
	nrf_gpio_cfg_output(DECODER_4067_S1);
	nrf_gpio_cfg_output(DECODER_4067_S2);
	nrf_gpio_cfg_output(DECODER_4067_S3);
}

void config_4067_single_ch(unsigned int ch)
{
	nrf_gpio_pin_clear(DECODER_4067_S0);
	nrf_gpio_pin_clear(DECODER_4067_S1);
	nrf_gpio_pin_clear(DECODER_4067_S2);
	nrf_gpio_pin_clear(DECODER_4067_S3);

	if(ch & 0x01) nrf_gpio_pin_set(DECODER_4067_S0);
	if(ch & 0x02) nrf_gpio_pin_set(DECODER_4067_S1);
	if(ch & 0x04) nrf_gpio_pin_set(DECODER_4067_S2);
	if(ch & 0x08) nrf_gpio_pin_set(DECODER_4067_S3);
}

//-----------------------------------------------------------------
//			CONFIG  -  GPIO OUTPUT for Motor drive
//-----------------------------------------------------------------

void config_output_gpio_2pins()
{
	nrf_gpio_cfg_output(GPIO_P16);
	nrf_gpio_cfg_output(GPIO_P17);
}

//-----------------------------------------------------------------
//			CONFIG  -  GPIO INPUT for DIP switch. This uses just 3 of 4 switch
//-----------------------------------------------------------------

void config_input_gpio_3switches()
{
	//	config dip switch
	nrf_gpio_cfg_input(DIPSWITCH_0, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(DIPSWITCH_1, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(DIPSWITCH_2, NRF_GPIO_PIN_NOPULL);
}



///////////////////////////////////////////////////////////////////////////////////
//
//			<--		HW CONFIG FUNCTIONS
//			--> 	APPLICATION
//
///////////////////////////////////////////////////////////////////////////////////


//-----------------------------------------------------------------
//			MEASURE SEAT PRESSURE
//-----------------------------------------------------------------

void reorder_adc_scanned_buffer_VENUS(void)
{
#define BOARD_LABEL_BASE	1
	/*
	Order is distorted.
	Scan order : 1, 2, 3, 4, 9, 10, 11, 12, 5, 6, 7, 8, 13, 14, 15, 16
	
	*/
	uint32_t ui_buffer_temp[20];
	
	ui_buffer_temp[0] 		= g_uiADC_value_seat[1 - BOARD_LABEL_BASE];
	ui_buffer_temp[1] 		= g_uiADC_value_seat[2 - BOARD_LABEL_BASE];
	ui_buffer_temp[2] 		= g_uiADC_value_seat[3 - BOARD_LABEL_BASE];
	ui_buffer_temp[3]		= g_uiADC_value_seat[4 - BOARD_LABEL_BASE];
	ui_buffer_temp[4] 		= g_uiADC_value_seat[9 - BOARD_LABEL_BASE];
	ui_buffer_temp[5] 		= g_uiADC_value_seat[10 - BOARD_LABEL_BASE];
	ui_buffer_temp[6] 		= g_uiADC_value_seat[11 - BOARD_LABEL_BASE];
	ui_buffer_temp[7] 		= g_uiADC_value_seat[12 - BOARD_LABEL_BASE];
	ui_buffer_temp[8] 		= g_uiADC_value_seat[5 - BOARD_LABEL_BASE];
	ui_buffer_temp[9] 		= g_uiADC_value_seat[6 - BOARD_LABEL_BASE];
	ui_buffer_temp[10] 	= g_uiADC_value_seat[7 - BOARD_LABEL_BASE];
	ui_buffer_temp[11]	= g_uiADC_value_seat[8 - BOARD_LABEL_BASE];
	ui_buffer_temp[12] 	= g_uiADC_value_seat[13 - BOARD_LABEL_BASE];
	ui_buffer_temp[13] 	= g_uiADC_value_seat[14 - BOARD_LABEL_BASE];
	ui_buffer_temp[14] 	= g_uiADC_value_seat[15 - BOARD_LABEL_BASE];
	ui_buffer_temp[15] 	= g_uiADC_value_seat[16 - BOARD_LABEL_BASE];

	for(int i = 0 ; i < NUM_OF_HC4067_Y ; i++){
		g_uiADC_value_seat[i] = ui_buffer_temp[i];
	}
}

void measure_adc_seat_16ch(IC4067BLOCK_ID ic4067_id)
{
	switch(ic4067_id){
		case ID_BLOCK_MAIN:
			nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_2);
			break;
		case ID_BLOCK_SUB:
			nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_3);
			break;
	}

	//	g_DIPSWITCH_state[DIP_1] == true;
	//	measure 16ch output of 4067
	unsigned int adc_raw_value = 0;
	unsigned int adc_value_trimmed8 = 0;
	for(unsigned int ic4067_outpin_id = 0 ; ic4067_outpin_id < NUM_OF_HC4067_Y ; ic4067_outpin_id++)
	{
		config_4067_single_ch(ic4067_outpin_id);
		
		adc_raw_value = ADC_IRQ_ConfigAndGetValue_10bit_Delay81us();
		adc_value_trimmed8 = adc_raw_value >> 2; // make 10bit to 8bit
		
		if(g_DIPSWITCH_state[DIP_1] == false){	//	if venus is 32 channel mode
			g_uiADC_value_seat[ic4067_outpin_id] = adc_value_trimmed8; 
		}
		else{	//	if venus is seat sensor mode (31 channel)
			if(ic4067_id == ID_BLOCK_MAIN) {		// main block
				switch(ic4067_outpin_id){
					case 6: case 12: case 14: 
						g_uiADC_value_seat[ic4067_outpin_id] = adc_value_trimmed8 * 4 / 5; //	reduce large cell's gauge.
						break;
					default:
						g_uiADC_value_seat[ic4067_outpin_id] = adc_value_trimmed8; 
						break;
				}
			}
			else {		//	sub block
				switch(ic4067_outpin_id){
					case 0: case 2: case 8: 
						g_uiADC_value_seat[ic4067_outpin_id] = adc_value_trimmed8 * 4 / 5; //	reduce large cell's gauge.
						break;
					default:
						g_uiADC_value_seat[ic4067_outpin_id] = adc_value_trimmed8;
						break;
				}
			}
		}

	}

	
	//	Trimming to 7bit, 127
	for(unsigned int ic4067_outpin_id = 0 ; ic4067_outpin_id < NUM_OF_HC4067_Y ; ic4067_outpin_id++)
	{
		if( ADC_TRIM_VALUE_7bit < g_uiADC_value_seat[ic4067_outpin_id] ){
			g_uiADC_value_seat[ic4067_outpin_id] = ADC_TRIM_VALUE_7bit;
		}
	}

	
	reorder_adc_scanned_buffer_VENUS();
}



unsigned char g_uc_battery_high = 0;
unsigned char g_uc_battery_low = 0;

//battery volt read
void measure_adc_battery_half()
{
	unsigned int adc_val_battery = 0;
	nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_7);

	//	get adc value
	adc_val_battery = ADC_IRQ_ConfigAndGetValue_10bit_Delay81us();
	g_uiBATTERY_level_100 = adc_val_battery;
	
	//	sehjin12 battery test
	{
		g_uc_battery_high = adc_val_battery / 100 + 100;
		g_uc_battery_low = adc_val_battery % 100;
	}
	
	{
#define	BATTERY_ADC_10bit_ZERO			408
#define	BATTERY_ADC_10bit_CUTOFF			475
#define	BATTERY_ADC_10bit_FULL				575
		
#define	BATTERY_LEVEL_DISPLAY_MAX		100


		g_uiBATTERY_level_100 = adc_val_battery - BATTERY_ADC_10bit_CUTOFF;
		
	}	

	//	trimming adc value
	if(100 < g_uiBATTERY_level_100)
		g_uiBATTERY_level_100 = 100;

}		

void check_battery_to_off(int threshold_battery_off)
{
	if(g_uiBATTERY_level_100 < threshold_battery_off) {
		gotoSystemPowerOff();
	}
}

//-----------------------------------------------------------------
//			MAKE BLE PACKET
//-----------------------------------------------------------------
void venus_packet_clear()
{
	memset(g_ucNUS_PacketBuffer, 0, 20);
}

void venus_packet_assemble(IC4067BLOCK_ID block_id)
{
	//	offset 0 : Header
	if(block_id == ID_BLOCK_MAIN) {	//	main 16 channel of 32
		if(g_DIPSWITCH_state[DIP_1] == false){	//	if venus is 32 channel mode
			g_ucNUS_PacketBuffer[0] = 'M';
		}
		else{
			g_ucNUS_PacketBuffer[0] = 'L';
		}
	}
	else {		//	sub 16 channel of 32
		if(g_DIPSWITCH_state[DIP_1] == false){	//	if venus is 32 channel mode
			g_ucNUS_PacketBuffer[0] = 'S';
		}
		else {
			g_ucNUS_PacketBuffer[0] = 'R';
		}
	}
	
	//	offset 1 ~ 16 : Scanned data
#define	LENGTH_OF_HEADER			1	
	for(int offset = 0 ; offset < NUM_OF_HC4067_Y ; offset++){
		g_ucNUS_PacketBuffer[offset + LENGTH_OF_HEADER] = g_uiADC_value_seat[offset];

		
		//	sum overall adc values
		g_seat_adc_sum += g_uiADC_value_seat[offset];

		if( THRESHOLD_VALID_CELL_CUTOFF < g_uiADC_value_seat[offset]) {
			g_seat_adc_valid_count++;
		}
	}
	
	//	sehjin12 battery
	//g_ucNUS_PacketBuffer[15] = g_uc_battery_high;
	//g_ucNUS_PacketBuffer[16] = g_uc_battery_low;
	
	//	offset 17 : battery
	g_ucNUS_PacketBuffer[NUM_OF_HC4067_Y + LENGTH_OF_HEADER] = g_uiBATTERY_level_100;
	
	//	offset 18 : empty

	//	offset 19 : dip switch state bit field
	g_ucNUS_PacketBuffer[19] = 0; // initialize
	g_ucNUS_PacketBuffer[19] = (g_DIPSWITCH_state[0] << 7) | (g_DIPSWITCH_state[1] << 6) | (g_DIPSWITCH_state[2] << 5);
}

void venus_packet_send(unsigned char * packet_data)
{
	nus_send_data_20(packet_data, NUS_PACKET_MAX_SIZE);
}


///////////////////////////////////////////////////////////////////////////////////
//
//			<--		APPLICATION
//			--> 	MAIN LOOP
//
///////////////////////////////////////////////////////////////////////////////////
void scan_adc_all_and_send_via_BLE(void);


void DIP_SWITCH_check_state()
{
	uint32_t pin_state_inv[DIPSWITCH_NUMBER];
	memset(pin_state_inv, 0, DIPSWITCH_NUMBER);

	pin_state_inv[DIP_0_0UART_1ADV] = nrf_gpio_pin_read(DIPSWITCH_0); // if return true --> switch ON, return false --> switch OFF
	pin_state_inv[DIP_1_0FAST_1SLOW] = nrf_gpio_pin_read(DIPSWITCH_1); // if return true --> switch ON, return false --> switch OFF
	
	for(int switch_index = 0 ; switch_index < DIPSWITCH__USE_NUMBER ; switch_index++){
		
		if(pin_state_inv[switch_index] == true) { // switch is OFF
			g_DIPSWITCH_state[switch_index]  = false;
		} 
		else  { // switch is ON
			g_DIPSWITCH_state[switch_index]  = true;
		}
	}
}


void config_HW(void)
{
	//	config core bsp
	config_bsp_nus();
	config_input_gpio_3switches();

	//	config peripheral and timer handler
	config_timer1_mainloop();
	
	config_venus_led_pins();
	config_venus_adc_block();
	config_venus_4067();
	config_output_gpio_2pins();

	//	read HW setting
	DIP_SWITCH_check_state();
#ifdef FORCE_ADV_MODE_FURNISTEM
	g_DIPSWITCH_state[DIP_0_0UART_1ADV] = true;
#endif
	
	g_DIPSWITCH_state[DIP_0_0UART_1ADV] = false;

	if(g_DIPSWITCH_state[DIP_0_0UART_1ADV] == false) {
		config_gap_param_and_init_nus();
	}
	else {
		config_gap_param_and_init_advertise();
	}
}


/**@brief Application main function.
 */
int main(void)
{
	config_HW();
	
	//	test - sehjin12 - battery test
	//turn_led_on(0);
	nrf_gpio_cfg_output(LED_0);
	
	while(1){};


	
	//	test - read dip switch
	if(false)
	{
		if(g_DIPSWITCH_state[DIP_0_0UART_1ADV] == true)
			turn_led_on(1);
		else
			turn_led_off(1);
	}


	//	main loop
	unsigned int loop_cnt = 0;	
	while(true)
	{
		__WFI(); // Wait for interrupts... It's not predicable. Doesn't wait just for Timer1 but also Timer0 of BLE stack and other interrupts.
		power_manage();

		check_battery_to_off(BATTERY_LEVEL_OFF);
		
		//nrf_delay_ms(10); //-- delete here		
		loop_cnt++;
	}
}

void scan_adc_half(IC4067BLOCK_ID block_id)
{	
	//	Step 1) measure one block  & battery
	measure_adc_seat_16ch(block_id);
	measure_adc_battery_half();
	
	//	make packet
	venus_packet_clear();
	venus_packet_assemble(block_id);
}


void scan_adc_all_and_send_via_BLE(void)
{	
	//	Step 1) measure main block  & battery
	measure_adc_seat_16ch(ID_BLOCK_MAIN);
	measure_adc_battery_half();
	
	//	make packet
	venus_packet_clear();
	venus_packet_assemble(ID_BLOCK_MAIN);
	venus_packet_send(g_ucNUS_PacketBuffer);

	
	//	Step 2) measure sub block 
	measure_adc_seat_16ch(ID_BLOCK_SUB);

	//	make packet
	venus_packet_clear();
	venus_packet_assemble(ID_BLOCK_SUB);
	venus_packet_send(g_ucNUS_PacketBuffer);
}

void on_BLE_receive(unsigned char * received_data, unsigned int length)
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
#define MSG_GPIO_TURN_OFF_RIGHT	"turn off right"

#define MSG_GPIO_TURN_LEFT_ON		"turn left on"
#define MSG_GPIO_TURN_LEFT_OFF		"turn left off"
#define MSG_GPIO_TURN_RIGHT_ON		"turn right on"
#define MSG_GPIO_TURN_RIGHT_OFF	"turn right off"

//	left
#define MSG_GPIO_p17_ON				"p17 1"
#define MSG_GPIO_P17_ON				"P17 1"
#define MSG_GPIO_p17_OFF				"p17 0"
#define MSG_GPIO_P17_OFF				"P17 0"

//	right
#define MSG_GPIO_p16_ON				"p16 1"
#define MSG_GPIO_P16_ON				"P16 1"
#define MSG_GPIO_p16_OFF				"p16 0"
#define MSG_GPIO_P16_OFF				"P16 0"


	//turn_led_on(1);

	if( 	(memcmp(MSG_GPIO_TURN_ON_LEFT, received_data, 12) == 0) ||
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
	

	nrf_delay_ms(5);

	turn_led_off(1);	
}


/*
	Release history
17-03-30 : Threshold 3종 출시 : 80, 140, 200 (기존 40은 너무 낮아서 불만이라는 반응)
*/

#define	THRESHOLD_WARNING	(140 << 1)


void work_adv_broadcast(uint32_t loop_count)
{
/*
	if( THRESHOLD_WARNING < g_seat_adc_sum ) {
		advertisement_update_switch_forever(true);
	}
	else {
		advertisement_update_switch_forever(false);
	}
*/
	if( (loop_count % 2) == 0) {
		//	initialize value
		g_seat_adc_sum = 0;
		g_seat_adc_valid_count = 0;
		memset(g_adv_data_packet, 0x00, PACKET_ADV_LEN);

		scan_adc_half(ID_BLOCK_MAIN);
	}
	else {
		scan_adc_half(ID_BLOCK_SUB);

		g_adv_data_packet[0] = 10;
		g_adv_data_packet[1] = g_seat_adc_sum / 100;
		g_adv_data_packet[2] = g_seat_adc_sum % 100;
		g_adv_data_packet[3] = g_seat_adc_valid_count;
		g_adv_data_packet[4] = g_uiBATTERY_level_100;

		advertisement_update_packet(g_adv_data_packet, PACKET_ADV_LEN);
	}

	//advertisement_update_packet(g_ucNUS_PacketBuffer, NUS_PACKET_MAX_SIZE);


	blink_led_save(0); // 0 : led #0

}

void work_gatt_uart(uint32_t loop_count)
{
	g_seat_adc_sum = 0;
	scan_adc_all_and_send_via_BLE();

	return;
	
	//	test - commentiize led off
	if(BLE_get_connection_state() == false){
		if( (loop_count %5) == 0){
			blink_led_save(0); // 0 : led #0
		}
	}
	else{
		blink_led_save(0); // 0 : led #0
		//blink_double_led_save();
	}
}


#define DO_ONCE_PER(_cnt, per)\
			((_cnt % per) == 0)

/**@brief timer1 loop main. don't place this function to main. __WFI is not called predicably
 */
void timer1_loop_main(void)
{
	static uint32_t valid_count = 0;

	g_main_loop_count++;
	//	tune timing tempo
	if( DO_ONCE_PER(g_main_loop_count, 2) ){
		return;
	}
	

	g_DIPSWITCH_state[DIP_0_0UART_1ADV] = false;
	if(g_DIPSWITCH_state[DIP_0_0UART_1ADV] == false)
		work_gatt_uart(valid_count);
	else {
		work_adv_broadcast(valid_count);	
	}
	
	valid_count++;

}




/** 
 * @}
 */



