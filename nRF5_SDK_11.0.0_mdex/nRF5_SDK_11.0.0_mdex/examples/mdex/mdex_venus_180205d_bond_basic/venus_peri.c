
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "boards.h"

#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
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



//===============================================================================================================
//		VENUS PERI - Variables, Declarations
//===============================================================================================================

volatile uint32_t g_uiADC_IRQ_HW_read;


bool g_DIPSWITCH_state[DIPSWITCH_NUMBER];

//===============================================================================================================
//		VENUS PERI - Functions Definitions
//===============================================================================================================

//-----------------------------------------------------------------
//			GET/SET
//-----------------------------------------------------------------

bool get_dipsw_state(int dip_sw_index){
	dipsw_scan_state();
	return g_DIPSWITCH_state[dip_sw_index];
}

//-----------------------------------------------------------------
//			CONFIG  -  UART
//-----------------------------------------------------------------
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handler(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;
	
	static uint8_t	tx_buffer_len = 0;
	static bool	tx_buffer_out = false;

#ifdef BOARD_VENUS
	return;
#endif	
	
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;
		
			tx_buffer_len++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN))) {
				err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE) {
					APP_ERROR_CHECK(err_code);
                }
				
				if(data_array[index - 1] == '\n') {
					tx_buffer_out = true;
				}

				index = 0;
            }
			break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
			printf("[error]ble comm\n");
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
			printf("[error]uart fifo\n");
            break;

        default:
            break;
    }
	
	//	log
	if(tx_buffer_out == true){
		printf("send via ble ok! (size=%d)\n",tx_buffer_len);
		tx_buffer_len = 0;
		tx_buffer_out = false;
	}

	return;
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
		APP_UART_FLOW_CONTROL_DISABLED,
        //APP_UART_FLOW_CONTROL_ENABLED, // deleted by sehjin12
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


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


uint32_t ADC_IRQ_ConfigAndGetValue_10bit_Delay81us(void)
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
		
		if (nrf_adc_config.reference & ADC_CONFIG_EXTREFSEL_Msk) {
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


void config_nrf_gpio_adc_3pins_sensor_bat(void)
{
	config_adc_each_port(ADC_PIN_BATTERY);	// adc for mercury/venus battery A7

	config_adc_each_port(ADC_PIN_MAIN); 		// adc for mercury/venus main board : A2
	config_adc_each_port(ADC_PIN_SUB); 		// adc for mercury/venus shield board : A3
}


//-----------------------------------------------------------------
//			CONFIG  -  GPIO
//-----------------------------------------------------------------
void app_led_turn_on(bool is_turn_on)
{
	if(is_turn_on == true) {
#if defined(BOARD_VENUS)
		nrf_drv_gpiote_out_high(TEST_GPIO_PIN_ID);
#elif defined(BOARD_PCA10028)
		nrf_drv_gpiote_out_low(TEST_GPIO_PIN_ID);
#endif 
	}
	else {
#if defined(BOARD_VENUS)
		nrf_drv_gpiote_out_low(TEST_GPIO_PIN_ID);
#elif defined(BOARD_PCA10028)
		nrf_drv_gpiote_out_high(TEST_GPIO_PIN_ID);
#endif 
	}
}

void app_led_toggle(void)
{
	nrf_drv_gpiote_out_toggle(TEST_GPIO_PIN_ID); // LED Toggle
}


void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	switch(action) {
		case GPIOTE_CONFIG_POLARITY_LoToHi:
			break;
		case GPIOTE_CONFIG_POLARITY_HiToLo:
			break;
		case GPIOTE_CONFIG_POLARITY_Toggle:
			break;
		default:
			break;
	}
}


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output, 
 * and configures GPIOTE to give an interrupt on pin change.
 */
void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
	//-		GPIO OUT
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);

    err_code = nrf_drv_gpiote_out_init(TEST_GPIO_PIN_ID, &out_config);
    APP_ERROR_CHECK(err_code);

	app_led_turn_on(false);


	//-		GPIO IN
	/*
    //nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(TEST_BUTTON_PIN_IN_ID, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(TEST_BUTTON_PIN_IN_ID, true);
	*/

}


//-----------------------------------------------------------------
//			CONFIG  -  4067
//-----------------------------------------------------------------

void config_nrf_gpio_out_4pins_4067(void)
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

void config_nrf_gpio_out_2pins_motor()
{
	nrf_gpio_cfg_output(GPIO_P16);
	nrf_gpio_cfg_output(GPIO_P17);
}

//-----------------------------------------------------------------
//			CONFIG  -  GPIO INPUT for DIP switch. This uses just 3 of 4 switch
//-----------------------------------------------------------------

void dipsw_scan_state()
{
	uint32_t pin_state_inv[DIPSWITCH_NUMBER];
	memset(pin_state_inv, 0, DIPSWITCH_NUMBER);

	pin_state_inv[DIP_0_0UART_1ADV] 	= nrf_gpio_pin_read(DIPSWITCH_0); // if return true --> switch ON, return false --> switch OFF
	pin_state_inv[DIP_1_0FAST_1SLOW] 	= nrf_gpio_pin_read(DIPSWITCH_1); // if return true --> switch ON, return false --> switch OFF
	pin_state_inv[DIP_2_NONE] 			= nrf_gpio_pin_read(DIPSWITCH_2); // if return true --> switch ON, return false --> switch OFF
	
	for(int switch_index = 0 ; switch_index < DIPSWITCH__USE_NUMBER ; switch_index++){		
		if(pin_state_inv[switch_index] == true) { // switch is OFF
			g_DIPSWITCH_state[switch_index]  = false;
		} 
		else  { // switch is ON
			g_DIPSWITCH_state[switch_index]  = true;
		}
		
#if defined(BOARD_PCA10028)
		g_DIPSWITCH_state[switch_index]  = false;
#endif
	}
}

void config_nrf_gpio_out_3pins_dipsw()
{
	//	config dip switch
	nrf_gpio_cfg_input(DIPSWITCH_0, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(DIPSWITCH_1, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(DIPSWITCH_2, NRF_GPIO_PIN_NOPULL);
}


//===============================================================================================================
//		Peripherals - TIMER
//===============================================================================================================

APP_TIMER_DEF(m_timer_id_peri_01);		/**< Venus timer 01. */
#define TIMER_INTERVAL_TIMER01		APP_TIMER_TICKS(550, APP_TIMER_PRESCALER) 	/**< Timer interval (ms, ticks). 5~3500 tested*/




/**@brief Function for handling the Venus application timer timeout.
 *
 * @details This function will be called each time the application timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */

//	TIMER_INTERVAL_TIMER01
static void peri_timeout_handler_01(void * p_context)
{
    UNUSED_PARAMETER(p_context);

	static uint32_t g_timer_counter_01 = 0;

	g_timer_counter_01++;
}



static void peri_timer_config()
{
    // Initialize timer module.
	//	This function should be called after "APP_TIMER_INIT"
	//	==> APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false); // ==> called in main()

	uint32_t err_code;

	err_code = app_timer_create(&m_timer_id_peri_01,
								APP_TIMER_MODE_REPEATED,
								peri_timeout_handler_01);
    APP_ERROR_CHECK(err_code);
}



//===============================================================================================================
//		Peripherals - MY functions
//===============================================================================================================

void config_HW(void)
{
	//	config core bsp
	config_nrf_gpio_out_3pins_dipsw();
	dipsw_scan_state();
		
	//	config peripheral and timer handler
	config_nrf_gpio_adc_3pins_sensor_bat();
	config_nrf_gpio_out_4pins_4067();
	config_nrf_gpio_out_2pins_motor();

	return;
}


void peri_main_start(void)
{
	
    uint32_t err_code;

    // Start Peri timers.
	//peri_timer_config();
	//err_code = app_timer_start(m_timer_id_peri_01, TIMER_INTERVAL_TIMER01, NULL);
	//APP_ERROR_CHECK(err_code);

}


