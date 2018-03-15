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
	[TODO]
	- Pin assign adaptive to Venus
	- DFU (Firmware upgrade)
	- Divide and rull : ble, peri, adc

	[Almost done, but LATER]
	- LED indication with BLE stack. Bonding, Connecting, Broadcasting...
	- Answer timer tick via nus request.
	- Save working : Idle (Check per 10 sec) - Wait (check per 2 sec) - Work (check per 1 sec).
	
	[DONE]
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
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"

#include "ble_nus.h"

#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"

#include "app_button.h"
#include "app_uart.h"
#include "app_util_platform.h"

#include "bsp.h"
#include "bsp_btn_ble.h"



#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Mdex_UART_bond"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS 		180                                    		/**< The advertising timeout (in units of seconds). */
//#define APP_ADV_TIMEOUT_IN_SECONDS   	0                                         	/**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static dm_application_instance_t        m_app_handle;                              /**< Application identifier allocated by device manager */
static dm_handle_t                       m_bonded_peer_handle;                          /**< Device reference handle to the current bonded central. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
		printf("receive via ble (size=%d)\n", length);
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
	
	printf("go to sleep...\n");

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

	printf("on_adv_evt : 0x%x \n", ble_adv_evt);
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
/*
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
*/
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
	
	printf("on_ble_evt header.evt_id = %d \n", p_ble_evt->header.evt_id);
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			
			nrf_drv_gpiote_out_low_to_led_on(LED_2);
			printf("[O] - ble connected!\n");
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
		
			nrf_drv_gpiote_out_high_to_led_off(LED_2);
			printf("[X] - ble disconnected!\n");
            break;

        case BLE_GATTS_EVT_TIMEOUT: // for bonding from hts example

            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

			/*
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
			*/

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    
	ble_conn_params_on_ble_evt(p_ble_evt);
    dm_ble_evt_handler(p_ble_evt);

    bsp_btn_ble_on_ble_evt(p_ble_evt);
	on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    //uint32_t err_code;
	//bool     is_indication_enabled;
	printf("device_manager_evt_handler ev id = 0x%x \n", p_event->event_id);

    switch(p_event->event_id)
    {
		case DM_EVT_CONNECTION:
			printf("[DM:CONN]dev id = %d, service id = %d \n", p_handle->device_id, p_handle->service_id);
			break;
		case DM_EVT_DISCONNECTION:
			printf("[DM:DISCONN]dev id = %d, service id = %d \n", p_handle->device_id, p_handle->service_id);
			break;
//        case DM_EVT_DEVICE_CONTEXT_LOADED: // Fall through.
        case DM_EVT_LINK_SECURED:
            m_bonded_peer_handle = (*p_handle);
			/*
            // Send a single temperature measurement if indication is enabled.
            // NOTE: For this to work, make sure ble_hts_on_ble_evt() is called before
            //       dm_ble_evt_handler() in ble_evt_dispatch().
            err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
            APP_ERROR_CHECK(err_code);

            if (is_indication_enabled)
            {
                temperature_measurement_send();
            }
		*/
            break;

        default:
            break;
    }

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;
	
	static uint8_t	tx_buffer_len = 0;
	static bool	tx_buffer_out = false;

	
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
		
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
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
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the Advertising permanently functionality.
 */
#define APP_ADV_FAST_INTERVAL            0x0028                                         /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL            0x0C80                                         /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_ADV_FAST_TIMEOUT             30                                             /**< The duration of the fast advertising period (in seconds). */
#define APP_ADV_SLOW_TIMEOUT             0                                            /**< The duration of the slow advertising period (in seconds). */

static void advertising_permanently_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    //advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; // permanently

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

//    ble_adv_modes_config_t options = {0};
    ble_adv_modes_config_t options =
    {
        BLE_ADV_WHITELIST_ENABLED,
        BLE_ADV_DIRECTED_ENABLED,
        BLE_ADV_DIRECTED_SLOW_DISABLED, 0,0,
        BLE_ADV_FAST_ENABLED, APP_ADV_FAST_INTERVAL, APP_ADV_FAST_TIMEOUT,
        BLE_ADV_SLOW_ENABLED, APP_ADV_SLOW_INTERVAL, APP_ADV_SLOW_TIMEOUT
    };
	//options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
	//options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	//options.ble_adv_fast_timeout  = 0; // 0 : permanently

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


#define	TEST_BUTTON_PIN_IN_ID	20 // = BUTTON_4
#define TEST_LED_PIN_ID			24 // = LED_4

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	nrf_drv_gpiote_out_toggle(TEST_LED_PIN_ID);

	/*
	if( action == GPIOTE_CONFIG_POLARITY_HiToLo )
		nrf_drv_gpiote_out_toggle(TEST_LED_PIN_ID);

	if( action == GPIOTE_CONFIG_POLARITY_LoToHi) {
		//nrf_drv_gpiote_out_set(TEST_LED_PIN_ID);
	}
	else if ( action == GPIOTE_CONFIG_POLARITY_HiToLo) {
		//nrf_drv_gpiote_out_clear(TEST_LED_PIN_ID);
	}
	*/

}
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output, 
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
	//-		GPIO OUT
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);

    err_code = nrf_drv_gpiote_out_init(TEST_LED_PIN_ID, &out_config);
    APP_ERROR_CHECK(err_code);

	nrf_drv_gpiote_out_clear(TEST_LED_PIN_ID);
	//nrf_drv_gpiote_out_high_to_led_off(TEST_LED_PIN_ID);


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


//===============================================================================================================
//	-->		VENUS APP Begins here
//===============================================================================================================
APP_TIMER_DEF(m_timer01_venus_id);		/**< Venus timer 01. */
APP_TIMER_DEF(m_timer02_venus_id);		/**< Venus timer 02. */
#define TIMER_INTERVAL_TIMER01		APP_TIMER_TICKS(3500, APP_TIMER_PRESCALER) 	/**< Timer interval (ms, ticks). 5~3500 tested*/
#define TIMER_INTERVAL_TIMER02		APP_TIMER_TICKS(1, APP_TIMER_PRESCALER) 	/**< Timer interval (ms, ticks). 1~3500 tested*/


/**@brief Function for handling the Venus application timer timeout.
 *
 * @details This function will be called each time the application timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */

static void timer01_venus_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

	nrf_drv_gpiote_out_toggle(TEST_LED_PIN_ID);

	//printf("timer 01 (3500ms) done\n");
}

static void timer02_venus_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	
	static uint32_t g_timer02_counter = 0;
	static uint32_t g_get_min_count = 0;

	g_timer02_counter++;
	if( g_timer02_counter % (1000 * 60) == 0) { // 1000 ms x 60 sec = 1 min
		g_get_min_count++;;
		printf("[timer] %d min past\n", g_get_min_count);
	}

}

static void venus_timer_config()
{
    // Initialize timer module.
	//	This function should be called after "APP_TIMER_INIT"
	//	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false); // ==> called in main()

	uint32_t err_code;

	// Create timers.
    err_code = app_timer_create(&m_timer01_venus_id,
                                APP_TIMER_MODE_REPEATED,
                                timer01_venus_timeout_handler);
    APP_ERROR_CHECK(err_code);

	// Create timers.
    err_code = app_timer_create(&m_timer02_venus_id,
                                APP_TIMER_MODE_REPEATED,
                                timer02_venus_timeout_handler);
    APP_ERROR_CHECK(err_code);

}


static void venus_app_init(void)
{
	venus_timer_config();
	
    uint32_t err_code;

    // Start application timers.
	err_code = app_timer_start(m_timer01_venus_id, TIMER_INTERVAL_TIMER01, NULL);
	APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_timer02_venus_id, TIMER_INTERVAL_TIMER02, NULL);
    APP_ERROR_CHECK(err_code);

}

//===============================================================================================================
//	<--		VENUS APP Ends here
//===============================================================================================================


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;

    // Initialize drivers.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    uart_init();
    gpio_init();
	
	//	Initialize bsp
    buttons_leds_init(&erase_bonds); // order should be after gpio_init
    
	//	Initialize ble
	ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    services_init();
    //advertising_init();
	advertising_permanently_init();
    conn_params_init();
	
	//	Initialize venus application
	{
		venus_app_init();
	}
	

    printf("\r\nUART Start!\r\n");


    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}


/** 
 * @}
 */
