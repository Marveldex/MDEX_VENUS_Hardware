#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
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

#include "venus_peri.h"
#include "venus_ble.h"



/*
	<< Contests >>
	bsp message
	ble stack
	device manager
	gap
	service
	nus (gatt)
	conn param
	advertise
*/

//===============================================================================================================
//		VENUS APP - Variables, Declarations
//===============================================================================================================

//---------------------------------------------------------------
//		Device name
#if defined(BOARD_PCA10028) 
#define DEVICE_NAME				"Venus 1.2 pca01"	/**< Name of device. Will be included in the advertising data. */
#elif defined(BOARD_VENUS)
#define DEVICE_NAME				"Venus 1.2.2"		/**< Name of device. Will be included in the advertising data. */
#else
#error "Board is not defined"
#endif

#define DEVICE_NAME_ADV			"MDEX-ADV"

#define DEVICE_NAME_LEN_MAX		26
char g_device_name[DEVICE_NAME_LEN_MAX];

//---------------------------------------------------------------
//		BLE, uuid, service(nus) context, handles
static uint16_t						m_conn_handle = BLE_CONN_HANDLE_INVALID;	/**< Handle of the current connection. */
static dm_application_instance_t	m_app_handle;								/**< Application identifier allocated by device manager */
static dm_handle_t					m_bonded_peer_handle;						/**< Device reference handle to the current bonded central. */
static ble_uuid_t					m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

ble_nus_t							m_nus;										/**< Structure to identify the Nordic UART Service. */
unsigned char g_adv_data_packet[PACKET_ADV_LEN];


//---------------------------------------------------------------
//		Timer
APP_TIMER_DEF(m_timer_id_ble_pair_reset);		/**< Venus timer. */
#define TIMER_INTERVAL_BLE_PAIRING_RESET		APP_TIMER_TICKS(500, APP_TIMER_PRESCALER) 	/**< Timer interval (ms, ticks). 5~3500 tested*/

//---------------------------------------------------------------
//		Functions
static void ble_timeout_handler_PairDone(void * p_context);
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);
uint32_t nus_send_data_20(unsigned char * data, unsigned int length);





//===============================================================================================================
//		VENUS APP - Functions Definitions
//===============================================================================================================

//	TIMER_INTERVAL_TIMER01
static void ble_timeout_handler_PairDone(void * p_context)
{
    UNUSED_PARAMETER(p_context);

	//printf("[BLE:RESET] now reset... \n");

	int count = 7;
	while (count--) {
		app_led_toggle();
		nrf_delay_ms(200);
	}

	sd_nvic_SystemReset();
}


//===============================================================================================================
//		Basic Functions
//===============================================================================================================

bool m_is_ble_service_connected = false;
bool is_ble_service_connected() {
	return m_is_ble_service_connected;
}

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
void gap_params_init(void)
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

/**@brief Function for the GAP initialization.
 *
 * @details This function is totally same to gap_params_init but device name.
 */
void gap_params_adv_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME_ADV,
                                          strlen(DEVICE_NAME_ADV));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing services that will be used by the application.
 */
void services_init(void)
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
void conn_params_init(void)
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

        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void)
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

void advertising_permanently_init(void)
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

	/*
    ble_adv_modes_config_t options =
    {
        BLE_ADV_WHITELIST_ENABLED,
        BLE_ADV_DIRECTED_ENABLED,
        BLE_ADV_DIRECTED_SLOW_DISABLED, 0,0,
        BLE_ADV_FAST_ENABLED, APP_ADV_FAST_INTERVAL, APP_ADV_FAST_TIMEOUT,
        BLE_ADV_SLOW_ENABLED, APP_ADV_SLOW_INTERVAL, APP_ADV_SLOW_TIMEOUT
    };
	*/
	ble_adv_modes_config_t options = {0};
	options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	options.ble_adv_fast_timeout  = 0; // 0 : permanently

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
	
	if(BLE_EVT_TX_COMPLETE < p_ble_evt->header.evt_id) {
		printf("on_ble_evt header.evt_id = 0x%x \n", p_ble_evt->header.evt_id);
	}
	
#if 0
	BLE_GAP_EVT_CONNECTED  = BLE_GAP_EVT_BASE,    /**< 0x10	Connection established.                         \n See @ref ble_gap_evt_connected_t.            */
	BLE_GAP_EVT_DISCONNECTED,                     /**< 0x11	Disconnected from peer.                         \n See @ref ble_gap_evt_disconnected_t.         */
	BLE_GAP_EVT_CONN_PARAM_UPDATE,                /**< 0x12	Connection Parameters updated.                  \n See @ref ble_gap_evt_conn_param_update_t.    */
	BLE_GAP_EVT_SEC_PARAMS_REQUEST,               /**< 0x13	Request to provide security parameters.         \n Reply with @ref sd_ble_gap_sec_params_reply.  \n See @ref ble_gap_evt_sec_params_request_t. */
	BLE_GAP_EVT_SEC_INFO_REQUEST,                 /**< 0x14	Request to provide security information.        \n Reply with @ref sd_ble_gap_sec_info_reply.    \n See @ref ble_gap_evt_sec_info_request_t.   */
	BLE_GAP_EVT_PASSKEY_DISPLAY,                  /**< 0x15	Request to display a passkey to the user.       \n In LESC Numeric Comparison, reply with @ref sd_ble_gap_auth_key_reply. \n See @ref ble_gap_evt_passkey_display_t. */
	BLE_GAP_EVT_KEY_PRESSED,                      /**< 0x16	Notification of a keypress on the remote device.\n See @ref ble_gap_evt_key_pressed_t           */
	BLE_GAP_EVT_AUTH_KEY_REQUEST,                 /**< 0x17	Request to provide an authentication key.       \n Reply with @ref sd_ble_gap_auth_key_reply.    \n See @ref ble_gap_evt_auth_key_request_t.   */
	BLE_GAP_EVT_LESC_DHKEY_REQUEST,               /**< 0x18	Request to calculate an LE Secure Connections DHKey. \n Reply with @ref sd_ble_gap_lesc_dhkey_reply.  \n See @ref ble_gap_evt_lesc_dhkey_request_t */
	BLE_GAP_EVT_AUTH_STATUS,                      /**< 0x19	Authentication procedure completed with status. \n See @ref ble_gap_evt_auth_status_t.          */
	BLE_GAP_EVT_CONN_SEC_UPDATE,                  /**< 0x1a	Connection security updated.                    \n See @ref ble_gap_evt_conn_sec_update_t.      */
	BLE_GAP_EVT_TIMEOUT,                          /**< 0x1b	Timeout expired.                                \n See @ref ble_gap_evt_timeout_t.              */
	BLE_GAP_EVT_RSSI_CHANGED,                     /**< 0x1c	RSSI report.                                    \n See @ref ble_gap_evt_rssi_changed_t.         */
	BLE_GAP_EVT_ADV_REPORT,                       /**< 0x1d	Advertising report.                             \n See @ref ble_gap_evt_adv_report_t.           */
	BLE_GAP_EVT_SEC_REQUEST,                      /**< 0x1e	Security Request.                               \n See @ref ble_gap_evt_sec_request_t.          */
	BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST,        /**< 0x1f	Connection Parameter Update Request.            \n Reply with @ref sd_ble_gap_conn_param_update. \n See @ref ble_gap_evt_conn_param_update_request_t. */
	BLE_GAP_EVT_SCAN_REQ_REPORT,                  /**< 0x20	Scan request report.                            \n See @ref ble_gap_evt_scan_req_report_t.      */
#endif
	/*
	//	Bond sequence...
	10 > 13 > 12 > 1a > [19] > 12 > 12 > 12
	*/


	switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			
			//nrf_drv_gpiote_out_low_to_led_on(LED_2);
			m_is_ble_service_connected = true;
			printf("[O] - ble connected!\n");
		break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
		
			//nrf_drv_gpiote_out_high_to_led_off(LED_2);
			m_is_ble_service_connected = false;
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
		case BLE_GAP_EVT_AUTH_STATUS:
			//printf("[OK] - device bonded (paired)\n");

			// Start application timers.
			err_code = app_timer_start(m_timer_id_ble_pair_reset, TIMER_INTERVAL_BLE_PAIRING_RESET, NULL);
			APP_ERROR_CHECK(err_code);
		break;

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
void ble_stack_init(void)
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

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
void buttons_leds_init(bool * p_erase_bonds)
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
	printf("device_manager_evt_handler ev id = 0x%x, parlen=%d, ev result = %d \n", p_event->event_id, p_event->event_paramlen, event_result);
	//printf("device_manager handle, app=%d, conn=%d, dev=%d, svc=%d \n", p_handle->appl_id, p_handle->connection_id, p_handle->device_id, p_handle->service_id);

	/*
	//	p_event->event_id sequence...
	'Bond (Pairing)' with nrfConnect or Android setting
		11 > 13 > 15 > 14
	'Delete bond (Forget)' with nrfConnect or Android setting
		12

	'Connect' with nrfConnect
		11
	'Disconnect' with nrfConnect
		12
	*/
	
#if 0
	#define DM_EVT_CONNECTION              0x11 /**< Indicates that link with the peer is established. */
	#define DM_EVT_DISCONNECTION           0x12 /**< Indicates that link with peer is torn down. */
	#define DM_EVT_SECURITY_SETUP          0x13 /**< Security procedure for link started indication */
	#define DM_EVT_SECURITY_SETUP_COMPLETE 0x14 /**< Security procedure for link completion indication. */
	#define DM_EVT_LINK_SECURED            0x15 /**< Indicates that link with the peer is secured. For bonded devices, subsequent reconnections with bonded peer will result only in this event when the link is secured and setup procedures will not occur unless the bonding information is either lost or deleted on either or both sides.  */
	#define DM_EVT_SECURITY_SETUP_REFRESH  0x16 /**< Indicates that the security on the link was re-established. */

	#define DM_EVT_DEVICE_CONTEXT_LOADED   0x21 /**< Indicates that device context for a peer is loaded. */
	#define DM_EVT_DEVICE_CONTEXT_STORED   0x22 /**< Indicates that device context is stored persistently. */
	#define DM_EVT_DEVICE_CONTEXT_DELETED  0x23 /**< Indicates that device context is deleted. */
	#define DM_EVT_SERVICE_CONTEXT_LOADED  0x31 /**< Indicates that service context for a peer is loaded. */
	#define DM_EVT_SERVICE_CONTEXT_STORED  0x32 /**< Indicates that service context is stored persistently. */
	#define DM_EVT_SERVICE_CONTEXT_DELETED 0x33 /**< Indicates that service context is deleted. */
#endif
    switch(p_event->event_id)
    {
		case DM_EVT_CONNECTION:
			BLE_LOG("[BLE:CONN]dev id = %d, service id = %d \n", p_handle->device_id, p_handle->service_id);
			break;
		case DM_EVT_DISCONNECTION:
			BLE_LOG("[BLE:DISCONN]dev id = %d, service id = %d \n", p_handle->device_id, p_handle->service_id);
			break;
        case DM_EVT_LINK_SECURED:
            m_bonded_peer_handle = (*p_handle);
			if(m_bonded_peer_handle.device_id != NULL){
				BLE_LOG("[BLE:LINK_SEC] %d\n", m_bonded_peer_handle.service_id);
			}
            break;
		case DM_EVT_SECURITY_SETUP_COMPLETE:
		case DM_EVT_SERVICE_CONTEXT_STORED:
			BLE_LOG("[BLE:BOND OK]dev id = %d, service id = %d \n", p_handle->device_id, p_handle->service_id);
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
void device_manager_init(bool erase_bonds)
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
	
	//printf("delete bond list \n");
	//dm_device_delete_all(&m_app_handle);
}

/*-----------------------------------------------------------------------------------------
	<<---	Basic Functions
	
	User custom functions -->
-----------------------------------------------------------------------------------------*/
//===============================================================================================================
//		User custom functions
//===============================================================================================================

/**@brief 
 *  @author sehjin12
 */

ble_advdata_manuf_data_t manuf_specific_data;

void advertisement_update_packet(unsigned char * packet_data, unsigned int packet_len)
{
	uint32_t      err_code;
	ble_advdata_t advdata;
	ble_advdata_t scanrsp;

	// Build advertising data struct to pass into @ref ble_advertising_init.
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	//-advdata.include_appearance      = true;
	advdata.include_appearance      = false;
	advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE ;
	
	//advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	//advdata.uuids_complete.p_uuids  = m_adv_uuids;
	
	memset(&scanrsp, 0, sizeof(scanrsp));
	scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	scanrsp.uuids_complete.p_uuids  = m_adv_uuids;


	//New part

	ble_advdata_manuf_data_t manuf_specific_data;
	memset(&manuf_specific_data, 0, sizeof(manuf_specific_data));
	
	manuf_specific_data.company_identifier = 0xFFFF; // 0xFFFF = none company id 
	manuf_specific_data.data.p_data = packet_data;
	manuf_specific_data.data.size  = packet_len;
	
	advdata.p_manuf_specific_data = &manuf_specific_data;


	ble_adv_modes_config_t options = {0};
	options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	//options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
	options.ble_adv_fast_timeout  = 0; // 0 means forever

	err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
	//err_code = ble_advdata_set(&advdata, NULL);
	APP_ERROR_CHECK(err_code);
}



uint32_t config_gap_param_and_init_advertise(void)
{
	uint32_t err_code;	

	//APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

	//ble_stack_init();

	memset(g_device_name, 0, DEVICE_NAME_LEN_MAX);
	strncpy(g_device_name, DEVICE_NAME_ADV, sizeof(DEVICE_NAME_ADV));
	gap_params_adv_init();

	//services_init();
	//advertising_init();
	//advertisement_update_switch_forever(false);
	//conn_params_init();	

	memset(g_adv_data_packet, 0x00, PACKET_ADV_LEN);
	advertisement_update_packet(g_adv_data_packet, PACKET_ADV_LEN);


	err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
	return (err_code);
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
	on_nus_receive_parse(p_data, length);
/*
	printf("receive via ble (size=%d)\n", length);
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
*/
}
/**@snippet [Handling the data received over BLE] */

uint32_t nus_send_data_20(unsigned char * data, unsigned int length)
{
	uint32_t       err_code;

	err_code = ble_nus_string_send(&m_nus, data, length);
	if (err_code != NRF_ERROR_INVALID_STATE){
		APP_ERROR_CHECK(err_code);
	}
	
	return err_code;
}

uint32_t ble_adv_start(BLE_ADV_MODE adv_mode)
{
	uint32_t err_code;

	// Create timers.
	err_code = app_timer_create(&m_timer_id_ble_pair_reset,
								APP_TIMER_MODE_SINGLE_SHOT,
								ble_timeout_handler_PairDone);
	APP_ERROR_CHECK(err_code);

	m_is_ble_service_connected = false; // sehjin12

	switch(adv_mode) {
		case ADV_REQ_CONNECT:
			err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
			break;
		
		case ADV_ONLY:
			err_code = config_gap_param_and_init_advertise();
			break;
	}

	APP_ERROR_CHECK(err_code);

	return err_code;
}



