#ifndef _VENUS_BLE_H_
#define	_VENUS_BLE_H_

//===============================================================================================================
//		VENUS APP - Variables, Declarations
//===============================================================================================================

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/



#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS 		180                                    		/**< The advertising timeout (in units of seconds). */
//#define APP_ADV_TIMEOUT_IN_SECONDS   	0                                         	/**< The advertising timeout (in units of seconds). */

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

typedef enum Ble_Adv_Mode_t {
	ADV_REQ_CONNECT = 0,
	ADV_ONLY
}BLE_ADV_MODE;

#define PACKET_ADV_LEN		5
extern unsigned char g_adv_data_packet[PACKET_ADV_LEN];

//===============================================================================================================
//		VENUS APP - Functions Definitions
//===============================================================================================================
void sleep_mode_enter(void);

void gap_params_init(void);
void services_init(void);
void ble_stack_init(void);
void conn_params_init(void);
void device_manager_init(bool erase_bonds);
void buttons_leds_init(bool * p_erase_bonds);

void advertising_init(void);
void advertising_permanently_init(void);
void advertisement_update_packet(unsigned char * packet_data, unsigned int packet_len);

uint32_t ble_adv_start(BLE_ADV_MODE adv_mode);


extern void on_nus_receive_parse(unsigned char * received_data, unsigned int length);
bool is_ble_service_connected(void);
uint32_t nus_send_data_20(unsigned char * data, unsigned int length);

/*-----------------------------------------------------------------------------------------
	<<---	Basic Functions
	
	User custom functions -->
-----------------------------------------------------------------------------------------*/
bool BLE_get_connection_state(void);


#ifndef DM_DISABLE_LOGS
#define BLE_LOG  app_trace_log  /**< Used for logging details. */
#define BLE_DUMP app_trace_dump /**< Used for dumping octet information to get details of bond information etc. */
#else //DM_DISABLE_LOGS
#define BLE_DUMP(...)           /**< Disables dumping of octet streams. */
#define BLE_LOG(...)            /**< Disables detailed logs. */
#endif //DM_DISABLE_LOGS, APP_DISABLE_LOGS

#endif // _VENUS_BLE_H_
