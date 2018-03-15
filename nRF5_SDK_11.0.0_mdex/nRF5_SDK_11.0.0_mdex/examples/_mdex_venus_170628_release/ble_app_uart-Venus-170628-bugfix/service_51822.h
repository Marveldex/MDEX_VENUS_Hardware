
///////////////////////////////////////////////////////////////////////////////////
//
//			--> 	NORDIC MACRO
//
///////////////////////////////////////////////////////////////////////////////////

//	Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device.
#define IS_SRVC_CHANGED_CHARACT_PRESENT		0

//	Number of central links used by the application. When changing this number remember to adjust the RAM settings
#define CENTRAL_LINK_COUNT		0
//	Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings
#define PERIPHERAL_LINK_COUNT	1


//	UUID type for the Nordic UART Service (vendor specific).
#define NUS_SERVICE_UUID_TYPE	BLE_UUID_TYPE_VENDOR_BEGIN


//	The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms).
#define APP_ADV_INTERVAL                64
//	The advertising timeout (in units of seconds).
#define APP_ADV_TIMEOUT_IN_SECONDS      180


//	Value of the RTC1 PRESCALER register.
#define APP_TIMER_PRESCALER             0
//	Size of timer operation queues.
#define APP_TIMER_OP_QUEUE_SIZE         4


//	Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units.
#define MIN_CONN_INTERVAL		MSEC_TO_UNITS(20, UNIT_1_25_MS)
//	Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units.
#define MAX_CONN_INTERVAL		MSEC_TO_UNITS(75, UNIT_1_25_MS)
//	Slave latency.
#define SLAVE_LATENCY		0	
//	Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
#define CONN_SUP_TIMEOUT		MSEC_TO_UNITS(4000, UNIT_10_MS)


//	Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
#define 	FIRST_CONN_PARAMS_UPDATE_DELAY  	APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
//	Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds)
#define 	NEXT_CONN_PARAMS_UPDATE_DELAY   	APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
//	Number of attempts before giving up the connection parameter negotiation.
#define 	MAX_CONN_PARAMS_UPDATE_COUNT    	3

//	Value used as error code on stack dump, can be used to identify stack location on stack unwind.
#define DEAD_BEEF		0xDEADBEEF


//	UART TX, RX buffer size
#define UART_TX_BUF_SIZE		256
#define UART_RX_BUF_SIZE		256


#ifndef NRF_APP_PRIORITY_HIGH
#define NRF_APP_PRIORITY_HIGH 1
#endif



void config_bsp_nus(void);
void config_gap_param_and_init_nus(void);
void config_gap_param_and_init_advertise(void);

void advertisement_update_switch_forever(bool is_switch_on);
void advertisement_update_packet(unsigned char * packet_data, unsigned int packet_len);


unsigned int nus_send_data_20(unsigned char * data, unsigned int length);

extern void on_BLE_receive(unsigned char * received_data, unsigned int length);

extern bool BLE_get_connection_state(void);


