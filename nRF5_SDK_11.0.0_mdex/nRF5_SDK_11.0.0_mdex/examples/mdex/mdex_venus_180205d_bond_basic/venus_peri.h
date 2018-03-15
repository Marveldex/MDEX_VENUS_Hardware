#ifndef _VENUS_PERI_H_
#define	_VENUS_PERI_H_

//===============================================================================================================
//		VENUS PERI - Variables, Declarations
//===============================================================================================================

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


#ifndef NRF_APP_PRIORITY_HIGH
#define NRF_APP_PRIORITY_HIGH 1
#endif


//	4067
#define	NUM_CELLS_in4067		16

typedef enum enIC4067BLOCK_ID{
	ID_BLOCK_MAIN = 0,
	ID_BLOCK_SUB,
	ID_BLOCK_NUM // 2
}IC4067BLOCK_ID;

extern bool g_DIPSWITCH_state[DIPSWITCH_NUMBER];



extern ble_nus_t	m_nus;	/**< Structure to identify the Nordic UART Service. */

//===============================================================================================================
//		VENUS PERI - FUNCTIONS
//===============================================================================================================

void uart_init(void);
void gpio_init(void);
void peri_main_start(void);

void config_HW(void);
void config_4067_single_ch(unsigned int ch);
void dipsw_scan_state(void);
bool get_dipsw_state(int dip_sw_index);

uint32_t ADC_IRQ_ConfigAndGetValue_10bit_Delay81us(void);


void app_led_turn_on(bool is_turn_on);
void app_led_toggle(void);

#endif // _VENUS_PERI_H_
