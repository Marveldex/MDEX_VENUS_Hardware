
 /*
	<<  Program Softdevice (S130) of Nordic 51822AA for this project  >>
 C:/Keil_v5/ARM/nRF5_SDK_11.0.0/components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex
 
< S130  binary >
	ROM Size : 
	RAM Size : 

< Application binary >
Assigned : 8,064 Byte of RAM, 151,552 Bytes of ROM
	Program Size: Code=10936 RO-data=344 RW-data=208 ZI-data=3568  
	ROM Size = Code + RO Data + RW Data = 10936 + 344 + 208 = 11488 = 11.22KB
	RAM Size = RW Data + ZI Data = 208 + 3568 = 3776 = 3.6KB

AA is 256K flash/16K sram, 
AB is 128K flash/16K sram, 
AC is 256K flash/32K sram 
*/


//-------------------------------------------------------
//			USER DEFINITION  -  VARIABLES
//-------------------------------------------------------

#define	NUS_PACKET_MAX_SIZE	20
uint8_t		g_ucNUS_PacketBuffer[NUS_PACKET_MAX_SIZE];
unsigned int g_seat_adc_sum = 0;
unsigned int g_seat_adc_valid_count = 0;
#define PACKET_ADV_LEN		5
unsigned char g_adv_data_packet[PACKET_ADV_LEN];

#define	THRESHOLD_VALID_CELL_CUTOFF	5


#define	NUM_OF_HC4067_Y		16
#define	NUM_OF_SENSOR			NUM_OF_HC4067_Y
uint32_t	g_uiADC_value_seat[NUM_OF_HC4067_Y];


#define	ADC_TRIM_VALUE_7bit	127

volatile unsigned int g_uiADC_IRQ_HW_read;


#define	BATTERY_LEVEL_OFF		1
uint32_t 	g_uiBATTERY_level_100 = 0;

typedef enum enIC4067BLOCK_ID{
	ID_BLOCK_MAIN = 0,
	ID_BLOCK_SUB = 1
}IC4067BLOCK_ID;

const uint8_t g_LED_HW_pin_map[LEDS_NUMBER] = LEDS_LIST;

//	HW block - DIP switch
typedef struct DIP_SWITCH_HW_t{
	bool is_state_0;
	bool is_state_1;
	bool is_state_2;
	bool is_state_3; // not used
}DIP_SWITCH_HW;
//DIP_SWITCH_HW	g_switch_hw;

bool g_DIPSWITCH_state[DIPSWITCH_NUMBER];


//-------------------------------------------------------
//			USER DEFINITION  -  FUNCTIONS
//-------------------------------------------------------
uint32_t g_main_loop_count = 0;


void timer_mainloop_handler(nrf_timer_event_t event_type, void* p_context);
void timer1_loop_main(void);

void blink_led_save(int id);

void measure_adc_seat_16ch(IC4067BLOCK_ID ic4067_id);
void measure_adc_battery_half(void);
void reorder_adc_scanned_buffer_VENUS(void);

void venus_packet_clear(void);
void venus_packet_assemble(IC4067BLOCK_ID block_id);
void venus_packet_send(unsigned char * packet_data);

void scan_and_send(void);

//	Android style function declaration
void config_HW(void);
void loop(void);


