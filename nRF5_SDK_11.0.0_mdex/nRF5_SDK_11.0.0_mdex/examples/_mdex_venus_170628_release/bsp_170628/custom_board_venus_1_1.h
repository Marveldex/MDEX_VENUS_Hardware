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
 
/*	
	sehjin12 
	this header file is copied from PCA10028.h file.
	Pin map should be changed
*/
#ifndef CUSTOM_BOARD_MERCURY_H
#define CUSTOM_BOARD_MERCURY_H


/*
	<< Mercury board pin maps by Marveldex >>
	X : IDLE, none : not exist
	
	P00/AREF0	: X
	P01/AIN2 	: A2, Z of 4067 in Main board
	P02/AIN3 	: A3. Z of 4067 in Shield board
	P03/AIN4 	: X
	P04/AIN5 	: X
	P05/AIN6 	: X
	P06/AIN7 	: AIN7, BAT_DET
	P07 		: X
	P08 		: none
	P09~P12(#4)	: X - SPI, I2C
	P13 		: LED3
	P14 		: LED4
	P15 		: X
	P16~P20(#5)	: none
	P21 		: D1, S0 of 4067
	P22 		: D2, S1 of 4067
	P23 		: D3, S2 of 4067
	P24 		: D4, S3 of 4067
	P25 		: A0
	P26/AIN0 	: XL0
	P27/AIN1 	: XL1
	P28 		: LED1
	P29 		: LED2
*/

//	ADC channel number
#define	ADC_PIN_MAIN			2
#define	ADC_PIN_SUB			3
#define	ADC_PIN_BATTERY	7


// LEDs definitions for Mercury
#define LEDS_NUMBER    4

#define LED_START      28	//	21
#define LED_0          28	//	21
#define LED_1          29	//	22
#define LED_2          13	//	23
#define LED_3          14	//	24
#define LED_STOP       14	//	24

#define LEDS_LIST { LED_0, LED_1, LED_2, LED_3 }

#define BSP_LED_0      LED_0
#define BSP_LED_1      LED_1
#define BSP_LED_2      LED_2
#define BSP_LED_3      LED_3

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)
#define BSP_LED_3_MASK (1<<BSP_LED_3)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK)
#define LEDS_MASK_ALL	LEDS_MASK
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK


//		Mercury - 76HC4067 Decoder
#define	DECODER_4067_S0		21
#define	DECODER_4067_S1		22
#define	DECODER_4067_S2		23
#define	DECODER_4067_S3		24

#define	DECODER_4067_Z		1


//		Mercury - ADC for Sensor

//		Mercury - ADC for Battery

//		Venus - GPIO for Motor
#define	GPIO_P16		16
#define	GPIO_P17		17

//		Venus - DIPSWITCH
typedef enum DIPSWITCH_PIN_NAME_tag{ // switch up, down (up is true, down is false)
	DIP_0_0UART_1ADV = 0, // false : UART, true : ADV
	DIP_1_0FAST_1SLOW, // false : FAST, true : SLOW
	DIP_2_NONE,
	DIP_3_NONE
}DIPSWITCH_PIN_NAME;

typedef enum DIPSWITCH_PIN_INDEX_tag{ // switch up, down (up is true, down is false)
	DIP_0 = 0, // false : UART, true : ADV
	DIP_1, // false : FAST, true : SLOW
	DIP_2,
	DIP_3
}DIPSWITCH_PIN_INDEX;

#define	DIPSWITCH_NUMBER			4
#define	DIPSWITCH__USE_NUMBER	2
#define	DIPSWITCH_0		18
#define	DIPSWITCH_1		19
#define	DIPSWITCH_2		20
#define	DIPSWITCH_3		"none"


//		Mercury - Followings should be modified
/*
#define BUTTONS_NUMBER 4

#define BUTTON_START   17
#define BUTTON_1       17
#define BUTTON_2       18
#define BUTTON_3       19
#define BUTTON_4       20
#define BUTTON_STOP    20
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2
#define BSP_BUTTON_2   BUTTON_3
#define BSP_BUTTON_3   BUTTON_4

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)
#define BSP_BUTTON_2_MASK (1<<BSP_BUTTON_2)
#define BSP_BUTTON_3_MASK (1<<BSP_BUTTON_3)

#define BUTTONS_MASK   0x001E0000
*/

// Low frequency clock source to be used by the SoftDevice
#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_RC,            \
                                 .rc_ctiv       = 16,                                \
                                 .rc_temp_ctiv  = 2,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM }
#endif

#endif // CUSTOM_BOARD_MERCURY_H
