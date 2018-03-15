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
#ifndef VENUIS_V2_A0_BASIC_H_
#define VENUIS_V2_A0_BASIC_H_

/********************************************************
	<<	VERSION	>>
	01 : First one. same to pca10028
	02 : Fits led, button. Deleted SPIS, SPIM0, SPIM1, Serialization to App/Conn B'd, Arduino
	03 : Srouce code decorated.
	
	<<	TODO >>
********************************************************/




#ifdef USE_PCA10028_SIMUL_PINMAP

#define	PIN_ID_DUMMY_OUT	26	//	idle pin of pca10028, originnally 26 is for LFCLK
#define	PIN_ID_DUMMY_IN		27	//	idle pin of pca10028, originnally 27 is for LFCLK


// LEDs definitions for PCA10028
#define LEDS_NUMBER    1

#define LED_START		21
#define LED_1			21
#define LED_2			PIN_ID_DUMMY_OUT	// 22: valid, 21: dummy (same to LED_1)
//#define LED_2 		22
#define LED_STOP  		21

#define LEDS_LIST { LED_1, LED_2 }

#define BSP_LED_0		LED_1
#define BSP_LED_1		LED_2

#define BSP_LED_0_MASK	(1<<BSP_LED_0)
#define BSP_LED_1_MASK	(1<<BSP_LED_1)

#define LEDS_MASK		(BSP_LED_0_MASK | BSP_LED_1_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK


//	BUTTONS
#define BUTTONS_NUMBER 	1	//	button number is at least 4. (fixed)

#define BUTTON_START	17
#define BUTTON_1		17
#define BUTTON_STOP   	17
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BUTTONS_MASK   0x001E0000



//	Original
/*
#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           true
*/
#define RX_PIN_NUMBER	11
#define TX_PIN_NUMBER	9
#define CTS_PIN_NUMBER	PIN_ID_DUMMY_OUT
#define RTS_PIN_NUMBER	PIN_ID_DUMMY_OUT
#define HWFC           	false



//-----------------------------------------------------------------------------
//		VENUS PINS ASSIGNMENT
//-----------------------------------------------------------------------------
#else	// VENUIS_V2_A0_BASIC_H_


/*
	<< Venus board pin maps by Marveldex >>
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
#define	PIN_ID_DUMMY_OUT	13	//	idle pin of pca10028, originnally 26 is for LFCLK
#define	PIN_ID_DUMMY_IN		14	//	idle pin of pca10028, originnally 27 is for LFCLK
#define	PIN_ID_DUMMY_BUTTON	15	//	Venus setting, Not used

#define LEDS_NUMBER    1

#define LED_START		28	//	21
//#define LED_0  		28	//	21
#define LED_1			28	//	22
#define LED_2			PIN_ID_DUMMY_OUT	//	22
#define LED_STOP		28	//	24

#define LEDS_LIST { LED_1, LED_2 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK)
#define LEDS_MASK_ALL	LEDS_MASK
//	all LEDs are lit when GPIO is low
#define LEDS_INV_MASK	(~LEDS_MASK)	//	for Venus : includes inverse (~)
//#define LEDS_INV_MASK	(LEDS_MASK)

//		Venus - LED 1 for application
#define TEST_GPIO_PIN_ID	29 // = LED_4
#define	TEST_GPIO_PIN_ID_MASK		(1 << TEST_GPIO_PIN_ID)
#define	TEST_GPIO_PIN_ID_MASK_INV	(~TEST_GPIO_PIN_ID_MASK)
#define TEST_GPIO_IS_ON(gpio_pin_mask) ((gpio_pin_mask) & (NRF_GPIO->OUT & TEST_GPIO_PIN_ID_MASK) )


//		Venus - BUTTONS
#define BUTTONS_NUMBER 	1	//	button number is at least 4. (fixed)

#define BUTTON_START	PIN_ID_DUMMY_BUTTON	// PIN_ID_DUMMY_BUTTON : 5
#define BUTTON_1		PIN_ID_DUMMY_BUTTON
#define BUTTON_STOP   	PIN_ID_DUMMY_BUTTON
#define BUTTON_PULL    	NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BUTTONS_MASK   0x001E0000


//	SIMULATE
#define RX_PIN_NUMBER	PIN_ID_DUMMY_IN
#define TX_PIN_NUMBER	PIN_ID_DUMMY_OUT
#define CTS_PIN_NUMBER	PIN_ID_DUMMY_OUT
#define RTS_PIN_NUMBER	PIN_ID_DUMMY_OUT
#define HWFC           	false



//		Venus - ADC for Battery
#define	ADC_PIN_BATTERY		7

//		Venus - ADC for Sensor
#define	ADC_PIN_MAIN		2
#define	ADC_PIN_SUB			3

//		Venus - 76HC4067 Decoder
#define	DECODER_4067_S0		21
#define	DECODER_4067_S1		22
#define	DECODER_4067_S2		23
#define	DECODER_4067_S3		24

//#define	DECODER_4067_Z		1


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

#define	DIPSWITCH_NUMBER		4
#define	DIPSWITCH__USE_NUMBER	3
#define	DIPSWITCH_0		18
#define	DIPSWITCH_1		19
#define	DIPSWITCH_2		20
#define	DIPSWITCH_3		"none"



#endif // USE_PCA10028_SIMUL_PINMAP



//-----------------------------------------------------------------------------

// Low frequency clock source to be used by the SoftDevice
//	case of internal oscillator
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_RC,	\
								 .rc_ctiv       = 16,					\
								 .rc_temp_ctiv  = 4,					\
								 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM}


#endif // VENUIS_V2_A0_BASIC_H_
