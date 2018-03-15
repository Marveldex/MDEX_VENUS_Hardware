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
#ifndef PCA10028_SIMUL_VENUS_03_H
#define PCA10028_SIMUL_VENUS_03_H

/********************************************************
	<<	VERSION	>>
	01 : First one. same to pca10028
	02 : Fits led, button. Deleted SPIS, SPIM0, SPIM1, Serialization to App/Conn B'd, Arduino
	03 : Srouce code decorated.
	
	<<	TODO >>
********************************************************/

#define	PIN_ID_DUMMY_OUT	26	//	idle pin of pca10028, originnally 26 is for LFCLK
#define	PIN_ID_DUMMY_IN		27	//	idle pin of pca10028, originnally 27 is for LFCLK


// LEDs definitions for PCA10028
#define LEDS_NUMBER    1

#define LED_START      21
#define LED_1          21
#define LED_2          PIN_ID_DUMMY_OUT	// 22: valid, 21: dummy (same to LED_1)
//#define LED_2          22
#define LED_STOP       21

#define LEDS_LIST { LED_1, LED_2 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK


//	BUTTONS
#define BUTTONS_NUMBER 	1	//	button number is at least 4. (fixed)

#define BUTTON_START   	17
#define BUTTON_1       	17
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


// Low frequency clock source to be used by the SoftDevice
//	case of internal oscillator
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_RC,	\
								 .rc_ctiv       = 16,					\
								 .rc_temp_ctiv  = 4,					\
								 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM}


#endif // PCA10028_SIMUL_VENUS_03_H
