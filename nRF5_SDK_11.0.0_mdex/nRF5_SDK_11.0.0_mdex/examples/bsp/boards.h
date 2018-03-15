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
#ifndef BOARDS_H
#define BOARDS_H

#include "nrf_gpio.h"
#include "nrf_log.h"

#if defined(BOARD_PCA10028) // includes cases of real PCA10028 and also Venus simulation with PCA10028
//  #include "pca10028.h"
   #include "pca10028_simul_venus01.h"
//  #include "pca10028_simul_venus02_pins.h"
//  #include "pca10028_simul_venus03_reduced.h"
//  #include "pca10028_simul_venus04_4067.h"
#elif defined(BOARD_CUSTOM)
  #include "custom_board.h"
#elif defined(BOARD_VENUS)
  #include "venus_v2_a0_basic.h"
#else
#error "Board is not defined"
#endif


#define APP_ENABLE_LOGS
//#undef APP_ENABLE_LOGS

#ifdef APP_ENABLE_LOGS
#define APP_LOG  printf  /**< Used for logging details. */
//#define APP_DUMP app_trace_dump /**< Used for dumping octet information to get details of bond information etc. */
#else //APP_ENABLE_LOGS
//#define APP_DUMP(...)           /**< Disables dumping of octet streams. */
#define APP_LOG(...)            /**< Disables detailed logs. */
#endif //DM_DISABLE_LOGS, APP_ENABLE_LOGS


//	sehjin12 - LEDS_INV_MASK. venus has ~ in definition, while PCA10028 doesn't

#define LEDS_OFF(leds_mask) do {  NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
							NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LEDS_ON(leds_mask) do {  NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
						   NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LED_IS_ON(leds_mask) ((leds_mask) & (NRF_GPIO->OUT ^ LEDS_INV_MASK) )

#define LEDS_INVERT(leds_mask) do { uint32_t gpio_state = NRF_GPIO->OUT;      \
							  NRF_GPIO->OUTSET = ((leds_mask) & ~gpio_state); \
							  NRF_GPIO->OUTCLR = ((leds_mask) & gpio_state); } while (0)

#define LEDS_CONFIGURE(leds_mask) do { uint32_t pin;                  \
								  for (pin = 0; pin < 32; pin++) \
									  if ( (leds_mask) & (1 << pin) )   \
										  nrf_gpio_cfg_output(pin); } while (0)



//#endif		//	USE_PCA10028_SIMUL_PINMAP


#endif		//	BOARDS_H
