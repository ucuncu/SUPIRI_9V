/******************************************************************************/
/** @file       led.c
 *******************************************************************************
 *
 *  @brief      Module for handling the attached user led
 *              <p>
 *              There are two user led's attached to the stm32l053:
 *              <ul>
 *                  <li> LED_GREEN: I/O PB4
 *                  <li> LED_RED: I/O PA5
 *              </ul>
 *
 *  @author     wht4
 *
 *  @remark     Last modifications
 *                 \li V1.0, February 2016, wht4, initial release
 *
 ******************************************************************************/
/*
 *  functions  global:
 *              led_init
 *              led_set
 *              led_clear
 *              led_toogle
 *  functions  local:
 *              .
 *
 ******************************************************************************/

/****** Header-Files **********************************************************/
#include "led.h"
#include "gpio.h"

/****** Macros ****************************************************************/

/****** Function prototypes ****************************************************/

/****** Data ******************************************************************/

/****** Implementation ********************************************************/

/*******************************************************************************
 *  function :    led_init
 ******************************************************************************/
void
led_init(void) {

    GpioInit_t tGpioInit = {GPIO_MODE_OUTPUT,
                            GPIO_OUTPUT_PUSH_PULL,
                            GPIO_SPEED_MEDIUM,
                            GPIO_PULL_NON
                           };

    /* Enable the peripheral clock of GPIOC */
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;

    /* Select output mode (01) on GPIOC pin 13 */
    gpio_init(GPIOC, 13, &tGpioInit);

}


/*******************************************************************************
 *  function :    led_set
 ******************************************************************************/
void led_set(void) 
{
   gpio_set(GPIOC, 13);
}


/*******************************************************************************
 *  function :    led_clear
 ******************************************************************************/
void led_clear(void) 
{
	gpio_clear(GPIOC, 13);
}


/*******************************************************************************
 *  function :    led_toogle
 ******************************************************************************/
void led_toogle(void) 
{
   gpio_toogle(GPIOC, 13);
}
