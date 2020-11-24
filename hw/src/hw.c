/******************************************************************************/
/** @file       hw.c
 *******************************************************************************
 *
 *  @brief      Module for initializing the attached hw
 *
 *  @author     wht4
 *
 *  @remark     Last modifications
 *                 \li V1.0, February 2016, wht4, initial release
 *
 ******************************************************************************/
/*
 *  functions  global:
 *              hw_init
 *  functions  local:
 *              hw_initSysclock
 *
 ******************************************************************************/

/****** Header-Files **********************************************************/
#include "stm32l0xx.h"

#include "config.h"
#include "led.h"
#include "lcd.h"
#include "button.h"
#include "selenoid.h"
#include "systick.h"
#include "btn.h"
#include "gpio.h"
#include "i2c.h"
#include "rtc.h"
#include "adc.h"
#include "nxp_lcd_driver.h"

/****** Macros ****************************************************************/

/****** Data types ************************************************************/

/****** Function prototypes ****************************************************/
void adc_con_pins_init(void);


/****** Data ******************************************************************/

/****** Implementation ********************************************************/


/*******************************************************************************
 *  function :    hw_initSysclock
 ******************************************************************************/
/** @brief        This function configures the system clock @16MHz and voltage
 *                scale 1 assuming the registers have their reset value before
 *                the call.
 *                <p>
 *                POWER SCALE   = RANGE 1
 *                SYSTEM CLOCK  = PLL MUL8 DIV2
 *                PLL SOURCE    = HSI/4
 *                FLASH LATENCY = 0
 *
 *  @copyright    Licensed under MCD-ST Liberty SW License Agreement V2,
 *                (the "License");
 *                You may not use this file except in compliance with the
 *                License. You may obtain a copy of the License at:
 *                http://www.st.com/software_license_agreement_liberty_v2
 *
 *  @type         global
 *
 *  @param[in]    in
 *  @param[out]   out
 *
 *  @return       void
 *
 ******************************************************************************/
void hw_initSysclock(void) {

	/* Enable power interface clock */
	RCC->APB1ENR |= (RCC_APB1ENR_PWREN);

	///* Select voltage scale 1 (1.65V - 1.95V) i.e. (01) */
	///* for VOS bits in PWR_CR */
	PWR->CR = (PWR->CR & ~(PWR_CR_VOS)) | PWR_CR_VOS_0;  // 
  
	/*!< Enable HSI READY interrupt */
  //RCC->CIER |= RCC_CICR_HSIRDYC;
	
	//NVIC_SetPriority(RCC_CRS_IRQn, 0); /**/ 
	//NVIC_EnableIRQ(RCC_CRS_IRQn); /*ENABLE BUTTON INTERRUPT*/ 
	
	/* Enable HSI divided by 1 in RCC-> CR */
	//RCC->CR |= RCC_CR_HSION | RCC_CR_HSIKERON;
	
	
	///* Wait for HSI ready flag and HSIDIV flag */
	////while ((RCC->CR & RCC_CR_HSIRDY) != RCC_CR_HSIRDY) {}
	
////	/* Select HSI as system clock */
////	RCC->CFGR |= RCC_CFGR_SW_HSI;

////	/*  Wait for clock switched on PLL */
////	while ((RCC->CFGR & RCC_CFGR_SW_HSI)  == 0) {}
}



/*******************************************************************************
 *  function :    hw_init
 ******************************************************************************/
void hw_init(void) {
	//
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	
	RCC->APB2SMENR &= ~RCC_APB2SMENR_SYSCFGSMEN; /* (1) */
	RCC->APB2SMENR &= ~RCC_APB2SMENR_TIM21SMEN; /* (1) */
	RCC->APB2SMENR &= ~RCC_APB2SMENR_TIM22SMEN; /* (1) */
	RCC->APB2SMENR &= ~RCC_APB2SMENR_ADC1SMEN; /* (1) */
	RCC->APB2SMENR &= ~RCC_APB2SMENR_SPI1SMEN; /* (1) */
	RCC->APB2SMENR &= ~RCC_APB2SMENR_USART1SMEN; /* (1) */
	//RCC->APB2SMENR &= ~RCC_APB2SMENR_DBGMCUSMEN; /* (1) */
	//
	
	RCC->AHBSMENR &= ~0x01000000; /* (1) */
	RCC->AHBSMENR &= ~RCC_AHBSMENR_DMA1SMEN; /* (1) */
	RCC->AHBSMENR &= ~RCC_AHBSMENR_DMA1SMEN; /* (1) */	
	RCC->AHBSMENR &= ~RCC_AHBSMENR_MIFSMEN; /* (1) */	
	RCC->AHBSMENR &= ~RCC_AHBSMENR_SRAMSMEN; /* (1) */	
	RCC->AHBSMENR &= ~RCC_AHBSMENR_CRCSMEN; /* (1) */	
	RCC->AHBSMENR &= ~RCC_AHBSMENR_TSCSMEN; /* (1) */	
	RCC->AHBSMENR &= ~RCC_AHBSMENR_RNGSMEN; /* (1) */	
	//
	RCC->APB1SMENR = 0; /* (1) */	
	//
	RCC->IOPSMENR &= ~RCC_IOPSMENR_GPIOASMEN; /* (1) */	
	//RCC->IOPSMEN = &= ~RCC_IOPSMENR_GPIOBSMEN; /* (1) */	
	RCC->IOPSMENR &= ~RCC_IOPSMENR_GPIOCSMEN; /* (1) */	
	RCC->IOPSMENR &= ~RCC_IOPSMENR_GPIODSMEN; /* (1) */	
	RCC->IOPSMENR &= ~RCC_IOPSMENR_GPIOHSMEN; /* (1) */		

	//RCC->AHBENR &= ~RCC_AHBENR_MIFEN; /* (1) */			
	
	//	
	I2C_Init();
	asm("nop");
	asm("nop");	
	asm("nop");
	asm("nop");   
	asm("nop");
	asm("nop");	
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");	
	asm("nop");
	asm("nop");
	//
	nxpInit();
	//
	asm("nop");
	asm("nop");	
	asm("nop");
	asm("nop");   
	asm("nop");
	asm("nop");	
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");	
	asm("nop");
	asm("nop");
	//
//	led_init();
//	led_clear();
	button_init();
	adc_con_pins_init();
	//	
	SetClockForADC();
	ConfigureADC();
	//CalibrateADC(); 
	EnableADC(); 
	//ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */	
	//	
	Configure_DBG();
	Configure_EXTI(); // configure button interrupt to wakeup, but disable IRQ
	//	
	adc_sens_read_disable();
	adc_bat_read_enable();
	
}


void arrangeGPIOforSleep(void)
{
		GpioInit_t tGpioButtonInit = {GPIO_MODE_INPUT,
                            GPIO_OUTPUT_PUSH_PULL,
                            GPIO_SPEED_MEDIUM,
                            GPIO_PULL_UP
                           };					 
		
		GpioInit_t tGpioInit = {GPIO_MODE_OUTPUT,
                            GPIO_OUTPUT_PUSH_PULL,
                            GPIO_SPEED_MEDIUM,
                            GPIO_PULL_NON
                           };
		
		GpioInit_t tGpioAnalogInit = {GPIO_MODE_ANALOG,
                            GPIO_OUTPUT_PUSH_PULL,
                            GPIO_SPEED_MEDIUM,
                            GPIO_PULL_NON
                           };
													 
													 
    /* Enable the peripheral clocks */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
		RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
		RCC->IOPENR |= RCC_IOPENR_GPIOHEN;													 
													 
    /* Select OUTPUT mode (00) on GPIOA pin 0 */
    //gpio_init(GPIOA, 0, &tGpioAnalogInit);										 
													 
		/* Select OUTPUT mode (00) on GPIOA pin 1 */
    gpio_init(GPIOA, 1, &tGpioInit);
		gpio_clear(GPIOA, 1);											 
													 
    /* Select OUTPUT mode (00) on GPIOA pin 2 */
    gpio_init(GPIOA, 2, &tGpioInit);
		gpio_clear(GPIOA, 2);											 
													 
		/* Select OUTPUT mode (00) on GPIOA pin 3 */
    gpio_init(GPIOA, 3, &tGpioInit);
		gpio_clear(GPIOA, 3);
		
		/* Select OUTPUT mode (00) on GPIOA pin 4 */
    gpio_init(GPIOA, 4, &tGpioInit);
		gpio_clear(GPIOA, 4);

    /* Select OUTPUT mode (00) on GPIOA pin 5 */
    gpio_init(GPIOA, 5, &tGpioInit);
		gpio_clear(GPIOA, 5);
		
		/* Select OUTPUT mode (00) on GPIOA pin 6 */
    gpio_init(GPIOA, 6, &tGpioInit);
		gpio_clear(GPIOA, 6);
		
    /* Select OUTPUT mode (00) on GPIOA pin 7 */
    gpio_init(GPIOA, 7, &tGpioInit);
		gpio_clear(GPIOA, 7);
		
		/* Select OUTPUT mode (00) on GPIOA pin 8 */
    gpio_init(GPIOA, 8, &tGpioInit);
		gpio_clear(GPIOA, 8);
		
		/* Select OUTPUT mode (00) on GPIOA pin 9 */
    gpio_init(GPIOA, 9, &tGpioInit);
		gpio_clear(GPIOA, 9);

    /* Select OUTPUT mode (00) on GPIOA pin 10 */
    gpio_init(GPIOA, 10, &tGpioInit);
		gpio_clear(GPIOA, 10);
		
		/* Select OUTPUT mode (00) on GPIOA pin 11 */
    gpio_init(GPIOA, 11, &tGpioInit);
		gpio_clear(GPIOA, 11);
		
    /* Select OUTPUT mode (00) on GPIOA pin 12 */
    gpio_init(GPIOA, 12, &tGpioInit);
		gpio_clear(GPIOA, 12);
		
//		/* Select OUTPUT mode (00) on GPIOA pin 13 */
    gpio_init(GPIOA, 13, &tGpioInit);
		gpio_set(GPIOA, 13);
//		
//		/* Select OUTPUT mode (00) on GPIOA pin 14 */
    gpio_init(GPIOA, 14, &tGpioAnalogInit);	
		//gpio_clear(GPIOA, 14);
		
    /* Select OUTPUT mode (00) on GPIOA pin 15 */
		gpio_init(GPIOA, 15, &tGpioAnalogInit);	
		//gpio_clear(GPIOA, 15);
															 
    /* Select INPUT mode (00) on GPIOB pin 0 */
    gpio_init(GPIOB, 0, &tGpioButtonInit);
		/* Select INPUT mode (00) on GPIOB pin 1 */
		
    gpio_init(GPIOB, 1, &tGpioButtonInit);
		
    /* Select INPUT mode (00) on GPIOB pin 2 */
    gpio_init(GPIOB, 2, &tGpioButtonInit);
		
		/* Select INPUT mode (00) on GPIOB pin 3 */
    gpio_init(GPIOB, 3, &tGpioButtonInit);
		
		/* Select INPUT mode (00) on GPIOB pin 4 */
    gpio_init(GPIOB, 4, &tGpioButtonInit);	
		
    /* Select OUTPUT mode (00) on GPIOB pin 5 */
    gpio_init(GPIOB, 5, &tGpioInit);
		gpio_clear(GPIOB, 5);
		
//		/* Select OUTPUT mode (00) on GPIOB pin 6 */
//    gpio_init(GPIOB, 6, &tGpioInit);
//		
//    /* Select OUTPUT mode (00) on GPIOB pin 7 */
//    gpio_init(GPIOB, 7, &tGpioInit);
//		
		/* Select OUTPUT mode (00) on GPIOB pin 8 */
    gpio_init(GPIOB, 8, &tGpioInit);
		gpio_clear(GPIOB, 8);
		
		/* Select OUTPUT mode (00) on GPIOB pin 9 */
    gpio_init(GPIOB, 9, &tGpioInit);	
		gpio_clear(GPIOB, 9);
		
    /* Select OUTPUT mode (00) on GPIOB pin 10 */
    gpio_init(GPIOB, 10, &tGpioInit);
		gpio_clear(GPIOB, 10);
		
		/* Select OUTPUT mode (00) on GPIOB pin 11 */
    gpio_init(GPIOB, 11, &tGpioInit);
		gpio_clear(GPIOB, 11);
		
    /* Select OUTPUT mode (00) on GPIOB pin 12 */
    gpio_init(GPIOB, 12, &tGpioInit);
		gpio_clear(GPIOB, 12);
		
		/* Select OUTPUT mode (00) on GPIOB pin 13 */
    gpio_init(GPIOB, 13, &tGpioInit);
		gpio_clear(GPIOB, 13);
		
		/* Select OUTPUT mode (00) on GPIOB pin 14 */
    gpio_init(GPIOB, 14, &tGpioInit);	
		gpio_clear(GPIOB, 14);
		
    /* Select OUTPUT mode (00) on GPIOB pin 15 */
		gpio_init(GPIOB, 15, &tGpioInit);
		gpio_clear(GPIOB, 15);
	
	
//			/* Select OUTPUT mode (00) on GPIOH pin 0 */
    gpio_init(GPIOH, 0, &tGpioAnalogInit);	
//		
//    /* Select OUTPUT mode (00) on GPIOH pin 1 */
		gpio_init(GPIOH, 1, &tGpioAnalogInit);
	
	
}


/*******************************************************************************
 *  function :    led_init
 ******************************************************************************/
void adc_con_pins_init(void) 
{

    GpioInit_t tGpioInit = {GPIO_MODE_OUTPUT,
                            GPIO_OUTPUT_PUSH_PULL,
                            GPIO_SPEED_MEDIUM,
                            GPIO_PULL_DOWN
                           };

    /* Enable the peripheral clock of GPIOA */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    /* Select output mode (01) on GPIOA pin 1 */
    gpio_init(GPIOA, 1, &tGpioInit);

    /* Select output mode (01) on GPIOA pin 2 */
    gpio_init(GPIOA, 2, &tGpioInit);
}

/*******************************************************************************
 *  function :    relayControl
 ******************************************************************************/
void relayControl (void)
{
	
}

