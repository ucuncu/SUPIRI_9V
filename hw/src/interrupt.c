/******************************************************************************/
/** @file       interrupt.c
 *******************************************************************************
 *
 *  @brief      All application defined interrupt handlers
 *
 *  @author     wht4
 *
 *  @remark     Last modifications
 *                 \li V1.0, February 2016, wht4, initial release
 *
 ******************************************************************************/
/*
 *  functions  global:
 *              SysTick_Handler
 *              EXTI0_1_IRQHandler
 *              NMI_Handler
 *              HardFault_Handler
 *              SVC_Handler
 *              PendSV_Handler
 *  functions  local:
 *              .
 *
 ******************************************************************************/

/****** Header-Files **********************************************************/
#include "stm32l0xx.h"

#include "systick.h"
#include "btn.h"
#include "rtc.h"
#include "hw.h"
#include "selenoid.h"
#include "nxp_lcd_driver.h"

extern void wakeUp(void);
/****** Macros ****************************************************************/

/****** Data types ************************************************************/

/****** Function prototypes ****************************************************/

/****** Data ******************************************************************/

/****** Implementation ********************************************************/

/*******************************************************************************
 *  function :    SysTick_Handler
 ******************************************************************************/
void
SysTick_Handler(void) {

    systick_irq();
}




/*******************************************************************************
 *  function :    RCC_CRS_IRQHandler
 ******************************************************************************/
void RCC_CRS_IRQHandler(void) 
{
	RCC->CIFR &= ~RCC_CIFR_HSIRDYF; // clear interrupt flag
	
	/* Select HSI as system clock */
	RCC->CFGR |= RCC_CFGR_SW_HSI | RCC_CFGR_SWS_HSI;

	/* Select MSI off */
	RCC->CR &= ~RCC_CR_MSION;
	
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
	
	fHSIOscillatorInit = 0;
	NVIC_SetPriority(RCC_CRS_IRQn, 0); /**/ 
	NVIC_DisableIRQ(RCC_CRS_IRQn); /*DISABLE BUTTON INTERRUPT*/ 

}



/*******************************************************************************
 *  function :    EXTI0_1_IRQHandler
 ******************************************************************************/
void RTC_IRQHandler(void) 
{
/*  Check line 17 has triggered the IT */
 if ((EXTI->PR & EXTI_PR_PR17) != 0)  
 {  
		if (RTC->ISR & RTC_ISR_ALRAF) 
		{
			RTC->ISR &= ~RTC_ISR_ALRAF; // clear Alarm A flag
			fSetNextAlarm = 1;
			if(!sleepCounter)
			{	
				fWakeUpFromRTC = 1;				
				wakeUp();
			}
		}
		
		if (RTC->ISR & RTC_ISR_ALRBF) 
		{
			RTC->ISR &= ~RTC_ISR_ALRBF; // clear Alarm B flag
			fResetLCDBufferwithDelay = 1; // gün dönümünde uyandiginda, RTC date registeri geç update ediyor eski günün üstüne yazip ekrani bozuyor, gecikmeli bir sekilde ekrani temizleyerek sorunu çözebildik.
			resetLCDBufferDelayMs = 100; // 100ms sonra ekran resetlenecek.
			++delayed_irr_counter;
			invalidPasswordCnt = 0;  // her gün 00'da counter sifirlansin. Örn; günde X defa hatali girme hakki var
			fproducerPasswordNeeded = 0;
			fUpdateInvPassCnt = 1;
			
			if((fStartManuelWork == 0) & (fStartAllRelayManuelWork == 0) & (fAnyRelayON == 0))
			{
				fUpdateRtcAlarm = 1;
			}
			if(fAnyRelayON)
			{
				fcelenderPlanDelayed = 1;
			}
			if(!sleepCounter)
			{
				fWakeUpFromRTC = 1;		
				wakeUp();
			}
		}
		 EXTI->PR |= EXTI_PR_PR17;
	}
}



/*******************************************************************************
 *  function :    EXTI0_1_IRQHandler
 ******************************************************************************/
void
EXTI0_1_IRQHandler(void) {

	/* Is there an interrupt on line 0 when device in sleep mode */
	if((EXTI->PR & EXTI_PR_PR0) != 0) 
	{
		if(fsleepEnable)
		{
			//fWakeUpButtonPressed = 1;
			btn_isr(BTN_1);
		}
			EXTI->PR |= EXTI_PR_PR0;
	}
}


/*******************************************************************************
 *  function :    NMI_Handler
 ******************************************************************************/
void
NMI_Handler(void) {

    /* Default handler */
}


/*******************************************************************************
 *  function :    HardFault_Handler
 ******************************************************************************/
void
HardFault_Handler(void) {

    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
			NVIC_SystemReset();
    }
}


/*******************************************************************************
 *  function :    SVC_Handler
 ******************************************************************************/
void
SVC_Handler(void) {

    /* Default handler */
}


/*******************************************************************************
 *  function :    PendSV_Handler
 ******************************************************************************/
void
PendSV_Handler(void) {

    /* Default handler */
}

/*******************************************************************************
 *  function :    I2C1_IRQHandler
 ******************************************************************************/
void
I2C1_IRQHandler(void) {

    /* Default handler */
	uint32_t I2C_InterruptStatus = I2C1->ISR; /* Get interrupt status */
  
}





