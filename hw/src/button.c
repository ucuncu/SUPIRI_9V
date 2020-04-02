/******************************************************************************/
/** @file       button.c
 *******************************************************************************
 *
 *  @remark     Last modifications
 *                 \li V1.0, February 2016, wht4, initial release
 *
 ******************************************************************************/
/*
 *              .
 *
 ******************************************************************************/

/****** Header-Files **********************************************************/
#include "button.h"
#include "gpio.h"
#include "led.h"
#include "systick.h"
#include "selenoid.h"
#include "nxp_lcd_driver.h"
#include "rtc.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>

unsigned char butdat = 0xFF;	// Button data
unsigned char prvdat = 0;			// Previous button data
unsigned char button = 0xFF;	// Button data store
unsigned char prvbut = 0;			// Previous button data
unsigned char presdb = 0;			// Pressed button
unsigned char vdcnt = 0;			// Valid data counter
unsigned char lvdcnt = 0;			// Long valid data counter
unsigned char lvd2cnt = 0;			// Long valid data counter

/****** Macros ****************************************************************/

/****** Data types ************************************************************/

/****** Function prototypes ****************************************************/
void clearAllDetailAdjustment(void);
uint32_t bcd2cnv(uint32_t bcdData);
/****** Data ******************************************************************/

/****** Implementation ********************************************************/

/* Returns the number of days to the start of the specified year, taking leap
 * years into account, but not the shift from the Julian calendar to the
 * Gregorian calendar. Instead, it is as though the Gregorian calendar is
 * extrapolated back in time to a hypothetical "year zero".
 */
int leap (int year)
{
  return year*365 + (year/4) - (year/100) + (year/400);
}

/* Returns a number representing the number of days since March 1 in the
 * hypothetical year 0, not counting the change from the Julian calendar
 * to the Gregorian calendar that occured in the 16th century. This
 * algorithm is loosely based on a function known as "Zeller's Congruence".
 * This number MOD 7 gives the day of week, where 0 = Monday and 6 = Sunday.
 */
int zeller (int year, int month, int day)
{
  year += ((month+9)/12) - 1;
  month = (month+9) % 12;
  return leap (year) + month*30 + ((6*month+5)/10) + day + 1;
}

/* Returns the day of week (1=Monday, 7=Sunday) for a given date.
 */
int dow (int year, int month, int day)
{
  return (zeller (year, month, day) % 7) + 1;
}


/*******************************************************************************
 *  function :    led_init
 ******************************************************************************/
void button_init(void) {

    GpioInit_t tGpioInit = {GPIO_MODE_INPUT,
                            GPIO_OUTPUT_PUSH_PULL,
                            GPIO_SPEED_MEDIUM,
                            GPIO_PULL_UP
                           };

    /* Enable the peripheral clock of GPIOB */
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    /* Select INPUT mode (00) on GPIOB pin 0 */
    gpio_init(GPIOB, 0, &tGpioInit);
		/* Select INPUT mode (00) on GPIOB pin 1 */
    gpio_init(GPIOB, 1, &tGpioInit);
    /* Select INPUT mode (00) on GPIOB pin 2 */
    gpio_init(GPIOB, 2, &tGpioInit);
		/* Select INPUT mode (00) on GPIOB pin 3 */
    gpio_init(GPIOB, 3, &tGpioInit);
		/* Select INPUT mode (00) on GPIOB pin 4 */
    gpio_init(GPIOB, 4, &tGpioInit);													 
}


void Configure_DBG(void)
{
  /* Enable the peripheral clock of DBG register */
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
  
  DBGMCU->CR |= DBGMCU_CR_DBG_STOP; /* To be able to debug in stop mode */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; /* To enter deep sleep when __WFI() */
  PWR->CR &=~ PWR_CR_PDDS; /* Select STOP mode */
	//PWR_CR_ULP set edilip ultralow power mode aktif edilebilir.
}

/**
  * Brief   This function configures EXTI.
  * Param   None
  * Retval  None
  */
void Configure_EXTI(void)
{
  /* Configure Syscfg, exti and nvic for pushbutton PB0 */
  /* (1) PB0 PB1 PB2 PB3 PB4 as source input */
  /* (2) Unmask port 0 */
  /* (3) Falling edge */
  /* (4) Set priority */
  /* (5) Enable EXTI0_1_IRQn */
  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PB; /* (1) */ 
	
  EXTI->IMR |= EXTI_IMR_IM0; /* (2) */ 
  EXTI->RTSR |= EXTI_FTSR_TR0; /* (3) */ 
	//
	
	NVIC_SetPriority(EXTI0_1_IRQn, 0); /**/ 
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	//			
}


/*=====================================================================
				*********** ANY BUTTON PRESSED ************			
=====================================================================*/

void anyButtonPressed(void) 
{
	sleepCounter = SLEEP_COUNTER_VAL;      // her tus basiminda uyku sayacini yeniden yükle
	flashingCnt = 0;
	flashingCnt2 = 0;
	flashingCnt3 = 0;
	flashingCnt4 = 0;
	flashingCnt5 = 0;
	flashingCnt6 = 0;
	flashingCnt7 = 0;
	flashingCnt8 = 0;	
	LCDBufferReset();
	fPlanSelenoidSettings = 0;
}

/*=====================================================================
				*********** UPDATE RTC HOUR TENS ************			
=====================================================================*/
void updateHour(uint8_t operation)
{
	static uint32_t NewHourTens=0;
	static uint32_t NewHour=0;
	static uint32_t NewHourUnits=0;
	static uint32_t NewTime=0;
	static uint32_t currentDate=0;
	static uint32_t maxVal=23;
	
	NewHourTens = (RTC->TR & RTC_TR_HT)>>20;	
	NewHourUnits = (RTC->TR & RTC_TR_HU)>>16;
	currentDate = RTC->DR;
	NewHour = NewHourTens*10 + NewHourUnits;
	
	if(operation == 1) // + pressed
	{
		++NewHour;
		if(NewHour == maxVal +1)
		{
			NewHour = 0;
		}
	}
	else // - pressed
	{
		
		if(NewHour == 0)
		{
			NewHour = maxVal;
		}
		else
		{
			--NewHour;
		}

	}
	
	NewHour = bcd2cnv(NewHour); 
	NewHourTens = (NewHour & 0x30)>>4;
	NewHourUnits = NewHour & 0x0F;
	
	NewTime = (RTC->TR & ~(RTC_TR_HU|RTC_TR_HT)) | (NewHourUnits<<16) | (NewHourTens<<20);

	Init_RTC(NewTime,currentDate);
}


/*=====================================================================
				*********** UPDATE RTC HOUR TENS ************			
=====================================================================*/
void updateMinute(uint8_t operation)
{
	static uint32_t NewMinuteTens=0;
	static uint32_t NewMinuteUnits=0;
	static uint32_t NewMinute=0;
	static uint32_t NewTime=0;
	static uint32_t currentDate=0;
	static uint32_t maxVal=59;
	
	NewMinuteTens = (RTC->TR & RTC_TR_MNT)>>12;
	NewMinuteUnits = (RTC->TR & RTC_TR_MNU)>>8;
	currentDate = RTC->DR;
	
	NewMinute = NewMinuteTens*10 + NewMinuteUnits;
	
	if(operation == 1) // + pressed
	{
		++NewMinute;
		if(NewMinute == maxVal + 1)
		{
			NewMinute = 0;
		}
	}
	else // - pressed
	{
		
		if(NewMinute == 0)
		{
			NewMinute = maxVal;
		}
		else
		{
			--NewMinute;
		}

	}
	NewMinute = bcd2cnv(NewMinute); 
	NewMinuteTens = (NewMinute & 0x70)>>4;
	NewMinuteUnits = NewMinute & 0x0F;
	
	NewTime = (RTC->TR & ~(RTC_TR_MNU | RTC_TR_MNT)) | (NewMinuteUnits<<8) | (NewMinuteTens<<12);
	Init_RTC(NewTime,currentDate);
}


/*=====================================================================
				*********** UPDATE RTC DAY ************			
=====================================================================*/
void updateDay(uint8_t operation)
{
	static uint32_t NewDate=0;
	static uint32_t currentTime=0;
	static uint32_t currentYear=0;
	static uint32_t currentMonth=0;
	static uint32_t currentDay=0;
	static uint32_t DayOfWeek=0;
	static uint32_t maxVal=31;
	
	currentYear = (((RTC->DR & RTC_DR_YT)>>20)*10 + ((RTC->DR & RTC_DR_YU)>>16));
	currentMonth = (((RTC->DR & RTC_DR_MT)>>12)*10 + ((RTC->DR & RTC_DR_MU)>>8));
	currentDay = (((RTC->DR & RTC_DR_DT)>>4)*10 + ((RTC->DR & RTC_DR_DU)));
	currentTime = RTC->TR;
//	

	if(currentMonth == 1)
	{
		 maxVal=31;
	}
	else if(currentMonth == 2)
	{
		if(currentYear%4)
		{
			maxVal=28;
		}
		else
		{
			maxVal=29;
		}
	}
	else if(currentMonth == 3)
	{
		maxVal=31;
	}
	else if(currentMonth == 4)
	{
		maxVal=30;
	}
	else if(currentMonth == 5)
	{
		maxVal=31;
	}
	else if(currentMonth == 6)
	{
		maxVal=30;
	}
	else if(currentMonth == 7)
	{
		maxVal=31;
	}
	else if(currentMonth == 8)
	{
		maxVal=31;
	}
	else if(currentMonth == 9)
	{
		maxVal=30;
	}
	else if(currentMonth == 10)
	{
		maxVal=31;
	}
	else if(currentMonth == 11)
	{
		maxVal=30;
	}
	else if(currentMonth == 12)
	{
		maxVal=31;
	}	
	
	currentTime = RTC->TR;
	if(operation == 1) // + pressed
	{
		++currentDay;
		if(currentDay == maxVal + 1)
		{
			currentDay = 1;
		}
	}
	else // - pressed
	{
		
		if(currentDay == 1)
		{
			currentDay = maxVal;
		}
		else
		{
			--currentDay;
		}
	}
	
	DayOfWeek = dow(currentYear,currentMonth,currentDay);
	
	currentDay = bcd2cnv(currentDay); 
	NewDate = ((RTC->DR & ~(RTC_DR_DT|RTC_DR_DU|RTC_DR_WDU)) | currentDay | (DayOfWeek<<13));	
	Init_RTC(currentTime,NewDate);
}

/*=====================================================================
				*********** UPDATE RTC MONTH ************			
=====================================================================*/
void updateMonth(uint8_t operation)
{
	static uint32_t NewMonth=0;
	static uint32_t currentYear=0;
	static uint32_t currentMonth=0;
	static uint32_t currentDay=0;
	static uint32_t currentTime=0;
	static uint32_t DayOfWeek=0;
	static uint32_t maxVal=12;
	
	currentYear = (((RTC->DR & RTC_DR_YT)>>20)*10 + ((RTC->DR & RTC_DR_YU)>>16));
	currentMonth = (((RTC->DR & RTC_DR_MT)>>12)*10 + ((RTC->DR & RTC_DR_MU)>>8));
	currentDay = (((RTC->DR & RTC_DR_DT)>>4)*10 + ((RTC->DR & RTC_DR_DU)));
	currentTime = RTC->TR;


	if(operation == 1) // + pressed
	{
		++currentMonth;
		if(currentMonth == maxVal + 1)
		{
			currentMonth = 1;
		}
	}
	else // - pressed
	{
		
		if(currentMonth == 1)
		{
			currentMonth = maxVal;
		}
		else
		{
			--currentMonth;
		}
	}

	DayOfWeek = dow(currentYear,currentMonth,currentDay);	
	
	currentMonth = bcd2cnv(currentMonth); 
	NewMonth = ((RTC->DR & ~(RTC_DR_MT|RTC_DR_MU|RTC_DR_WDU)) | (currentMonth<<8)  | (DayOfWeek<<13));
	Init_RTC(currentTime,NewMonth);
}

/*=====================================================================
				*********** UPDATE RTC MONTH ************			
=====================================================================*/
void updateYear(uint8_t operation)
{
	static uint32_t NewYear=0;
	static uint32_t currentYear=0;
	static uint32_t currentMonth=0;
	static uint32_t currentDay=0;
	static uint32_t currentTime=0;
	static uint32_t DayOfWeek=0;;
	static uint32_t maxVal=99;
	
	currentYear = (((RTC->DR & RTC_DR_YT)>>20)*10 + ((RTC->DR & RTC_DR_YU)>>16));
	currentMonth = (((RTC->DR & RTC_DR_MT)>>12)*10 + ((RTC->DR & RTC_DR_MU)>>8));
	currentDay = (((RTC->DR & RTC_DR_DT)>>4)*10 + ((RTC->DR & RTC_DR_DU)));
	
	currentTime = RTC->TR;


	if(operation == 1) // + pressed
	{
		++currentYear;
		if(currentYear == maxVal + 1)
		{
			currentYear = 0;
		}
	}
	else // - pressed
	{
		
		if(currentYear == 0)
		{
			currentYear = maxVal;
		}
		else
		{
			--currentYear;
		}
	}
	
	DayOfWeek = dow(currentYear,currentMonth,currentDay);
	
	currentYear = bcd2cnv(currentYear); 
	NewYear = ((RTC->DR & ~(RTC_DR_YT|RTC_DR_YU|RTC_DR_WDU)) | (currentYear<<16) | (DayOfWeek<<13));	
	Init_RTC(currentTime,NewYear);
}



/*=====================================================================
			*********** CONVERT BINARY TO DECÝMAL ************			
=====================================================================*/
/* This subroutine converts the binary data in bcdData to decimal value.
As example; when bcdData is "00100000"  (means it is 32 in decimal), the 
output of this subroutine is "00110010" ( Most significant four bit "3", 
least significant four bit "2")
=====================================================================*/
uint32_t bcd2cnv(uint32_t bcdData)
{
	uint32_t tmp1 = 0;
	bcdData = bcdData & 0x000000FF;  //?????
	if(bcdData >99)
	{
		return 0;
	}
	while (bcdData >= 10)
	{
		bcdData = bcdData - 10;
		tmp1 = tmp1 + 16;
	}
	bcdData = bcdData| tmp1;
	
	return bcdData;
}


/*=====================================================================
			*********** SELECTED SELENOID UPDATE ************			
=====================================================================*/

void selectedSelenoidUpdate(uint8_t option) 
{
	if(option) // + button pressed
	{
		++selectedSelenoid;
		if(selectedSelenoid == 5)
		{
			selectedSelenoid = 1;
		}
	}
	else
	{
		if(selectedSelenoid == 1)
		{
			selectedSelenoid = 4;
		}
		else
		{
			--selectedSelenoid;
		}
	}
}


/*=====================================================================
			*********** SELECTED SELENOID WORK DURATION ADJ ************			
=====================================================================*/

void selenoidWorkDurAdj(uint8_t option) 
{
	if(option) // + button pressed
	{
		++sel_work_duration[selectedSelenoid-1];
		if(sel_work_duration[selectedSelenoid-1] == 100)
		{
			sel_work_duration[selectedSelenoid-1] = 0;
		}
	}	
	else
	{
		if(sel_work_duration[selectedSelenoid-1] == 0)
		{
			sel_work_duration[selectedSelenoid-1] = 99;
		}
		else
		{
			--sel_work_duration[selectedSelenoid-1];
		}					
	}
}			

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void SelenoidStartTime_1_HourAdjust(uint8_t option) 
{
	if(option) // + button pressed
	{
		++sel_start_time_hour[selectedSelenoid-1][0];
		if(sel_start_time_hour[selectedSelenoid-1][0] == 24)
		{
			sel_start_time_hour[selectedSelenoid-1][0] = 0;
		}
	}
	else
	{
		if(sel_start_time_hour[selectedSelenoid-1][0] == 0)
		{
			sel_start_time_hour[selectedSelenoid-1][0] = 23;
		}
		else
		{
			--sel_start_time_hour[selectedSelenoid-1][0];
		}
	}
}

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void SelenoidStartTime_1_MinuteAdjust(uint8_t option) 
{
	if(option) // + button pressed
	{
		++sel_start_time_min[selectedSelenoid-1][0];
		if(sel_start_time_min[selectedSelenoid-1][0] == 60)
		{
			sel_start_time_min[selectedSelenoid-1][0] = 0;
		}
	}
	else
	{
		if(sel_start_time_min[selectedSelenoid-1][0] == 0)
		{
			sel_start_time_min[selectedSelenoid-1][0] = 59;
		}
		else
		{
			--sel_start_time_min[selectedSelenoid-1][0];
		}
	}
}

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void SelenoidStartTime_2_HourAdjust(uint8_t option) 
{
	if(option) // + button pressed
	{
		if((sel_start_time_hour[selectedSelenoid-1][1] ==0xFF)|(sel_start_time_min[selectedSelenoid-1][1] ==0xFF))
		{
			sel_start_time_min[selectedSelenoid-1][1] = 0;
			sel_start_time_hour[selectedSelenoid-1][1] = 0;
		}
		else
		{
			++sel_start_time_hour[selectedSelenoid-1][1];
			if(sel_start_time_hour[selectedSelenoid-1][1] == 24)
			{
				sel_start_time_hour[selectedSelenoid-1][1] = 0;
			}
		}
	}
	else
	{
		if((sel_start_time_hour[selectedSelenoid-1][1] == 0)&(sel_start_time_min[selectedSelenoid-1][1] == 0))
		{
			sel_start_time_min[selectedSelenoid-1][1] = 0xFF;
			sel_start_time_hour[selectedSelenoid-1][1] = 0xFF;
		}	
		else if(sel_start_time_hour[selectedSelenoid-1][1] == 0)
		{
			sel_start_time_hour[selectedSelenoid-1][1] = 23;
		}
		else if(sel_start_time_hour[selectedSelenoid-1][1] != 0xFF)
		{
			--sel_start_time_hour[selectedSelenoid-1][1];
		}
	}
}

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void SelenoidStartTime_2_MinuteAdjust(uint8_t option) 
{
	if(option) // + button pressed
	{
		if((sel_start_time_hour[selectedSelenoid-1][1] ==0xFF)|(sel_start_time_min[selectedSelenoid-1][1] ==0xFF))
		{
			sel_start_time_min[selectedSelenoid-1][1] = 0;
			sel_start_time_hour[selectedSelenoid-1][1] = 0;
		}
		else
		{
			++sel_start_time_min[selectedSelenoid-1][1];
			if(sel_start_time_min[selectedSelenoid-1][1] == 60)
			{
				sel_start_time_min[selectedSelenoid-1][1] = 0;
			}
		}
	}
	else
	{
		if((sel_start_time_hour[selectedSelenoid-1][1] == 0)&(sel_start_time_min[selectedSelenoid-1][1] == 0))
		{
			sel_start_time_min[selectedSelenoid-1][1] = 0xFF;
			sel_start_time_hour[selectedSelenoid-1][1] = 0xFF;
		}			
		else if(sel_start_time_min[selectedSelenoid-1][1] == 0)
		{
			sel_start_time_min[selectedSelenoid-1][1] = 59;
		}
		else if(sel_start_time_min[selectedSelenoid-1][1] != 0xFF)
		{
			--sel_start_time_min[selectedSelenoid-1][1];
		}
	}
}

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void SelenoidStartTime_3_HourAdjust(uint8_t option) 
{
	if(option) // + button pressed
	{
		if((sel_start_time_hour[selectedSelenoid-1][2] ==0xFF)|(sel_start_time_min[selectedSelenoid-1][2] ==0xFF))
		{
			sel_start_time_min[selectedSelenoid-1][2] = 0;
			sel_start_time_hour[selectedSelenoid-1][2] = 0;
		}
		else
		{
			++sel_start_time_hour[selectedSelenoid-1][2];
			if(sel_start_time_hour[selectedSelenoid-1][2] == 24)
			{
				sel_start_time_hour[selectedSelenoid-1][2] = 0;
			}
		}
	}
	else
	{
		if((sel_start_time_hour[selectedSelenoid-1][2] == 0)&(sel_start_time_min[selectedSelenoid-1][2] == 0))
		{
			sel_start_time_min[selectedSelenoid-1][2] = 0xFF;
			sel_start_time_hour[selectedSelenoid-1][2] = 0xFF;
		}			
		else if(sel_start_time_hour[selectedSelenoid-1][2] == 0)
		{
			sel_start_time_hour[selectedSelenoid-1][2] = 23;
		}
		else if(sel_start_time_hour[selectedSelenoid-1][2] != 0xFF)
		{
			--sel_start_time_hour[selectedSelenoid-1][2];
		}
	}
}

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void SelenoidStartTime_3_MinuteAdjust(uint8_t option) 
{
	if(option) // + button pressed
	{
		if((sel_start_time_hour[selectedSelenoid-1][2] ==0xFF)|(sel_start_time_min[selectedSelenoid-1][2] ==0xFF))
		{
			sel_start_time_min[selectedSelenoid-1][2] = 0;
			sel_start_time_hour[selectedSelenoid-1][2] = 0;
		}
		else
		{
			++sel_start_time_min[selectedSelenoid-1][2];
			if(sel_start_time_min[selectedSelenoid-1][2] == 60)
			{
				sel_start_time_min[selectedSelenoid-1][2] = 0;
			}
		}
	}
	else
	{
		if((sel_start_time_hour[selectedSelenoid-1][2] == 0)&(sel_start_time_min[selectedSelenoid-1][2] == 0))
		{
			sel_start_time_min[selectedSelenoid-1][2] = 0xFF;
			sel_start_time_hour[selectedSelenoid-1][2] = 0xFF;
		}			
		else if(sel_start_time_min[selectedSelenoid-1][2] == 0)
		{
			sel_start_time_min[selectedSelenoid-1][2] = 59;
		}
		else if(sel_start_time_min[selectedSelenoid-1][2] != 0xFF)
		{
			--sel_start_time_min[selectedSelenoid-1][2];
		}
	}
}

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void SelenoidStartTime_4_HourAdjust(uint8_t option) 
{
	if(option) // + button pressed
	{
		if((sel_start_time_hour[selectedSelenoid-1][3] ==0xFF)|(sel_start_time_min[selectedSelenoid-1][3] ==0xFF))
		{
			sel_start_time_min[selectedSelenoid-1][3] = 0;
			sel_start_time_hour[selectedSelenoid-1][3] = 0;
		}
		else
		{
			++sel_start_time_hour[selectedSelenoid-1][3];
			if(sel_start_time_hour[selectedSelenoid-1][3] == 24)
			{
				sel_start_time_hour[selectedSelenoid-1][3] = 0;
			}
		}
	}
	else
	{
		if((sel_start_time_hour[selectedSelenoid-1][3] == 0)&(sel_start_time_min[selectedSelenoid-1][3] == 0))
		{
			sel_start_time_min[selectedSelenoid-1][3] = 0xFF;
			sel_start_time_hour[selectedSelenoid-1][3] = 0xFF;
		}			
		else if(sel_start_time_hour[selectedSelenoid-1][3] == 0)
		{
			sel_start_time_hour[selectedSelenoid-1][3] = 23;
		}
		else if(sel_start_time_hour[selectedSelenoid-1][3] != 0xFF)
		{
			--sel_start_time_hour[selectedSelenoid-1][3];
		}
	}
}

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void SelenoidStartTime_4_MinuteAdjust(uint8_t option) 
{

	if(option) // + button pressed
	{
		if((sel_start_time_hour[selectedSelenoid-1][3] ==0xFF)|(sel_start_time_min[selectedSelenoid-1][3] ==0xFF))
		{
			sel_start_time_min[selectedSelenoid-1][3] = 0;
			sel_start_time_hour[selectedSelenoid-1][3] = 0;
		}
		else
		{
			++sel_start_time_min[selectedSelenoid-1][3];
			if(sel_start_time_min[selectedSelenoid-1][3] == 60)
			{
				sel_start_time_min[selectedSelenoid-1][3] = 0;
			}
		}
	}
	else
	{
		if((sel_start_time_hour[selectedSelenoid-1][3] == 0)&(sel_start_time_min[selectedSelenoid-1][3] == 0))
		{
			sel_start_time_min[selectedSelenoid-1][3] = 0xFF;
			sel_start_time_hour[selectedSelenoid-1][3] = 0xFF;
		}	
		else if(sel_start_time_min[selectedSelenoid-1][3] == 0)
		{
			sel_start_time_min[selectedSelenoid-1][3] = 59;
		}
		else if(sel_start_time_min[selectedSelenoid-1][3] != 0xFF)
		{
			--sel_start_time_min[selectedSelenoid-1][3];
		}
	}
}


/*=====================================================================
	 ***** SELECTED SELENOID DELAYED IRRIGATION VALUE ADJ ************			
=====================================================================*/

void DelayValueAdjust(uint8_t option) 
{
	delayed_irr_counter = 0;  // her ayar yapildiginda aralikli sulamanin ilk günü o gün olur.
	
	if(option) // + button pressed
	{
		++sel_delayed_irr_val[selectedSelenoid-1];
		if(sel_delayed_irr_val[selectedSelenoid-1] == 30)
		{
			sel_delayed_irr_val[selectedSelenoid-1] = 0;
		}
	}
	else
	{
		if(sel_delayed_irr_val[selectedSelenoid-1] == 0)
		{
			sel_delayed_irr_val[selectedSelenoid-1] = 29;
		}
		else
		{
			--sel_delayed_irr_val[selectedSelenoid-1];
		}
	}
}

/*=====================================================================
				*********** MANUEL ALL WORK DURATION ADJUST ************			
=====================================================================*/
void ManuelALLWorkDurationAdjust(uint8_t option)
{	
	if(option) // + button pressed
	{
		++all_manuel_work_duration;
		if(all_manuel_work_duration == 100)
		{
			all_manuel_work_duration = 0;
		}
	}	
	else
	{
		if(all_manuel_work_duration == 0)
		{
			all_manuel_work_duration = 99;
		}
		else
		{
			--all_manuel_work_duration;
		}					
	}	
}


/*=====================================================================
				*********** MANUEL WORK DURATION ADJUST ************			
=====================================================================*/
void ManuelWorkDurationAdjust(uint8_t option)
{	
	if(option) // + button pressed
	{
		++sel_manuel_work_duration[selectedSelenoid-1];
		if(sel_manuel_work_duration[selectedSelenoid-1] == 100)
		{
			sel_manuel_work_duration[selectedSelenoid-1] = 0;
		}
	}	
	else
	{
		if(sel_manuel_work_duration[selectedSelenoid-1] == 0)
		{
			sel_manuel_work_duration[selectedSelenoid-1] = 99;
		}
		else
		{
			--sel_manuel_work_duration[selectedSelenoid-1];
		}					
	}	
}

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void passwordAdjust(uint8_t digitNumber, uint8_t option) 
{
	if(option) // + button pressed
	{
		++tmpPassword[digitNumber-1];
		if(tmpPassword[digitNumber-1] == 10)
		{
			tmpPassword[digitNumber-1] = 0;
		}
	}
	else
	{
		if(tmpPassword[digitNumber-1] == 0)
		{
			tmpPassword[digitNumber-1] = 9;
		}
		else
		{
			--tmpPassword[digitNumber-1];
		}
	}
}

/*=====================================================================
	 ***** SELECTED SELENOID START TIME UPDATE ************			
=====================================================================*/

void producerPassAdj(uint8_t digitNumber, uint8_t option) 
{
	if(option) // + button pressed
	{
		++factoryPASSWORD[digitNumber-1];
		if(factoryPASSWORD[digitNumber-1] == 10)
		{
			factoryPASSWORD[digitNumber-1] = 0;
		}
	}
	else
	{
		if(factoryPASSWORD[digitNumber-1] == 0)
		{
			factoryPASSWORD[digitNumber-1] = 9;
		}
		else
		{
			--factoryPASSWORD[digitNumber-1];
		}
	}
}

/*=====================================================================
	 *****   CHECK PASSWORD  ************			
=====================================================================*/

void checkPasword(void) 
{
	if((password[0] == tmpPassword[0]) & (password[1] == tmpPassword[1]) & (password[2] == tmpPassword[2]) & (password[3] == tmpPassword[3]))
	{
		clearAllDetailAdjustment();
		LCDBufferReset();
		tmpPassword[0] = 0;
		tmpPassword[1] = 0;
		tmpPassword[2] = 0;
		tmpPassword[3] = 0;
		fHomePageWithPassword = 0;
		fhomePage = 1;
		fDeviceLockedWithPassword = 0;
	}
	else
	{
		tmpPassword[0] = 0;
		tmpPassword[1] = 0;
		tmpPassword[2] = 0;
		tmpPassword[3] = 0;
		fPasswordDigit1Adjust = 1;
		++invalidPasswordCnt;
		fUpdateInvPassCnt = 1;	
		if(invalidPasswordCnt >= 5)
		{
			fproducerPasswordNeeded = 1;
			fproducerPassword_1_Adj = 1;
		}
			
	}
}

void checkFactoryKey(void) 
{
	if((factoryPASSWORD[0] == factoryReseyKEY[0]) & (factoryPASSWORD[1] == factoryReseyKEY[1]) & (factoryPASSWORD[2] == factoryReseyKEY[2]) & (factoryPASSWORD[3] == factoryReseyKEY[3])& (factoryPASSWORD[4] == factoryReseyKEY[4])& (factoryPASSWORD[5] == factoryReseyKEY[5])& (factoryPASSWORD[6] == factoryReseyKEY[6])& (factoryPASSWORD[7] == factoryReseyKEY[7])& (factoryPASSWORD[8] == factoryReseyKEY[8])& (factoryPASSWORD[9] == factoryReseyKEY[9]))
	{
		clearAllDetailAdjustment();
		LCDBufferReset();
		fHomePageWithPassword = 0;
		fhomePage = 1;
		fDeviceLockedWithPassword = 0;
		fPasswordSetted = 0;
		fUpdatePassword = 1;
		invalidPasswordCnt = 0;
		fUpdateInvPassCnt = 1;
		tmpPassword[0] = 0;
		tmpPassword[1] = 0;
		tmpPassword[2] = 0;
		tmpPassword[3] = 0;
		fproducerPasswordNeeded = 0;
		fproducerPassword_1_Adj = 0;
		fproducerPassword_2_Adj = 0;
		fproducerPassword_3_Adj = 0;
		fproducerPassword_4_Adj = 0;
		fproducerPassword_5_Adj = 0;
		fproducerPassword_6_Adj = 0;
		fproducerPassword_7_Adj = 0;
		fproducerPassword_8_Adj = 0;
		fproducerPassword_9_Adj = 0;
		fproducerPassword_10_Adj = 0;			
		
	}
	else
	{
		fproducerPasswordNeeded = 0;
		fproducerPassword_1_Adj = 0;
		fproducerPassword_2_Adj = 0;
		fproducerPassword_3_Adj = 0;
		fproducerPassword_4_Adj = 0;
		fproducerPassword_5_Adj = 0;
		fproducerPassword_6_Adj = 0;
		fproducerPassword_7_Adj = 0;
		fproducerPassword_8_Adj = 0;
		fproducerPassword_9_Adj = 0;
		fproducerPassword_10_Adj = 0;	
		fproducerPasswordNeeded = 1;
		fproducerPassword_1_Adj = 1;	
	}
		factoryPASSWORD[0] = 0;
		factoryPASSWORD[1] = 0;
		factoryPASSWORD[2] = 0;
		factoryPASSWORD[3] = 0;	
		factoryPASSWORD[4] = 0;
		factoryPASSWORD[5] = 0;
		factoryPASSWORD[6] = 0;
		factoryPASSWORD[7] = 0;	
		factoryPASSWORD[8] = 0;
		factoryPASSWORD[9] = 0;	
}

/*=====================================================================
				*********** INCREASE BUTTON PRESSED ************			
=====================================================================*/
void ibprs( void )
{
	anyButtonPressed();
	//********************************
	//Producer Password Need Screen
	//********************************
	if(fproducerPasswordNeeded)
	{
		if(fproducerPassword_1_Adj)
		{	
			producerPassAdj(1,1);		
		}
		else if(fproducerPassword_2_Adj)
		{
			producerPassAdj(2,1);	
		}	
		else if(fproducerPassword_3_Adj)
		{
			producerPassAdj(3,1);	
		}		
		else if(fproducerPassword_4_Adj)
		{
			producerPassAdj(4,1);	
		}		
		else if(fproducerPassword_5_Adj)
		{
			producerPassAdj(5,1);	
		}		
		else if(fproducerPassword_6_Adj)
		{
			producerPassAdj(6,1);	
		}		
		else if(fproducerPassword_7_Adj)
		{
			producerPassAdj(7,1);	
		}		
		else if(fproducerPassword_8_Adj)
		{
			producerPassAdj(8,1);	
		}		
		else if(fproducerPassword_9_Adj)
		{
			producerPassAdj(9,1);	
		}		
		else if(fproducerPassword_10_Adj)
		{
			producerPassAdj(10,1);	
		}				
	}	
	//********************************
	//Date and watch Adjustment Screen
	//********************************
	else if(fwatchAdjustment)
	{
		if(fdayAdjust)
		{
			updateDay(1);
		}
		else if(fmonthAdjust)
		{
			updateMonth(1);
		}
		else if(fyearAdjust)
		{
			updateYear(1);
		}
		else if(fhourAdjust)
		{
			updateHour(1);
		}
		else if(fminuteAdjust)
		{
			updateMinute(1);
		}
		//
		//
		fcelenderPlanDelayed = 0;
		fUpdateRtcAlarm = 1;  // sistem resetlenirse sulama yapacak
		finitRtcAlarm = 1;  // sistem resetlenirse geçmis sulamalari dikkate alma
		//
		//
	}
//********************************
//Selenoid Programming Screen
//********************************	
	else if(fselenoidProgramming)
	{
		if(fselectedSelenoidAdjust)
		{
			selectedSelenoidUpdate(1);
		}
		else if(fSelenoidWorkDurAdjust)
		{
			selenoidWorkDurAdj(1);
		}
		else if(fSelStartTime_1_HourAdjust)
		{
			SelenoidStartTime_1_HourAdjust(1);
		}	
		else if(fSelStartTime_1_MinuteAdjust)
		{
			SelenoidStartTime_1_MinuteAdjust(1);
		}
		else if(fSelStartTime_2_HourAdjust)
		{
			SelenoidStartTime_2_HourAdjust(1);
		}	
		else if(fSelStartTime_2_MinuteAdjust)
		{
			SelenoidStartTime_2_MinuteAdjust(1);  // buradan sonra 3 ve 4 gelecek
		}
		else if(fSelStartTime_3_HourAdjust)
		{
			SelenoidStartTime_3_HourAdjust(1);
		}	
		else if(fSelStartTime_3_MinuteAdjust)
		{
			SelenoidStartTime_3_MinuteAdjust(1);
		}
		else if(fSelStartTime_4_HourAdjust)
		{
			SelenoidStartTime_4_HourAdjust(1);
		}	
		else if(fSelStartTime_4_MinuteAdjust)
		{
			SelenoidStartTime_4_MinuteAdjust(1);  // buradan sonra 3 ve 4 gelecek
		}		
		else if(fHergunSelected)
		{					
			fHergunSelected = 0;
			fSpecialSelected = 1;
		}	
		else if(fSpecialSelected)
		{
			if(fSpecialDaySelect_PT)
			{
				fSpecialDaySelect_PT = 0;
				fSpecialDaySelect_SA = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] | 0b00000001;  // PAZARTESI
				sel_delayed_irr_val[selectedSelenoid-1] = 0; //gun ayari yapti, aralikli sulama varya sifirla	
			}
			else if(fSpecialDaySelect_SA)
			{
				fSpecialDaySelect_SA = 0;
				fSpecialDaySelect_CA = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] | 0b00000010;  // SALI
				sel_delayed_irr_val[selectedSelenoid-1] = 0; //gun ayari yapti, aralikli sulama varya sifirla	
			}			
			else if(fSpecialDaySelect_CA)
			{
				fSpecialDaySelect_CA = 0;
				fSpecialDaySelect_PE = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] | 0b00000100;  // CARSAMBA
				sel_delayed_irr_val[selectedSelenoid-1] = 0; //gun ayari yapti, aralikli sulama varya sifirla	
			}			
			else if(fSpecialDaySelect_PE)
			{
				fSpecialDaySelect_PE = 0;
				fSpecialDaySelect_CU = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] | 0b00001000;  // PERSEMBE
				sel_delayed_irr_val[selectedSelenoid-1] = 0; //gun ayari yapti, aralikli sulama varya sifirla	
			}			
			else if(fSpecialDaySelect_CU)
			{
				fSpecialDaySelect_CU = 0;
				fSpecialDaySelect_CT = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] | 0b00010000;  // CUMA
				sel_delayed_irr_val[selectedSelenoid-1] = 0; //gun ayari yapti, aralikli sulama varya sifirla	
			}			
			else if(fSpecialDaySelect_CT)
			{
				fSpecialDaySelect_CT = 0;
				fSpecialDaySelect_PZ = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] | 0b00100000;  // CUMARTESI		
				sel_delayed_irr_val[selectedSelenoid-1] = 0; //gun ayari yapti, aralikli sulama varya sifirla				
			}			
			else if(fSpecialDaySelect_PZ)
			{
				fSpecialDaySelect_PZ = 0;
				fSpecialSelected = 0;
				fselectedSelenoidAdjust = 1;  
				fselenoidProgramming = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] | 0b01000000;  // PAZAR	
				sel_delayed_irr_val[selectedSelenoid-1] = 0; //gun ayari yapti, aralikli sulama varya sifirla					
				selectedSelenoidUpdate(1);				
			}	
			else
			{
				fSpecialSelected = 0;
				fDelayedIrrigationSelected = 1;
			}
		}				
		else if(fDelayedIrrigationSelected)
		{
			if(fDelayValueAdjust)
			{
				DelayValueAdjust(1);
				sel_special_irr_days[selectedSelenoid-1] = 0b00000000;  // aralikli sulama ayarlanmaya basladi, özel günleri sifirla
			}
			else
			{
				fDelayedIrrigationSelected = 0;
				fHergunSelected = 1;
			}
		}
		else if(fmanuelIrrigationAdjust)
		{
			fManuelRelayAdjust = 1;		
		}
			///// ayar yaparken tus basimi oldukça aktif prograni yenile
		fcelenderPlanDelayed = 0;
		fUpdateRtcAlarm = 1;  // sistem resetlenirse sulama yapacak
		finitRtcAlarm = 1;  // sistem resetlenirse geçmis sulamalari dikkate alma
			////		
	}
//************************************
//Manuel Irrigation Programming Screen
//************************************
	else if(fmanuelIrrigationAdjust)
	{
		if(fActivateManuelMode)
		{
			if((!fStartManuelWork)&&(!fStartAllRelayManuelWork))
			{
				fActivateManuelMode = 0;	
				fManuelRelayAdjust = 1;			
				fAll_Her_Selected = 1;
			}
		}
		else if(fManuelRelayAdjust)
		{
			if(fAll_Her_Selected == 1)
			{
				fAll_Her_Selected = 0;
			}
			else
			{
				if(selectedSelenoid == 4)
				{
					fAll_Her_Selected = 1;
				}
				selectedSelenoidUpdate(1);
			}
		}
		else if(fManuelAllRelayWorkDurAdj)
		{
			ManuelALLWorkDurationAdjust(1);
		}
		else if(fManuelWorkDurationAdjust)
		{
			ManuelWorkDurationAdjust(1);		
		}	
	}	
//************************************
//Password Setting Screen
//************************************
	else if(fPasswordAdjust)
	{
		if(fPasswordDigit1Adjust)
		{
			passwordAdjust(1,1); // Digit 1 +
		}
		else if(fPasswordDigit2Adjust)
		{
			passwordAdjust(2,1); //Digit 2 +
		}
		else if(fPasswordDigit3Adjust)
		{
			passwordAdjust(3,1); //Digit 3 +
		}	
		else if(fPasswordDigit4Adjust)
		{
			passwordAdjust(4,1); //Digit 4 +
		}		
	}	
//************************************
//HomePage with Password Screen
//************************************
	else if(fHomePageWithPassword)
	{
		if(fPasswordDigit1Adjust)
		{
			passwordAdjust(1,1); // Digit 1 +
		}
		else if(fPasswordDigit2Adjust)
		{
			passwordAdjust(2,1); //Digit 2 +
		}
		else if(fPasswordDigit3Adjust)
		{
			passwordAdjust(3,1); //Digit 3 +
		}	
		else if(fPasswordDigit4Adjust)
		{
			passwordAdjust(4,1); //Digit 4 +
		}		
	}		
}

/*=====================================================================
				*********** DECREASE BUTTON PRESSED ************			
=====================================================================*/
void dbprs( void )
{
	anyButtonPressed();
	//********************************
	//Producer Password Need Screen
	//********************************
	if(fproducerPasswordNeeded)
	{
		if(fproducerPassword_1_Adj)
		{	
			producerPassAdj(1,0);		
		}
		else if(fproducerPassword_2_Adj)
		{
			producerPassAdj(2,0);	
		}	
		else if(fproducerPassword_3_Adj)
		{
			producerPassAdj(3,0);	
		}		
		else if(fproducerPassword_4_Adj)
		{
			producerPassAdj(4,0);	
		}		
		else if(fproducerPassword_5_Adj)
		{
			producerPassAdj(5,0);	
		}		
		else if(fproducerPassword_6_Adj)
		{
			producerPassAdj(6,0);	
		}		
		else if(fproducerPassword_7_Adj)
		{
			producerPassAdj(7,0);	
		}		
		else if(fproducerPassword_8_Adj)
		{
			producerPassAdj(8,0);	
		}		
		else if(fproducerPassword_9_Adj)
		{
			producerPassAdj(9,0);	
		}		
		else if(fproducerPassword_10_Adj)
		{
			producerPassAdj(10,0);	
		}				
	}
		//********************************
	//Date and watch Adjustment Screen
	//********************************
	else if(fwatchAdjustment)
	{
		if(fdayAdjust)
		{
			updateDay(0);
		}
		else if(fmonthAdjust)
		{
			updateMonth(0);
		}
		else if(fyearAdjust)
		{
			updateYear(0);
		}
		else if(fhourAdjust)
		{
			updateHour(0);
		}
		else if(fminuteAdjust)
		{
			updateMinute(0);
		}	
		//
		//
		fcelenderPlanDelayed = 0;
		fUpdateRtcAlarm = 1;  // sistem resetlenirse sulama yapacak
		finitRtcAlarm = 1;  // sistem resetlenirse geçmis sulamalari dikkate alma
		//
		//
	}
	else if(fselenoidProgramming)
	{
		if(fselectedSelenoidAdjust)
		{
			selectedSelenoidUpdate(0);
		}
		else if(fSelenoidWorkDurAdjust)
		{
			selenoidWorkDurAdj(0);
		}
		else if(fSelStartTime_1_HourAdjust)
		{
			SelenoidStartTime_1_HourAdjust(0);
		}	
		else if(fSelStartTime_1_MinuteAdjust)
		{
			SelenoidStartTime_1_MinuteAdjust(0);
		}
		else if(fSelStartTime_2_HourAdjust)
		{
			SelenoidStartTime_2_HourAdjust(0);
		}	
		else if(fSelStartTime_2_MinuteAdjust)
		{
			SelenoidStartTime_2_MinuteAdjust(0);  // buradan sonra 3 ve 4 gelecek
		}	
		else if(fSelStartTime_3_HourAdjust)
		{
			SelenoidStartTime_3_HourAdjust(0);
		}	
		else if(fSelStartTime_3_MinuteAdjust)
		{
			SelenoidStartTime_3_MinuteAdjust(0);
		}
		else if(fSelStartTime_4_HourAdjust)
		{
			SelenoidStartTime_4_HourAdjust(0);
		}	
		else if(fSelStartTime_4_MinuteAdjust)
		{
			SelenoidStartTime_4_MinuteAdjust(0);  // buradan sonra 3 ve 4 gelecek
		}			
		else if(fHergunSelected)
		{					
			fHergunSelected = 0;
			fSpecialSelected = 1;
		}	
		else if(fSpecialSelected)
		{
			if(fSpecialDaySelect_PT)
			{
				fSpecialDaySelect_PT = 0;
				fSpecialDaySelect_SA = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] & ~0b00000001;  // PAZARTESI
			}
			else if(fSpecialDaySelect_SA)
			{
				fSpecialDaySelect_SA = 0;
				fSpecialDaySelect_CA = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] & ~0b00000010;  // SALI
			}			
			else if(fSpecialDaySelect_CA)
			{
				fSpecialDaySelect_CA = 0;
				fSpecialDaySelect_PE = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] & ~0b00000100;  // CARSAMBA
			}			
			else if(fSpecialDaySelect_PE)
			{
				fSpecialDaySelect_PE = 0;
				fSpecialDaySelect_CU = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] & ~0b00001000;  // PERSEMBE
			}			
			else if(fSpecialDaySelect_CU)
			{
				fSpecialDaySelect_CU = 0;
				fSpecialDaySelect_CT = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] & ~0b00010000;  // CUMA
			}			
			else if(fSpecialDaySelect_CT)
			{
				fSpecialDaySelect_CT = 0;
				fSpecialDaySelect_PZ = 1;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] & ~0b00100000;  // CUMARTESI				
			}			
			else if(fSpecialDaySelect_PZ)
			{
				fSpecialDaySelect_PZ = 0;
				fselectedSelenoidAdjust = 1;  
				fselenoidProgramming = 1;
				fSpecialSelected = 0;
				sel_special_irr_days[selectedSelenoid-1] = sel_special_irr_days[selectedSelenoid-1] & ~0b01000000;  // PAZAR
				selectedSelenoidUpdate(1);				
			}	
			else
			{
				fSpecialSelected = 0;
				fDelayedIrrigationSelected = 1;
			}
		}				
		else if(fDelayedIrrigationSelected)
		{
			if(fDelayValueAdjust)
			{
				DelayValueAdjust(0);
				sel_special_irr_days[selectedSelenoid-1] = 0b00000000;  // aralikli sulama ayarlanmaya basladi, özel günleri sifirla  !! ilk tus basiminda 29 güne kurulum yapiliyor, o yüzden burada da sifirladik
			}
			else
			{
				fDelayedIrrigationSelected = 0;
				fHergunSelected = 1;
			}
		}
		///// ayar yaparken tus basimi oldukça aktif prograni yenile
		fcelenderPlanDelayed = 0;
		fUpdateRtcAlarm = 1;  // sistem resetlenirse sulama yapacak
		finitRtcAlarm = 1;  // sistem resetlenirse geçmis sulamalari dikkate alma
		////		
	}	
//************************************
//Manuel Irrigation Programming Screen
//************************************
//	else if(fmanuelIrrigationAdjust)
//	{
//		if(fActivateManuelMode)
//		{
//			fActivateManuelMode = 0;
//			fManuelRelayAdjust = 1;
//		}
//		else if(fManuelRelayAdjust)
//		{
//			selectedSelenoidUpdate(0);
//		}
//		else if(fManuelWorkDurationAdjust)
//		{
//			ManuelWorkDurationAdjust(0);		
//		}	
//	}	
	
	else if(fmanuelIrrigationAdjust)
	{
		if(fManuelRelayAdjust)
		{
			if(fAll_Her_Selected == 1)
			{
				fAll_Her_Selected = 0;
			}
			else
			{
				if(selectedSelenoid == 1)
				{
					fAll_Her_Selected = 1;
				}
				selectedSelenoidUpdate(0);
			}
		}
		else if(fManuelAllRelayWorkDurAdj)
		{
			ManuelALLWorkDurationAdjust(0);
		}
		else if(fManuelWorkDurationAdjust)
		{
			ManuelWorkDurationAdjust(0);		
		}	
	}		
	
	
//************************************
//Password Setting Screen
//************************************
	else if(fPasswordAdjust)
	{
		if(fPasswordDigit1Adjust)
		{
			passwordAdjust(1,0); // Digit 1 -
		}
		else if(fPasswordDigit2Adjust)
		{
			passwordAdjust(2,0); //Digit 2 -
		}
		else if(fPasswordDigit3Adjust)
		{
			passwordAdjust(3,0); //Digit 3 -
		}	
		else if(fPasswordDigit4Adjust)
		{
			passwordAdjust(4,0); //Digit 4 -
		}		
	}
//************************************
//HomePage with Password Screen
//************************************
	else if(fHomePageWithPassword)
	{
		if(fPasswordDigit1Adjust)
		{
			passwordAdjust(1,0); // Digit 1 -
		}
		else if(fPasswordDigit2Adjust)
		{
			passwordAdjust(2,0); //Digit 2 -
		}
		else if(fPasswordDigit3Adjust)
		{
			passwordAdjust(3,0); //Digit 3 -
		}	
		else if(fPasswordDigit4Adjust)
		{
			passwordAdjust(4,0); //Digit 4 -
		}		
	}			
}


/*=====================================================================
				*********** MENU BUTTON PRESSED ************			
=====================================================================*/
void mbprs( void )
{
	anyButtonPressed();
	clearAllDetailAdjustment();
	tmpPassword[0] = 0;
	tmpPassword[1] = 0;
	tmpPassword[2] = 0;   // sifreler gösterilmesin
	tmpPassword[3] = 0;
	selectedSelenoid = 1;
	if(fhomePage)
	{
		fhomePage = 0;
		fwatchAdjustment = 1;
	}
	else if(fwatchAdjustment)
	{
		if(fDeviceLockedWithPassword)
		{
			fwatchAdjustment = 0;
			fHomePageWithPassword = 1;
			fPasswordDigit1Adjust = 1;
		}
		else
		{
			fwatchAdjustment = 0;
			fselenoidProgramming = 1;
			fselectedSelenoidAdjust = 1;
		}
	}
	else if(fselenoidProgramming)
	{
		fselectedSelenoidAdjust = 0;
		fselenoidProgramming = 0;
		fmanuelIrrigationAdjust = 1;
		fActivateManuelMode = 1;
	}
	else if(fmanuelIrrigationAdjust)
	{
		fmanuelIrrigationAdjust = 0;
		fActivateManuelMode = 0;
//		fmanuelIrrigationPlanAdjust = 1;     // manuel plan iptal edildi yazilim yoktu zaten 15.01.2019
//		fActivateManuelPlanMode = 1;
		fPasswordAdjust = 1;
		fPasswordActivateMode = 1;
	}	
//	else if(fmanuelIrrigationPlanAdjust)
//	{
//		fmanuelIrrigationPlanAdjust = 0;
//		fActivateManuelPlanMode = 0;
//		fPasswordAdjust = 1;
//		fPasswordActivateMode = 1;
//	}	
	else if(fPasswordAdjust)
	{
		fPasswordAdjust = 0;
		fPasswordActivateMode = 0;
		fhomePage = 1;
	}
	else
	{
		fhomePage = 1;  // uykundan uyandiginda Ram silinirse diye güvenlik amaçli. gerek yok aslinda???
	}		
}
/*=====================================================================
				*********** FORWARD BUTTON PRESSED ************			
=====================================================================*/
void forwardbprs( void )
{
	anyButtonPressed();
	//********************************
	//Producer Password Need Screen
	//********************************
	if(fproducerPasswordNeeded)
	{
		if(fproducerPassword_1_Adj)
		{
			fproducerPassword_1_Adj = 0;
			fproducerPassword_2_Adj = 1;			
		}
		else if(fproducerPassword_2_Adj)
		{
			fproducerPassword_2_Adj = 0;
			fproducerPassword_3_Adj = 1;			
		}
		else if(fproducerPassword_3_Adj)
		{
			fproducerPassword_3_Adj = 0;
			fproducerPassword_4_Adj = 1;			
		}
		else if(fproducerPassword_4_Adj)
		{
			fproducerPassword_4_Adj = 0;
			fproducerPassword_5_Adj = 1;			
		}
		else if(fproducerPassword_5_Adj)
		{
			fproducerPassword_5_Adj = 0;
			fproducerPassword_6_Adj = 1;			
		}
		else if(fproducerPassword_6_Adj)
		{
			fproducerPassword_6_Adj = 0;
			fproducerPassword_7_Adj = 1;			
		}
		else if(fproducerPassword_7_Adj)
		{
			fproducerPassword_7_Adj = 0;
			fproducerPassword_8_Adj = 1;			
		}
		else if(fproducerPassword_8_Adj)
		{
			fproducerPassword_8_Adj = 0;
			fproducerPassword_9_Adj = 1;			
		}
		else if(fproducerPassword_9_Adj)
		{
			fproducerPassword_9_Adj = 0;
			fproducerPassword_10_Adj = 1;			
		}
		else if(fproducerPassword_10_Adj)
		{
			fproducerPassword_10_Adj = 0;
			fproducerPassword_1_Adj = 1;			
		}		
		else
		{
			fproducerPassword_1_Adj = 1;
			fproducerPassword_2_Adj = 0;
			fproducerPassword_3_Adj = 0;
			fproducerPassword_4_Adj = 0;
			fproducerPassword_5_Adj = 0;
			fproducerPassword_6_Adj = 0;
			fproducerPassword_7_Adj = 0;
			fproducerPassword_8_Adj = 0;
			fproducerPassword_9_Adj = 0;
			fproducerPassword_10_Adj = 0;
		}
	}
	
//********************************
//Date and watch Adjustment Screen
//********************************
	else if(fwatchAdjustment)
	{
		if(fyearAdjust)
		{
			fyearAdjust = 0;
			fmonthAdjust = 1;			
		}
		else if(fmonthAdjust)
		{
			fmonthAdjust = 0;
			fdayAdjust = 1;			
		}
		else if(fdayAdjust)
		{
			fdayAdjust = 0;
			fhourAdjust = 1;			
		}
		else if(fhourAdjust)
		{
			fhourAdjust = 0;
			fminuteAdjust = 1;			
		}
		else if(fminuteAdjust)
		{
			fminuteAdjust = 0;
			clearAllDetailAdjustment();
			fwatchAdjustment = 0; 
			if(fDeviceLockedWithPassword)
			{
				fHomePageWithPassword = 1;
				fPasswordDigit1Adjust = 1;
			}
			else
			{
				fselenoidProgramming = 1;    // SAAT TARIH AYARLANDIKTAN SONRA SELENOID PROGRAMLAMAYA GEÇILSIN 
				fselectedSelenoidAdjust = 1; // selenoid seçim modu hemen baslasin
			}	
		}		
		else
		{
			fyearAdjust = 1;
			fdayOfWeekAdjust = 0;
			fminuteAdjust = 0;
			fhourAdjust = 0;	
			fmonthAdjust = 0;			
			fdayAdjust = 0;	
		}
	}
//********************************
//Selenoid Programming Screen
//********************************	
	else if(fselenoidProgramming)
	{
		if(fselectedSelenoidAdjust)
		{
			fselectedSelenoidAdjust = 0;
			fSelenoidWorkDurAdjust = 1;
		}
		else if(fSelenoidWorkDurAdjust)
		{
			fSelStartTime_1_HourAdjust = 1;
			fSelenoidWorkDurAdjust = 0;
		}
		else if(fSelStartTime_1_HourAdjust)
		{
			fSelStartTime_1_MinuteAdjust = 1;
			fSelStartTime_1_HourAdjust = 0;
		}
		else if(fSelStartTime_1_MinuteAdjust)
		{
			fSelStartTime_2_HourAdjust = 1;
			fSelStartTime_1_MinuteAdjust = 0;
		}
		else if(fSelStartTime_2_HourAdjust)
		{
			fSelStartTime_2_MinuteAdjust = 1;      // UZUN BASIM OLURSA BASKA MENUYE GIDECEK
			fSelStartTime_2_HourAdjust = 0;
		}
		else if(fSelStartTime_2_MinuteAdjust)
		{
			fSelStartTime_3_HourAdjust = 1;
			fSelStartTime_2_MinuteAdjust = 0;
		}
		else if(fSelStartTime_3_HourAdjust)
		{
			fSelStartTime_3_MinuteAdjust = 1;
			fSelStartTime_3_HourAdjust = 0;
		}
		else if(fSelStartTime_3_MinuteAdjust)
		{
			fSelStartTime_4_HourAdjust = 1;
			fSelStartTime_3_MinuteAdjust = 0;
		}
		else if(fSelStartTime_4_HourAdjust)
		{
			fSelStartTime_4_MinuteAdjust = 1;      
			fSelStartTime_4_HourAdjust = 0;
		}
		else if(fSelStartTime_4_MinuteAdjust)
		{
			    
			fSelStartTime_4_MinuteAdjust = 0;
			if(sel_delayed_irr_val[selectedSelenoid-1])
			{
				fDelayedIrrigationSelected = 1;
				fSpecialSelected = 0;
				fHergunSelected = 0; 
			}
			else if(sel_special_irr_days[selectedSelenoid-1])
			{
				fDelayedIrrigationSelected = 0;
				fSpecialSelected = 1;
				fHergunSelected = 0; 
			}
			else
			{
				fDelayedIrrigationSelected = 0;
				fSpecialSelected = 0;
				fHergunSelected = 1;  
			}
		}		
		else if(fHergunSelected)
		{
			if(selectedSelenoid == 4)
			{
				fHergunSelected = 0;
				fselectedSelenoidAdjust = 0;
				fselenoidProgramming = 0;
				sel_delayed_irr_val[selectedSelenoid-1] = 0; //hergün seçildiginde daha önce seçilen aralikli sulamayi sifirla		
				sel_special_irr_days[selectedSelenoid-1] = 0b00000000;  //hergün seçildiginde daha önce seçilen özel günleri sifirla	
				fhomePage = 1;
			}
			else
			{
				fselenoidProgramming = 1;  
				fselectedSelenoidAdjust = 1;      // Tekrar Vana Ayarlamaya gitsin 
				sel_delayed_irr_val[selectedSelenoid-1] = 0; //hergün seçildiginde daha önce seçilen aralikli sulamayi sifirla		
				sel_special_irr_days[selectedSelenoid-1] = 0b00000000;  //hergün seçildiginde daha önce seçilen özel günleri sifirla	
				selectedSelenoidUpdate(1);   // Burada aktif olan Vana numarasi arttirilarak geri döndürülebilir
				fHergunSelected = 0;
			}
		}		
		else if(fSpecialSelected)
		{
			if((fSpecialDaySelect_PT)|(fSpecialDaySelect_SA)|(fSpecialDaySelect_CA)|(fSpecialDaySelect_PE)|(fSpecialDaySelect_CU)|(fSpecialDaySelect_CT)|(fSpecialDaySelect_PZ))
			{
				fDelayedIrrigationSelected = 0;
				fDelayValueAdjust = 0;
				fSpecialDaySelect_PT = 0;	
				fSpecialDaySelect_SA = 0;	
				fSpecialDaySelect_CA = 0;	
				fSpecialDaySelect_PE = 0;	
				fSpecialDaySelect_CU = 0;	
				fSpecialDaySelect_CT = 0;	
				fSpecialDaySelect_PZ = 0;		
				fselectedSelenoidAdjust = 1;  
				fselenoidProgramming = 1;	
				selectedSelenoidUpdate(1);   // Burada aktif olan Vana numarasi arttirilarak geri döndürülebilir
				fSpecialSelected = 0;
			}
			else
			{
				fselenoidProgramming = 1;  
				fSpecialDaySelect_PT = 1;	
			}
		}
		else if(fDelayedIrrigationSelected)
		{
			if(fDelayValueAdjust)
			{
				fDelayedIrrigationSelected = 0;
				fDelayValueAdjust = 0;
				fselectedSelenoidAdjust = 1;  
				fselenoidProgramming = 1;
				selectedSelenoidUpdate(1);   // Burada aktif olan Vana numarasi arttirilarak geri döndürülebilir
			}
			else
			{
				fDelayValueAdjust = 1;
			}
		}		
	}
//************************************
//Manuel Irrigation Programming Screen
//************************************
	else if(fmanuelIrrigationAdjust)
	{
		if(fManuelRelayAdjust)
		{
			fManuelRelayAdjust = 0;
			if(fAll_Her_Selected)
			{
				fManuelWorkDurationAdjust = 0;
				fManuelAllRelayWorkDurAdj = 1;
			}
			else
			{
				fManuelWorkDurationAdjust = 1;
				fManuelAllRelayWorkDurAdj = 0;
			}
		}
		else if(fManuelAllRelayWorkDurAdj)
		{
			if(all_manuel_work_duration != 0) // 0 dakikaya kurulmasin, ana sayfaya dönsün
			{
				fManuelAllRelayWorkDurAdj = 0;
				fStartAllRelayManuelWork = 1;
				startManuelWorkDelay = 3000; // milisaniye gecikme ekle, kurulu olan alarm varsa önce onunla arasindaki farki gösteriyor, düzelmek için delay eklenebilir
				fUpdateRtcAlarm = 1;
			}
			else
			{
				fmanuelIrrigationAdjust = 0;
				fManuelAllRelayWorkDurAdj = 0;
				fhomePage = 1;
				selectedSelenoid = 1;
			}
		}
		else if(fManuelWorkDurationAdjust)
		{			
			if(sel_manuel_work_duration[selectedSelenoid-1] != 0) // 0 dakikaya kurulmasin, ana sayfaya dönsün
			{
				fManuelWorkDurationAdjust = 0;
				fStartManuelWork = 1;
				startManuelWorkDelay = 3000; // milisaniye gecikme ekle, kurulu olan alarm varsa önce onunla arasindaki farki gösteriyor, düzelmek için delay eklenebilir
				fUpdateRtcAlarm = 1;
			}
			else
			{
				fmanuelIrrigationAdjust = 0;
				fManuelWorkDurationAdjust = 0;
				fhomePage = 1;
				selectedSelenoid = 1;
			}
		}
	}
//************************************
//Manuel Irrigation Programming Screen
//************************************
//	else if(fmanuelIrrigationPlanAdjust)
//	{
//		fActivateManuelPlanMode = 0;
//	}
//************************************
//Password Adjusting Screen
//************************************
	else if(fPasswordAdjust)
	{
		if(fPasswordDigit1Adjust)
		{
			fPasswordDigit1Adjust = 0;
			fPasswordDigit2Adjust = 1;
		}
		else if(fPasswordDigit2Adjust)
		{
			fPasswordDigit2Adjust = 0;
			fPasswordDigit3Adjust = 1;			
		}
		else if(fPasswordDigit3Adjust)
		{
			fPasswordDigit3Adjust = 0;
			fPasswordDigit4Adjust = 1;			
		}
		else if(fPasswordDigit4Adjust)
		{
			fPasswordDigit4Adjust = 0;
			fPasswordDigit1Adjust = 1;			
		}
		else
		{	
			fPasswordAdjust = 1;
			fPasswordActivateMode = 0;	
			fsetNewPasswordMode = 1;
			fPasswordDigit1Adjust = 1;				
		}
	}
//************************************
//HomePage with Password Screen
//************************************
	else if(fHomePageWithPassword)
	{
		if(fPasswordDigit1Adjust)
		{
			fPasswordDigit1Adjust = 0;
			fPasswordDigit2Adjust = 1;
		}
		else if(fPasswordDigit2Adjust)
		{
			fPasswordDigit2Adjust = 0;
			fPasswordDigit3Adjust = 1;			
		}
		else if(fPasswordDigit3Adjust)
		{
			fPasswordDigit3Adjust = 0;
			fPasswordDigit4Adjust = 1;			
		}
		else if(fPasswordDigit4Adjust)
		{
			fPasswordDigit4Adjust = 0;
			if((tmpPassword[0] == 0) & (tmpPassword[1] == 0) & (tmpPassword[2] == 0) & (tmpPassword[3] == 0)) 
			{
				fPasswordDigit1Adjust = 1;
			}
			else
			{
				checkPasword();	
			}				
		}
	}	
}
/*=====================================================================
				*********** BACK BUTTON PRESSED ************			
=====================================================================*/
void backbprs( void )
{
	
	anyButtonPressed();
	
	//********************************
	//Producer Password Need Screen
	//********************************
	if(fproducerPasswordNeeded)
	{
		if(fproducerPassword_1_Adj)
		{
			fproducerPassword_1_Adj = 0;
			fproducerPassword_10_Adj = 1;			
		}
		else if(fproducerPassword_2_Adj)
		{
			fproducerPassword_2_Adj = 0;
			fproducerPassword_1_Adj = 1;			
		}
		else if(fproducerPassword_3_Adj)
		{
			fproducerPassword_3_Adj = 0;
			fproducerPassword_2_Adj = 1;			
		}
		else if(fproducerPassword_4_Adj)
		{
			fproducerPassword_4_Adj = 0;
			fproducerPassword_3_Adj = 1;			
		}
		else if(fproducerPassword_5_Adj)
		{
			fproducerPassword_5_Adj = 0;
			fproducerPassword_4_Adj = 1;			
		}
		else if(fproducerPassword_6_Adj)
		{
			fproducerPassword_6_Adj = 0;
			fproducerPassword_5_Adj = 1;			
		}
		else if(fproducerPassword_7_Adj)
		{
			fproducerPassword_7_Adj = 0;
			fproducerPassword_6_Adj = 1;			
		}
		else if(fproducerPassword_8_Adj)
		{
			fproducerPassword_8_Adj = 0;
			fproducerPassword_7_Adj = 1;			
		}
		else if(fproducerPassword_9_Adj)
		{
			fproducerPassword_9_Adj = 0;
			fproducerPassword_8_Adj = 1;			
		}
		else if(fproducerPassword_10_Adj)
		{
			fproducerPassword_10_Adj = 0;
			fproducerPassword_9_Adj = 1;			
		}		
		else
		{
			fproducerPassword_1_Adj = 1;
			fproducerPassword_2_Adj = 0;
			fproducerPassword_3_Adj = 0;
			fproducerPassword_4_Adj = 0;
			fproducerPassword_5_Adj = 0;
			fproducerPassword_6_Adj = 0;
			fproducerPassword_7_Adj = 0;
			fproducerPassword_8_Adj = 0;
			fproducerPassword_9_Adj = 0;
			fproducerPassword_10_Adj = 0;
		}
	}	
	
	
//********************************
//Date and watch Adjustment Screen
//********************************
	else if(fwatchAdjustment)
	{
		if(fminuteAdjust)
		{
			fhourAdjust = 1;
			fminuteAdjust = 0;			
		}
	}	
	
//********************************
//Selenoid Programming Screen
//********************************	
	else if(fselenoidProgramming)
	{
		if(fSelenoidWorkDurAdjust)
		{
			fselectedSelenoidAdjust = 1;
			fSelenoidWorkDurAdjust = 0;
		}
		else if(fSelStartTime_1_HourAdjust)
		{
			fSelenoidWorkDurAdjust =1;
			fSelStartTime_1_HourAdjust = 0;
		}
		else if(fSelStartTime_1_MinuteAdjust)
		{
			fSelStartTime_1_HourAdjust = 1;
			fSelStartTime_1_MinuteAdjust = 0;
		}
		else if(fSelStartTime_2_HourAdjust)
		{
			fSelStartTime_1_MinuteAdjust = 1;      // UZUN BASIM OLURSA BASKA MENUYE GIDECEK
			fSelStartTime_2_HourAdjust = 0;
		}
		else if(fSelStartTime_2_MinuteAdjust)
		{
			fSelStartTime_2_HourAdjust = 1;
			fSelStartTime_2_MinuteAdjust = 0;
		}
		else if(fSelStartTime_3_HourAdjust)
		{
			fSelStartTime_2_MinuteAdjust = 1;
			fSelStartTime_3_HourAdjust = 0;
		}
		else if(fSelStartTime_3_MinuteAdjust)
		{
			fSelStartTime_3_HourAdjust = 1;
			fSelStartTime_3_MinuteAdjust = 0;
		}
		else if(fSelStartTime_4_HourAdjust)
		{
			fSelStartTime_3_MinuteAdjust = 1;      // UZUN BASIM OLURSA BASKA MENUYE GIDECEK
			fSelStartTime_4_HourAdjust = 0;
		}
		else if(fSelStartTime_4_MinuteAdjust)
		{
			fSelStartTime_4_HourAdjust = 1;
			fSelStartTime_4_MinuteAdjust = 0;
		}		
	}	
	
//************************************
//Manuel Irrigation Programming Screen
//************************************
	else if(fmanuelIrrigationAdjust)
	{
		fcelenderPlanDelayed = 0;
		fUpdateRtcAlarm = 1; // manuel sulama iptal edildiginde tekrar planli sulamalari yap
		finitRtcAlarm = 1; // planlama yaptiktan sonra zamani geçmis sulamalari iptal et
		fManuelRelayAdjust = 0;
		fActivateManuelMode = 0;
		fManuelWorkDurationAdjust = 0;
		fManuelAllRelayWorkDurAdj = 0;
		fStartManuelWork = 0;	
		fStartAllRelayManuelWork = 0;
		startManuelWorkDelay = 0;
		if(fRelay1Status)
		{
			fCloseRelay1 = 1;
		}
		if(fRelay2Status)
		{
			fCloseRelay2 = 1; //  açik bir röle varsa kapat
		}
		if(fRelay3Status)
		{
			fCloseRelay3 = 1;
		}
		if(fRelay4Status)
		{
			fCloseRelay4 = 1;
		}	
		if(fDeviceLockedWithPassword)
		{
			fHomePageWithPassword = 1;
			fPasswordDigit1Adjust = 1;
		}
		else
		{
			fhomePage = 1;
		}
	}
//************************************
//Manuel Irrigation Plan Programming Screen
//************************************
//	else if(fmanuelIrrigationPlanAdjust)
//	{
//		fmanuelIrrigationPlanAdjust = 0;
//		fActivateManuelPlanMode = 0;
//		fStartManuelWork = 0;
//		startManuelWorkDelay = 0;
//		LCDBufferReset();
//		fCloseRelay1 = 1;
//		fCloseRelay2 = 1;
//		fCloseRelay3 = 1;
//		fCloseRelay4 = 1;
//		if(fDeviceLockedWithPassword)
//		{
//			fHomePageWithPassword = 1;
//			fPasswordDigit1Adjust = 1;
//		}
//		else
//		{
//			fhomePage = 1;
//		}
//	}	
//************************************
//Password Adjusting Screen
//************************************
	else if(fPasswordAdjust)
	{
		if(fPasswordDigit1Adjust)
		{
			fPasswordDigit1Adjust = 0;
			fPasswordDigit4Adjust = 1;
		}
		else if(fPasswordDigit2Adjust)
		{
			fPasswordDigit2Adjust = 0;
			fPasswordDigit1Adjust = 1;			
		}
		else if(fPasswordDigit3Adjust)
		{
			fPasswordDigit3Adjust = 0;
			fPasswordDigit2Adjust = 1;			
		}
		else if(fPasswordDigit4Adjust)
		{
			fPasswordDigit4Adjust = 0;
			fPasswordDigit3Adjust = 1;			
		}
	}
//************************************
//HomePage with Password Screen
//************************************
	else if(fHomePageWithPassword)
	{
		if(fPasswordDigit1Adjust)
		{
			fPasswordDigit1Adjust = 0;
			fPasswordDigit4Adjust = 1;
		}
		else if(fPasswordDigit2Adjust)
		{
			fPasswordDigit2Adjust = 0;
			fPasswordDigit1Adjust = 1;			
		}
		else if(fPasswordDigit3Adjust)
		{
			fPasswordDigit3Adjust = 0;
			fPasswordDigit2Adjust = 1;			
		}
		else if(fPasswordDigit4Adjust)
		{
			fPasswordDigit4Adjust = 0;
			fPasswordDigit3Adjust = 1;
		}
	}			
}


/*=====================================================================
				*********** CLEAR ALL ADJUSTMENT MODES ************			
=====================================================================*/
void clearAllDetailAdjustment(void)
{
	
	fminuteAdjust = 0;	
	fhourAdjust = 0;	
	fdayAdjust = 0;  
	fmonthAdjust = 0;			
	fyearAdjust	= 0;	

	fManuelRelayAdjust = 0;
	fActivateManuelMode = 0;
	fManuelWorkDurationAdjust = 0;
	fManuelAllRelayWorkDurAdj = 0;
	
	fPasswordActivateMode = 0;	
	fsetNewPasswordMode = 0;
	
	fPasswordDigit1Adjust = 0;	// ????
	fPasswordDigit2Adjust = 0;
	fPasswordDigit3Adjust = 0;	
	fPasswordDigit4Adjust = 0;
	
	fSelenoidWorkDurAdjust = 0;
	fSelenoidStartTimeAdjust = 0;
	fHergunSelected = 0;
	fDelayedIrrigationSelected = 0;
	fSpecialSelected = 0;
	fDelayValueAdjust = 0;	
	fSelStartTime_1_HourAdjust = 0;
	fSelStartTime_1_MinuteAdjust = 0;
	fSelStartTime_2_HourAdjust = 0;
	fSelStartTime_2_MinuteAdjust = 0;
	fSelStartTime_3_HourAdjust = 0;				//SONRA EKLE
	fSelStartTime_3_MinuteAdjust = 0;				//SONRA EKLE
	fSelStartTime_4_HourAdjust = 0;				//SONRA EKLE      
	fSelStartTime_4_MinuteAdjust = 0;				//SONRA EKLE	
}


/*=====================================================================
		********** FORWARD AND INCREASE BUTTON LONG PRESSED ***********			
=====================================================================*/

void IncAndFwdLPrs(void)
{
	anyButtonPressed();
	
	if(fselenoidProgramming & fselectedSelenoidAdjust)
	{
		fReadfromEEPROM = 1;
		fselectedSelenoidAdjust = 0;
		fCopySelenoidSettings = 0;
		fPlanSelenoidSettings = 1;
	}
}

/*=====================================================================
				*********** DOWN AND INCREASE BUTTON LONG PRESSED ************			
=====================================================================*/

void dbAndIncLPrs(void)
{
	anyButtonPressed();
	
	if(fselenoidProgramming & fselectedSelenoidAdjust & (selectedSelenoid == 1))
	{
		fRecordtoEEPROM = 1;
		fselectedSelenoidAdjust = 0;
		fCopySelenoidSettings = 0;
		fPlanSelenoidSettings = 1;
	}
}

/*=====================================================================
				*********** INCREASE AND BACK BUTTON LONG PRESSED ************			
=====================================================================*/

void incAndBckLPrs(void)
{
	anyButtonPressed();
	
	if(fselenoidProgramming & fselectedSelenoidAdjust & (selectedSelenoid == 2))
	{
		fcopyAndPasteSelSettings= 1;
		fRecordtoEEPROM = 1;
		fselectedSelenoidAdjust = 0;
		fCopySelenoidSettings = 1;
	}
}

/*=====================================================================
				*********** FORWARD AND BACK BUTTON PRESSED ************			
=====================================================================*/

void fwdAndBckPrs(void)
{
	
	anyButtonPressed();
	clearAllDetailAdjustment();
	LCDBufferReset();
	fPasswordAdjust = 0;
	fUpdatePassword = 1;
	if((tmpPassword[0] == 0) & (tmpPassword[1] == 0) & (tmpPassword[2] == 0) & (tmpPassword[3] == 0)) 
	{	
		fPasswordSetted = 0;
		fDeviceLockedWithPassword = 0;
		fhomePage = 1;
	}
	else
	{
		fPasswordSetted = 1;
		fDeviceLockedWithPassword = 1;
		fPasswordDigit1Adjust = 1;
		fHomePageWithPassword = 1;
	}
}

/*=====================================================================
				*********** BUTTON READ ************			
=====================================================================*/
void butread( void )
{
	butdat = (GPIOB-> IDR) & 0x001F;	
		
	if (butdat==prvdat)					//.0=sw1,.1=sw2,.2=sw3,.3=sw4,.4=sw5,.5=sw6
	{
		vdcnt++;
		if (button != butdat)			//Long button control
		{
			if ( vdcnt == 3)			//20ms * 3 = 60ms debounce
			{
				vdcnt = 0;
				prvbut = button;
				button = butdat;
				presdb = (butdat ^ prvbut) & prvbut;
				if (presdb != 0)
				{
					if ((presdb == 0x10)&(!fSystemOFF))			// - Button Pressed 	
					{
						dbprs();			
					}
					if ((presdb == 0x08)&(!fSystemOFF))				// + Button Pressed
					{
						ibprs();
					}
					if ((presdb == 0x04)&(!fSystemOFF))				// Menu Button Pressed
					{
						if((!fHomePageWithPassword)&(!fproducerPasswordNeeded))
						{
							mbprs();
						}							
					}
					if ((presdb == 0x02)&(!fSystemOFF))				// Forward Button Pressed			
					{
						forwardbprs();
					}
					if ((presdb == 0x01)&(!fSystemOFF))				// Back Button Pressed			
					{
						backbprs();
					}
					if ((presdb == 0x0A)&(!fSystemOFF)&(!fproducerPasswordNeeded))				// + And Forward Button Pressed 			
					{
						anyButtonPressed();
						fShowBatteryLevel = 1;
					}
					if ((presdb == 0x03)&(!fSystemOFF)&(!fproducerPasswordNeeded))				// Back And Forward Button Pressed 			
					{
						if(fhomePage)
						{
							fshowCurrentDayOfWeek = 1;
						}			
					}
					if ((presdb == 0x07)&(fproducerPasswordNeeded))				//Menu, Back And Forward Button Pressed 			
					{
						checkFactoryKey();		
					}
				}
			}
		}
		else
		{										//long button pressed
		if (vdcnt == 8)				//8*20=160ms
		{
			vdcnt=0;
			if (lvdcnt!= 255)
			{
				lvdcnt++;
				if (lvdcnt == 15)			// 15*160 = ~2.4 sec 
				{
					if ((presdb == 0x0A)&(!fSystemOFF)&(!fproducerPasswordNeeded))			// + And Forward Button Pressed 
					{
						IncAndFwdLPrs();
					}					
					if ((presdb == 0x18)&(!fSystemOFF)&(!fproducerPasswordNeeded))				// + And - Button Pressed 
					{
						dbAndIncLPrs();
					}
					if ((presdb == 0x09)&(!fSystemOFF)&(!fproducerPasswordNeeded))		// + And Back Button Pressed 
					{					
						incAndBckLPrs();
					}
					if ((presdb == 0x03)&(!fSystemOFF)&(!fproducerPasswordNeeded))				// + And Back Button Pressed 
					{	
						if(fPasswordAdjust)
						{
							fwdAndBckPrs();
						}														
					}
					if (presdb == 0x01)			// Back Button LONG Pressed 
					{	  
						if(fAnyRelayON)
						{
							if((fRelay1Status)&(!fStartManuelWork)&(!fStartAllRelayManuelWork))
							{
								fCloseRelay1 = 1;
							}
							if((fRelay2Status)&(!fStartManuelWork)&(!fStartAllRelayManuelWork))
							{
								fCloseRelay2 = 1;
							}
							if((fRelay3Status)&(!fStartManuelWork)&(!fStartAllRelayManuelWork))
							{
								fCloseRelay3 = 1;
							}
							if((fRelay4Status)&(!fStartManuelWork)&(!fStartAllRelayManuelWork))
							{
								fCloseRelay4 = 1;
							}
						}
						else
						{
							if((fhomePage)||(fHomePageWithPassword))
							{
								if(fSystemOFF)
								{
									fSystemOFF = 0;
									NVIC_SystemReset();
								}	
								else	
								{
									LCDBufferReset();
									fCloseRelay1 = 1;
									fCloseRelay2 = 1;
									fCloseRelay3 = 1;
									fCloseRelay4 = 1;
									fSystemOFF = 1;
									sleepCounter = 0;
									fsleepEnable = 1;
								}
							}
						}
					}	
					if ((presdb == 0x02)&(!fSystemOFF)&(!fproducerPasswordNeeded))			// Forward Button LONG Pressed 
					{	  
						if(fhomePage)
						{
								fEraseAllPrograms = 1;
								fCloseRelay1 = 1;
								fCloseRelay2 = 1;
								fCloseRelay3 = 1;
								fCloseRelay4 = 1;
							}
						}
					}						
				}				
				if (lvd2cnt!= 255)		
				{
					lvd2cnt++;
					if (lvd2cnt == 5)			// 8*160 = ~800ms 
					{
						--lvd2cnt;
						if ((presdb == 0x10)&(!fSystemOFF)&(!fproducerPasswordNeeded))				// - Button Pressed 	
						{
							dbprs();			
						}
						if ((presdb == 0x08)&(!fSystemOFF)&(!fproducerPasswordNeeded))				// + Button Pressed
						{
							ibprs();
						}	
					}
				}
			}						
		}	
	}		
	else
	{
		vdcnt = 0;
		lvdcnt = 0;
		lvd2cnt = 0;
		prvdat = butdat;
		if(fShowBatteryLevel == 1)
		{
			LCDBufferReset();
		}
		fShowBatteryLevel = 0;
		fshowCurrentDayOfWeek = 0;
	}
}


