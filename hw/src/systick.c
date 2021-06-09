/******************************************************************************/
/** @file       systick.c
 *******************************************************************************
 *
 *  @brief      General systick module for stm32l0
 *              <p>
 *              Allows to generate delays in busy loop
 *
 *  @author     wht4
 *
 *  @remark     Last modifications
 *                 \li V1.0, February 2016, wht4, initial release
 *
 ******************************************************************************/
/*
 *  functions  global:
 *              systick_init
 *              systick_delayMs
 *              systick_getTick
 *              systick_irq
 *  functions  local:
 *              .
 *
 ******************************************************************************/

/****** Header-Files **********************************************************/
#include "stm32l0xx.h"
#include "systick.h"
#include "nxp_lcd_driver.h"
#include "selenoid.h"
#include "button.h"

int f1ms = 0;
int f2ms = 0;
int f20ms = 0;
int f200ms = 0;
int f1s = 0;
int f2s = 0;
int f10s = 0;

int cnt2ms = 0;
int cnt20ms = 0;
int cnt200ms = 0;
int cnt1s = 0;
int cnt2s = 0;
int cnt10s = 0;

int fsleepEnable = 0;
int sleepCounter = 0;

extern void myWatchDogBOMB(void);

/****** Macros ****************************************************************/

/****** Data types ************************************************************/

/****** Fuction prototypes ****************************************************/

/****** Data ******************************************************************/
uint32_t u32Tick = 0;

/****** Implementation ********************************************************/


/*******************************************************************************
 *  function :    systick_init
 ******************************************************************************/
void
systick_init(uint32_t u32Ticks) {

    SysTick_Config(u32Ticks);
}


/*******************************************************************************
 *  function :    systick_delayMs
 ******************************************************************************/
void
systick_delayMs(uint32_t u32DelayMs) {

    uint32_t u32StartTick = u32Tick;
    while((u32DelayMs + u32StartTick) > u32Tick) {
        // TODO Go to sleep mode within busy loop
    }
}


/*******************************************************************************
 *  function :    systick_getTick
 ******************************************************************************/
uint32_t
systick_getTick(void) {
	return (u32Tick);
}


/*******************************************************************************
 *  function :    systick_irq
 ******************************************************************************/
void systick_irq(void) {

  u32Tick++;
	
	f1ms = 1;

//-----------------//
//--MY WATCHDOG---//	
//-----------------//	
	if(!fsleepEnable)
	{
		++myWatchDogCnt;
		if(myWatchDogCnt == 10000) // 10 sec no response, so reset system
		{
				NVIC_SystemReset();		// restleme yerine tekrar init edilebilir 
		}
	}	
	
//-----------------//
//-----COUNTER-----//	
//-----------------//		
//*********	
	++cnt2ms; 
	if(cnt2ms >= 2 )
	{
		cnt2ms = 0;
		f2ms = 1;
	}
//*********		
	++cnt20ms;
	if(cnt20ms >= 20 )
	{
		cnt20ms = 0;
		f20ms = 1;
	}
//*********
//*********		
	++cnt200ms;
	if(cnt200ms >= 200 )
	{
		cnt200ms = 0;
		f200ms = 1;
	}
//*********		
	++cnt1s;
	if(cnt1s >= 1000 )
	{
		cnt1s = 0;
		f1s = 1;
	}	
//*********			
//*********		
	++cnt2s;
	if(cnt2s >= 2000 )
	{
		cnt2s = 0;
		f2s = 1;
	}	
//*********		
//*********		
	++cnt10s;
	if(cnt10s >= 10000 )
	{
		cnt10s = 0;
		f10s = 1;
	}
	
	
if(fResetLCDBufferwithDelay)	
{
	--resetLCDBufferDelayMs;
	if(resetLCDBufferDelayMs == 0)
	{
		fResetLCDBufferwithDelay = 0;
		fResetLCDBuffer = 1;
	}
}	


	
if(fOpenRelay_1_WithDelay)	
{
	--relayOpenDelayMs;
	if(relayOpenDelayMs == 0)
	{
		fOpenRelay_1_WithDelay = 0;
		fOpenRelay1 = 1;
	}
}	

if(fOpenRelay_2_WithDelay)	
{
	--relayOpenDelayMs;
	if(relayOpenDelayMs == 0)
	{
		fOpenRelay_2_WithDelay = 0;
		fOpenRelay2 = 1;
	}
}	

if(fOpenRelay_3_WithDelay)	
{
	--relayOpenDelayMs;
	if(relayOpenDelayMs == 0)
	{
		fOpenRelay_3_WithDelay = 0;
		fOpenRelay3 = 1;
	}
}	

if(fOpenRelay_4_WithDelay)	
{
	--relayOpenDelayMs;
	if(relayOpenDelayMs == 0)
	{
		fOpenRelay_4_WithDelay = 0;
		fOpenRelay4 = 1;
	}
}	


//-----------------//
//--SLEEP COUNTER--//	
//-----------------//	
	if(sleepCounter != 0)	
	{
		--sleepCounter;
		if(sleepCounter == 0 )
		{
			fsleepEnable = 1;
			fWakeUpFromRTC = 0;
		}	
	}
	
//-----------------//
//--AFTER WAKEUP COUNTER--//	
//-----------------//	
	if(afterWakeUpCnt != 0)	
	{
		--afterWakeUpCnt;
	}	
	
//-----------------//
//--AFTER WAKEUP COUNTER--//	
//-----------------//	
	if(startManuelWorkDelay != 0)	
	{
		--startManuelWorkDelay;
	}	
		
	
	
//-----------------//
//----POWER ON-----//	
//-----------------//	
	if(powerON)
	{
		if(cntPowerOn4second)
		{
			--cntPowerOn4second;
			
			if(cntPowerOn4second>=2400)
			{
				if(cntPowerOn4second==2400)
				{
					fCloseRelay1 = 1;
				}
				fPowerOnRelay1OFF = 1;
				fPowerOnRelay2OFF = 0;
				fPowerOnRelay3OFF = 0;
				fPowerOnRelay4OFF = 0;
			}
			else if(cntPowerOn4second>=1600)
			{
				if (SELENOID_NUMBER == 1)
				{
					cntPowerOn4second = 0; 
				}
				else
				{
					if(cntPowerOn4second==1600)
					{
						fCloseRelay2 = 1;
					}
					fPowerOnRelay1OFF = 0;
					fPowerOnRelay2OFF = 1;
					fPowerOnRelay3OFF = 0;
					fPowerOnRelay4OFF = 0;
				}
			}
			else if(cntPowerOn4second>=800)
			{
				if (SELENOID_NUMBER == 2)
				{
					cntPowerOn4second = 0; 
				}
				else
				{
					if(cntPowerOn4second==800)
					{
						fCloseRelay3 = 1;
					}
					fPowerOnRelay1OFF = 0;
					fPowerOnRelay2OFF = 0;
					fPowerOnRelay3OFF = 1;
					fPowerOnRelay4OFF = 0;
				}
			}
			else
			{
				if (SELENOID_NUMBER == 3)
				{
					cntPowerOn4second = 0; 
				}
				else
				{
					if(cntPowerOn4second==0)
					{
						fCloseRelay4 = 1;
					}
					fPowerOnRelay1OFF = 0;
					fPowerOnRelay2OFF = 0;
					fPowerOnRelay3OFF = 0;
					fPowerOnRelay4OFF = 1;
				}
			}
		}
		else
		{
			fPowerOnRelay1OFF = 0;
			fPowerOnRelay2OFF = 0;
			fPowerOnRelay3OFF = 0;
			fPowerOnRelay4OFF = 0;
			powerON = 0;
			cntPowerOn4second = 3200;
			fwatchAdjustment = 1;
			fResetLCDBuffer = 1;
			if(fDeviceLockedWithPassword)
			{
				fHomePageWithPassword = 1;
				fPasswordDigit1Adjust = 1;
				fwatchAdjustment = 0;
			}
		}	
	}
}

