/******************************************************************************/
/** @file       main.c
 *******************************************************************************
 *
 *  @brief      SUPIRI  
 *
 ******************************************************************************/


/****** Header-Files **********************************************************/
#include "stm32l0xx.h"
#include "adc.h"
#include "hw.h"
#include "gpio.h"
#include "led.h"
#include "lcd.h"
#include "button.h"
#include "selenoid.h"
#include "systick.h"
#include "btn.h"
#include "systick.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"

#include "rtc.h"
#include "i2c.h"
#include "nxp_lcd_driver.h"

#define CONFIG_SYSTICK_1MS           ( 16000UL )

#define ERROR_ERASE 0x01
#define ERROR_PROG_BYTE  0x02
#define ERROR_PROG_16B_WORD 0x04
#define ERROR_PROG_32B_WORD 0x08
#define ERROR_WRITE_PROTECTION 0x10
#define ERROR_READOUT_PROTECTION 0x20
#define ERROR_FETCH_DURING_PROG 0x40
#define ERROR_SIZE 0x80
#define ERROR_ALIGNMENT 0x100
#define ERROR_NOT_ZERO 0x200
#define ERROR_OPTION_NOT_VALID 0x400
#define ERROR_UNKNOWN 0x800

#define FLASH_SR_FWWERR ((uint32_t)0x00020000)

#define DATA_E2_ADDR   ((uint32_t)0x08080000)  /* Data EEPROM address */

#define WWDG_REFRESH      (0x7F)

/* Private define ------------------------------------------------------------*/
/* NVM key definitions */
#define FLASH_PDKEY1               ((uint32_t)0x04152637) /*!< Flash power down key1 */
#define FLASH_PDKEY2               ((uint32_t)0xFAFBFCFD) /*!< Flash power down key2: used with FLASH_PDKEY1 
                                                              to unlock the RUN_PD bit in FLASH_ACR */

#define FLASH_PEKEY1               ((uint32_t)0x89ABCDEF) /*!< Flash program erase key1 */
#define FLASH_PEKEY2               ((uint32_t)0x02030405) /*!< Flash program erase key: used with FLASH_PEKEY2
                                                               to unlock the write access to the FLASH_PECR register and
                                                               data EEPROM */

#define FLASH_PRGKEY1              ((uint32_t)0x8C9DAEBF) /*!< Flash program memory key1 */
#define FLASH_PRGKEY2              ((uint32_t)0x13141516) /*!< Flash program memory key2: used with FLASH_PRGKEY2
                                                               to unlock the program memory */

#define FLASH_OPTKEY1              ((uint32_t)0xFBEAD9C8) /*!< Flash option key1 */
#define FLASH_OPTKEY2              ((uint32_t)0x24252627) /*!< Flash option key2: used with FLASH_OPTKEY1 to
                                                              unlock the write access to the option byte block */

extern void BtnCallback (Btn_t tBtn, BtnHandlingCtx_t tBtnHandlingCtx);
extern void FLASH_IRQHandler(void);
void myWatchDogBOMB(void);
void UnlockPELOCK(void);
void LockNVM(void);
void EepromErase(uint32_t addr);volatile uint16_t error = 0;
void erase_all_programs(void);
void programCelendar(void);
uint8_t updateRTCAlarm(void);
void celenderSort(void);
void updateSelenoidPosition(uint8_t prgNum);
void manuelWorkCelender(void);
void manuelAllWorkCelender(void);

void sleepCon (void);
void batteryVoltageCon(void);
float Voltage_Conversion(int ADC_Reading);
void Configure_WWDG(void);
void recort_to_EEPROM(void);
void read_from_EEPROM(void);
void recort_password_to_EEPROM(void);
void copyAndPasteSelSettings(void);
void resetPendingRelay(void);
void cancelPrevIrr(void);
void updateInvalidPassWord(void);

__INLINE void read_Password_from_EEPROM(void);
	
int digcnt = 0;
int testingMode = 0;
int adcSensReading = 0;
uint8_t activeProgram = 0;

float readBatteryVoltage = 0;
float batteryVoltageSample[10] = {0};
int batteryRecordCnt = 0;
		
		
/*******************************************************************************
 *  function :    main
 ******************************************************************************/
int main(void) 
{
	/*************************************
	......
	....
	..
	.
	INIT SYSTEM
	.
	..
	....
	......
	**************************************/
	SystemInit();
	hw_initSysclock();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
	hw_init();
	for(int i = 0; i<100; i++); //wait a little

	selenoid_init();
	selenoid_sleep();
	selenoid_disable();
	/*************************************
	......
	....
	..
	.
	CHECK TIME AND DATE
	.
	..
	....
	......
	**************************************/	
	uint32_t fDateTimeInitFlag = 0;
	uint32_t updateRTCDate = 0;
	uint32_t updateRTCTime = 0;
	fDateTimeInitFlag = (*(uint8_t *)(DATA_E2_ADDR+54));	
	if(!fDateTimeInitFlag)
	{
		UnlockPELOCK();
		FLASH->PECR |= FLASH_PECR_ERRIE | FLASH_PECR_EOPIE; /* enable flash interrupts */
			*(uint8_t *)(DATA_E2_ADDR+54) = 1; /* SET EMPROM INIT FLAG */
		__WFI();
		LockNVM();	
		Init_RTC(0x00000000, 0x0019D207 ); //Time, Date  ilk kurulum bu sekilde.
	}
	else
	{
		updateRTCDate = (*(uint8_t *)(DATA_E2_ADDR+55));
		updateRTCDate = updateRTCDate | (*(uint8_t *)(DATA_E2_ADDR+56))<<8; 
		updateRTCDate = updateRTCDate | (*(uint8_t *)(DATA_E2_ADDR+57))<<16;
		updateRTCDate = updateRTCDate | (*(uint8_t *)(DATA_E2_ADDR+58))<<24; 
		updateRTCTime = (*(uint8_t *)(DATA_E2_ADDR+59));
		updateRTCTime = updateRTCTime | (*(uint8_t *)(DATA_E2_ADDR+60))<<8; 
		updateRTCTime = updateRTCTime | (*(uint8_t *)(DATA_E2_ADDR+61))<<16;
		updateRTCTime = updateRTCTime | (*(uint8_t *)(DATA_E2_ADDR+62))<<24; 	
		Init_RTC(updateRTCTime, updateRTCDate ); //Time, Date  ilk kurulum bu sekilde.			
	}	
	
	
	/*************************************
	......
	....
	..
	.
	READ_STORED_DATA_FROM_EEPROM
	.
	..
	....
	......
	**************************************/	
	uint32_t fEepromInitFlag = 0;
	fEepromInitFlag = (*(uint8_t *)(DATA_E2_ADDR+53));	
	if(!fEepromInitFlag)
	{
		UnlockPELOCK();
		FLASH->PECR |= FLASH_PECR_ERRIE | FLASH_PECR_EOPIE; /* enable flash interrupts */
			*(uint8_t *)(DATA_E2_ADDR+53) = 1; /* SET EMPROM INIT FLAG */
		__WFI();
		LockNVM();	
		fRecordtoEEPROM = 1;  // program ilk kez çalistiginda ( EEPROM BOS ISE ) EEPROM'a default ayarlari kaydet
	}
	else
	{
		read_from_EEPROM(); 
	}
	/*************************************
	......
	....
	..
	.
	CHECK IF PASSWORD SETTED OR NOT
	.
	..
	....
	......
	**************************************/		
	read_Password_from_EEPROM();
	if(fPasswordSetted)
	{
		fDeviceLockedWithPassword = 1;
	}
	
	/*************************************
	......
	....
	..
	.
	READ_IVALID ERROR COUNTER
	.
	..
	....
	......
	**************************************/	
	invalidPasswordCnt = (*(uint8_t *)(DATA_E2_ADDR+53));	
	/*************************************
	......
	....
	..
	.
	WAIT FOR INTERNAL HSI OSCILLATOR INIT
	.
	..
	....
	......
	**************************************/		
//	LCD_Ram_Data_Row[10] = sevenSegCode('t');
//	LCD_Ram_Data_Row[11] = sevenSegCode('i');
//	LCD_Ram_Data_Row[12] = sevenSegCode('n');		// PowerOn durumda röleler sirasiyla kapatilir ve ekranda kapali yanip söner.
//	LCD_Ram_Data_Row[13] = sevenSegCode('I');
//	LCD_Ram_Data_Row[14] = sevenSegCode('X');
//	LCD_Ram_Data_Row[15] = sevenSegCode('X');	
//	I2C_Write_LCD(0x40, LCD_Ram_Data_Row, 30);
	/*************************************/
	//while(fHSIOscillatorInit);	 							//  wait until HSI stable 
	/*************************************
	......
	....
	..
	.
	CLEAR LCD
	.
	..
	....
	......
	**************************************/	
	I2C_Write_LCD(0x40, LCD_Ram_Data_Row, 30);   // RESET LCD RAM		
	/*************************************

	......
	....
	..
	.
	MAIN LOOP
	.
	..
	....
	......
	**************************************/	
	configureAlarmB(0x80000000);
	sleepCounter = SLEEP_COUNTER_VAL;
	fUpdateRtcAlarm = 1;  // sistem resetlenirse sulama yapacak
	finitRtcAlarm = 1;  // sistem resetlenirse geçmis sulamalari dikkate alma
	/**************************************/
	while (1) 
	{
		if(f2ms)
		{
			f2ms = 0;   
			myWatchDogCnt = 0;  // Refresh whatchdog counter
			recort_to_EEPROM();
			recort_password_to_EEPROM();	
			read_from_EEPROM();	
			erase_all_programs();
			updateInvalidPassWord();
		}
		if(f20ms)
		{
			f20ms = 0;
			butread();
			copyAndPasteSelSettings();
			if(fResetLCDBuffer)
			{
				fResetLCDBuffer = 0;
				LCDBufferReset();
			}
			updateLCDBuffer();
			I2C_Write(0x80,0x00);		// 	Reset data pointer		
			I2C_Write_LCD(0x40, LCD_Ram_Data_Row, 30);						
		}
		if(f200ms)
		{
			f200ms = 0;	
			if((!afterWakeUpCnt) && (!powerON) && (!fSystemOFF))
			{
				batteryVoltageCon();
			}	
		}
		if(f1s)
		{	
			f1s = 0;
			if((fUpdateRtcAlarm)&(!fselenoidProgramming))
			{
				fUpdateRtcAlarm = 0;	//
				if(fStartManuelWork)
				{
					manuelWorkCelender();
				}
				else if(fStartAllRelayManuelWork)
				{
					resetPendingRelay();	//
					manuelAllWorkCelender();
					updateRTCAlarm(); 		//						
				}
				else
				{
					programCelendar();		//		
					resetPendingRelay();	//
					celenderSort(); 			//
					if(finitRtcAlarm) // ayar degistirildiginde buffer Update edilir ama zamani geçmis sulamalar iptal edilir.
					{
						cancelPrevIrr();
					}
					updateRTCAlarm(); 		//		
				}
			}
			if((fSetNextAlarm)&(!fselenoidProgramming))
			{
				fSetNextAlarm = 0;
				activeProgram = updateRTCAlarm(); 		//
				if(!fSystemOFF)
				{
					updateSelenoidPosition(activeProgram);
				}
			}
			selenoidCon();
			sleepCon();	
		}		
		if(f2s)
		{
			f2s = 0;
		}
		if(f10s)
		{
			f10s = 0;	
			myWatchDogBOMB();			
		}
	}
}


/*******************************************************************************
 *  function :    BATTERY VOLTAGE CONTROL
 ******************************************************************************/
void batteryVoltageCon(void)
{
		ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */	
		systick_delayMs(10);
		adcSensReading = ADC1->DR;
		ADC1->CR |= ADC_CR_ADSTP; /* start the ADC conversion */	
		readBatteryVoltage = Voltage_Conversion(adcSensReading);
		batteryVoltageSample[batteryRecordCnt] = readBatteryVoltage;
		++batteryRecordCnt;
		if(batteryRecordCnt >= 10)
		{
			fBatteryReady = 1;
			batteryRecordCnt = 0;
			batteryVoltage = 0;
			for(int i = 0; i<10; i++)
			{
				batteryVoltage = batteryVoltage + batteryVoltageSample[i]/10;
			}
				//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				batteryVoltage = batteryVoltage + 1.482;  // For Calibration!!! Calibration factor eklenecek
				//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		}

		if (batteryVoltage > 9)
		{
			batteryVoltage = 9;
		}
		
		if (batteryVoltage < 6.8)  // batvoltaj<6.8 röleleri kapat, pil bitiyor
		{
			LCDPowerOFF();
			fCloseRelay1 = 1;
			fCloseRelay2 = 1;
			fCloseRelay3 = 1;
			fCloseRelay4 = 1;
			selenoidCon();
		}
}


/*******************************************************************************
 *  function :    SLEEP CONTROL
 ******************************************************************************/
void sleepCon(void)
{
	if(fsleepEnable)
	{	
		LCDBufferReset();		
		uint32_t command1 = 0x00;  //load data pointer to 0
		I2C_Write(0x80,command1);
		I2C_Write_LCD(0x40, LCD_Ram_Data_Row, 30);	// LCD ekranini ve datayi sil
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");					
		I2C_Write(0x80,0xc0);	//mode set  -- DISABLE
		asm("nop");												 
		asm("nop");
		DAC->CR &= ~DAC_CR_EN1;
		DisableADC();
		//
		adc_sens_read_disable();
		adc_bat_read_disable();
		//
		selenoid_sleep();
		selenoid_disable();
		//
		LCDPowerOFF();
		//
		Reset_I2C();  // disable yapmadik
		//
//	/*Disable Clock for I2C*/
		//
		myWatchDogCnt = 0;  // Refresh whatchdog counter
		//
		//PWR->CR &= ~PWR_CR_DBP;		//orj comment
		//PWR->CR |= PWR_CR_DSEEKOFF;		//orj comment
		
		//
		RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
		RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
		RCC->APB1ENR &= ~RCC_APB1ENR_LCDEN;
	
		
		RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN; /* (1) */
		
		
		 
		RCC->APB2SMENR &= ~RCC_APB2SMENR_SYSCFGSMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB2SMENR_TIM21SMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB2SMENR_TIM22SMEN; /* (1) */
		//RCC->APB2SMENR &= ~RCC_APB2SMENR_ADC1SMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB2SMENR_SPI1SMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB2SMENR_USART1SMEN; /* (1) */
		//RCC->APB2SMENR &= ~RCC_APB2SMENR_DBGMCUSMEN; /* (1) */   // orj comment
		//
//		// Mantiksiz
		RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
		DBGMCU->CR &= ~DBGMCU_CR_DBG; /* To be able to debug in all power down modes */
		RCC->APB2ENR &= ~RCC_APB2ENR_DBGMCUEN;
//		//
		//
		//
		NVIC_SetPriority(EXTI0_1_IRQn, 0); /**/ 
		NVIC_EnableIRQ(EXTI0_1_IRQn); /*ENABLE BUTTON INTERRUPT*/ 
		//
		arrangeGPIOforSleep();
		//
		tmpPassword[0] = 0;
		tmpPassword[1] = 0;
		tmpPassword[2] = 0;   // sifreler gösterilmesin
		tmpPassword[3] = 0;
		
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
		//
		fwatchAdjustment = 0;
		fmanuelIrrigationPlanAdjust = 0;
		fmanuelIrrigationAdjust = 0;
		fselenoidProgramming = 0;
		fManuelRelayAdjust = 0;
		fPasswordActivateMode = 0;
		fPasswordAdjust = 0;
		fAll_Her_Selected = 0;
		clearAllDetailAdjustment();
		//		
		asm("nop");												 
		asm("nop");
		asm("nop");												 
		asm("nop");		
		asm("nop");												 
		asm("nop");
		asm("nop");												 
		asm("nop");	
		//
		//
		sleepCounter = 0; // normalde 0, RTC interruptinda güvenlik amaçli.
		fWakeUpFromRTC = 0; //
		//		
		btn_registerCallback(BTN_1, BTN_HANDLING_CTX_ISR, BtnCallback);
		//
    /* Prepare for Standby */
    // if WKUP pins are already high, the WUF bit will be set
		PWR->CSR |= PWR_CSR_EWUP1 | PWR_CSR_EWUP1 ;
//     
    PWR->CR |= PWR_CR_CWUF; // clear the WUF flag after 2 clock cycles
    PWR->CR |= PWR_CR_ULP;   // V_{REFINT} is off in low-power mode
		PWR->CR |= PWR_CR_FWU;  // enable fast wake up
		
    PWR->CR &= ~PWR_CR_PDDS; 
		//PWR->CR |= PWR_CR_PDDS; // Enter Standby mode when the CPU enters deepsleep
    //PWR->CR |= PWR_CR_LPSDSR; // *!< Low-power deepsleep/sleep/ Internal Regulator Off*/  
		
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // low-power mode = stop mode
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; // reenter low-power mode after ISR

		__DSB(); // Ensures SLEEPONEXIT is set immediately before sleep		

		__disable_irq();
		SysTick->CTRL  &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |SysTick_CTRL_ENABLE_Msk);  /* Enable SysTick IRQ and SysTick Timer */
		//
//		NVIC_SetPriority(RTC_IRQn, 0); /**/ 
//		NVIC_EnableIRQ(RTC_IRQn); /*ENABLE BUTTON INTERRUPT*/ 
		//	
		EXTI->PR = 0xFFFFFFFF;
		__WFI(); // Go to sleep
		//
		//
		__enable_irq();
		NVIC_EnableIRQ(SysTick_IRQn);
		NVIC_SetPriority(EXTI0_1_IRQn, 0); /**/ 
		NVIC_EnableIRQ(EXTI0_1_IRQn); /*ENABLE BUTTON INTERRUPT*/ 
	}	
}

/*******************************************************************************
 *  function :    WAKEUP FROM SLEEP MODE
 ******************************************************************************/

void wakeUp(void)
{
		SystemInit();

		hw_initSysclock();    // bunlar kaldirildiktan sonra güç tüketimi ölçülmedi
		asm("nop");
		asm("nop");   
		asm("nop");	
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");   
		asm("nop");
		asm("nop");	;
		SystemCoreClockUpdate();
		asm("nop");
		asm("nop");   
		asm("nop");	
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");   
		asm("nop");
		asm("nop");	;
		SysTick_Config(SystemCoreClock / 1000);
		asm("nop");
		asm("nop");   
		asm("nop");	
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");   
		asm("nop");
		asm("nop");	;
		///wait a little for clock stabilation
		for(long int k=0; k<50000;k++);
		////
		asm("nop");
		asm("nop");   
		asm("nop");	
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");   
		asm("nop");
		asm("nop");
		hw_init();
		asm("nop");
		asm("nop");   
		asm("nop");	
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");   
		asm("nop");
		asm("nop");
		myWatchDogCnt = 0;  // Refresh whatchdog counter
		afterWakeUpCnt = 10000; // uyandiktan sonra hemen pil kontrolüne baslamasin, 10 sn ye sonra, stabilizasyon için
	  //PWR->CR &= ~PWR_CR_ULP;   // V_{REFINT} is on
		PWR->CR &= ~PWR_CR_FWU;  // disable fast wake up
		//PWR->CR &= ~PWR_CR_LPSDSR; // *!< Low-power deepsleep/sleep/ Internal Regulator Off*/  
		//RCC->IOPSMENR |= RCC_AHBENR_MIFEN; /* (1) */		
		asm("nop");
		asm("nop");   
		asm("nop");
		asm("nop");
		asm("nop");   
		asm("nop");
		asm("nop");	
		LCDBufferReset();		
		LCDPowerON();	   // LCD Power ON, Wait for stabilization

		selenoid_init();
		fsleepEnable = 0;
		batteryRecordCnt = 0;
		sleepCounter = 5000; // sistem kapali ise 5 saniye
		if(!fSystemOFF)
		{
			sleepCounter = SLEEP_COUNTER_VAL; // sistem kapali degilse 30 saniye
		}
//		if(fWakeUpFromRTC)   // RTC 'den uyandiginda sadece 5 saniye açik kalsin ve ADC hiç okumaya baslamasin diye kullanilabilir
//		{
//			fWakeUpFromRTC = 0;
//			sleepCounter = 5000; // sistem RTC ile uyanmissa gece 00' da uyanmissa 5 saniye sonra geri uyanki
//		}
		
		Reset_I2C();
		//
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk; // Disable SLEEPONEXIT
		__DSB(); // Ensures SLEEPONEXIT is set
		asm("nop");   
		asm("nop");
		asm("nop");
		asm("nop");		
		// burata tüm ayar modlarini sifirla		 !!!!!!!!!!!!!!!!!!! 
		fhomePage = 1;
		if(fPasswordSetted)
		{
			fhomePage = 0;
			fHomePageWithPassword = 1;
			fDeviceLockedWithPassword = 1;
			fPasswordDigit4Adjust = 0;
			fPasswordDigit3Adjust = 0;
			fPasswordDigit2Adjust = 0;
			fPasswordDigit1Adjust = 1;
		}
		configureAlarmB(0x80000000);  /// her günn 00:00 da interup alip o günün sulamasi planlanacak, bazen alarm iptal oluyor, sorunu bulamadigim için burada sürekli kuruyorum.
	}

/*******************************************************************************
 *  function :    BtnCallback - WAKEUP FROM SLEEP MODE with BUTTON PRESS
 ******************************************************************************/
void BtnCallback (Btn_t tBtn, BtnHandlingCtx_t tBtnHandlingCtx) 
{
		fWakeUpFromRTC = 0;
		wakeUp();
    (void) tBtn;
    (void) tBtnHandlingCtx;
}


/**
  \fn					Voltage_Conversion(int ADC_Reading)
  \brief			Convert the ADC value into Voltage
	\param			int ADC_Reading: The converted value to assess
	\returns		float voltage: actual voltage
*/


/*******************************************************************************
 *  function :    Voltage_Conversion - Convert ADC Readings to Voltage Value
 ******************************************************************************/
float Voltage_Conversion(int ADC_Reading){

	/* Local Variables */
	float voltage = 0.0;
	float voltage_Per_Division = (3.3/4096)*5.7;		//Supply voltage is known as 3.3.
	
	/* Calculate Voltage */
	voltage = ADC_Reading*voltage_Per_Division;
//	if(voltage < 6)
//	{
//		voltage = 6;
//	}
	return(voltage);
}

/**
  * Brief   This function unlocks the data EEPROM and the FLASH_PECR. 
  *         The data EEPROM will be ready to be erased or programmed
  *         but the program memory will be still locked till PRGLOCK is set.
  *         It first checks no flash operation is on going,
  *         then unlocks PELOCK if it is locked.
  * Param   None
  * Retval  None
  */
__INLINE void UnlockPELOCK(void)
{  
  /* (1) Wait till no operation is on going */
  /* (2) Check if the PELOCK is unlocked */
  /* (3) Perform unlock sequence */
	long unsigned int timeOut = 0;
  while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */  
  {
    /* For robust implementation, add here time-out management */
		++timeOut;
		if(timeOut>timeOutVal)
		{
			break;
		}
  }  
  if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0) /* (2) */
  {    
    FLASH->PEKEYR = FLASH_PEKEY1; /* (3) */
    FLASH->PEKEYR = FLASH_PEKEY2;
  }
}


/**
  * Brief   This function locks the NVM.
  *         It first checks no flash operation is on going,
  *         then locks the flash.
  * Param   None
  * Retval  None
  */
__INLINE void LockNVM(void)
{  
  /* (1) Wait till no operation is on going */
  /* (2) Locks the NVM by setting PELOCK in PECR */
	long unsigned int timeOut = 0; 	
  while ((FLASH->SR & FLASH_SR_BSY) != 0)  /* (1) */  
  {
    /* For robust implementation, add here time-out management */
		++timeOut;
		if(timeOut>timeOutVal)
		{
			break;
		}
  }  
  FLASH->PECR |= FLASH_PECR_PELOCK; /* (2) */
}

/**
  * Brief   This function erases a word of data EEPROM.
  *         The ERASE bit and DATA bit are set in PECR at the beginning 
  *         and reset at the endof the function. In case of successive erase, 
  *         these two operations could be performed outside the function.
  *         The flash interrupts must have been enabled prior to call 
  *         this function.
  * Param   addr is the 32-bt word address to erase
  * Retval  None
  */
__INLINE void EepromErase(uint32_t addr)
{   
  /* (1) Set the ERASE and DATA bits in the FLASH_PECR register 
         to enable page erasing */
  /* (2) Write a 32-bit word value at the desired address 
         to start the erase sequence */
  /* (3) Enter in wait for interrupt. The EOP check is done in the Flash ISR */
  /* (6) Reset the ERASE and DATA bits in the FLASH_PECR register 
         to disable the page erase */
  FLASH->PECR |= FLASH_PECR_ERASE | FLASH_PECR_DATA; /* (1) */    
  *(__IO uint32_t *)addr = (uint32_t)0; /* (2) */
  __WFI(); /* (3) */
  FLASH->PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_DATA); /* (4) */
}



/*=====================================================================
				*********** 	COPY PASTE SETTINGS ************			
=====================================================================*/

void copyAndPasteSelSettings(void)
{
	if(fcopyAndPasteSelSettings)
	{	
		fcopyAndPasteSelSettings = 0;
		sel_work_duration[3] = sel_work_duration[0];
		sel_work_duration[2] = sel_work_duration[0];	
		sel_work_duration[1] = sel_work_duration[0];
		
		sel_delayed_irr_val[3] = sel_delayed_irr_val[0];
		sel_delayed_irr_val[2] = sel_delayed_irr_val[0];	
		sel_delayed_irr_val[1] = sel_delayed_irr_val[0];
		
		sel_manuel_work_duration[3] = sel_manuel_work_duration[0];
		sel_manuel_work_duration[2] = sel_manuel_work_duration[0];	
		sel_manuel_work_duration[1] = sel_manuel_work_duration[0];	
		
		sel_special_irr_days[3] = sel_special_irr_days[0];
		sel_special_irr_days[2] = sel_special_irr_days[0];	
		sel_special_irr_days[1] = sel_special_irr_days[0];		

		sel_start_time_min[1][0] = sel_start_time_min[0][0];
		sel_start_time_min[1][1] = sel_start_time_min[0][1];
		sel_start_time_min[1][2] = sel_start_time_min[0][2];
		sel_start_time_min[1][3] = sel_start_time_min[0][3];
		sel_start_time_min[2][0] = sel_start_time_min[0][0];
		sel_start_time_min[2][1] = sel_start_time_min[0][1];
		sel_start_time_min[2][2] = sel_start_time_min[0][2];
		sel_start_time_min[2][3] = sel_start_time_min[0][3];	
		sel_start_time_min[3][0] = sel_start_time_min[0][0];
		sel_start_time_min[3][1] = sel_start_time_min[0][1];
		sel_start_time_min[3][2] = sel_start_time_min[0][2];
		sel_start_time_min[3][3] = sel_start_time_min[0][3];	

		sel_start_time_hour[1][0] = sel_start_time_hour[0][0];
		sel_start_time_hour[1][1] = sel_start_time_hour[0][1];
		sel_start_time_hour[1][2] = sel_start_time_hour[0][2];
		sel_start_time_hour[1][3] = sel_start_time_hour[0][3];
		sel_start_time_hour[2][0] = sel_start_time_hour[0][0];
		sel_start_time_hour[2][1] = sel_start_time_hour[0][1];
		sel_start_time_hour[2][2] = sel_start_time_hour[0][2];
		sel_start_time_hour[2][3] = sel_start_time_hour[0][3];	
		sel_start_time_hour[3][0] = sel_start_time_hour[0][0];
		sel_start_time_hour[3][1] = sel_start_time_hour[0][1];
		sel_start_time_hour[3][2] = sel_start_time_hour[0][2];
		sel_start_time_hour[3][3] = sel_start_time_hour[0][3];
	}	
}


/*******************************************************************************
 *  function :   ERASE TO EEPROM
 ******************************************************************************/
/***/
__INLINE void erase_all_programs(void)
{
	if(fEraseAllPrograms)
	{
		fEraseAllPrograms = 0;   
//		UnlockPELOCK();
//		for(int i = 0; i<48; i++)
//		{
//			EepromErase(DATA_E2_ADDR + i);    // döngüden çikmiyor
//		}
//		LockNVM();
		//RESET USER ADJUSTED VALUES
		sel_work_duration[0] = 0;
		sel_work_duration[1] = 0;
		sel_work_duration[2] = 0;
		sel_work_duration[3] = 0;
		sel_delayed_irr_val[0] = 0;
		sel_delayed_irr_val[1] = 0;
		sel_delayed_irr_val[2] = 0;
		sel_delayed_irr_val[3] = 0;
		sel_manuel_work_duration[0] = 0;
		sel_manuel_work_duration[1] = 0;
		sel_manuel_work_duration[2] = 0;
		sel_manuel_work_duration[3] = 0;
		sel_special_irr_days[0] = 0;
		sel_special_irr_days[1] = 0;
		sel_special_irr_days[2] = 0;   // ilk deger 0 olsun  Bits --> ACTIVATED, Pt, Sa, Ca, Pr, Cu, Ct, Pz -->
		sel_special_irr_days[3] = 0;	
		sel_start_time_min[0][0] = 0;
		sel_start_time_min[0][1] = 0xFF;
		sel_start_time_min[0][2] = 0xFF;	
		sel_start_time_min[0][3] = 0xFF;	
		sel_start_time_min[1][0] = 0;
		sel_start_time_min[1][1] = 0xFF;
		sel_start_time_min[1][2] = 0xFF;	
		sel_start_time_min[1][3] = 0xFF;	
		sel_start_time_min[2][0] = 0;
		sel_start_time_min[2][1] = 0xFF;
		sel_start_time_min[2][2] = 0xFF;	
		sel_start_time_min[2][3] = 0xFF;	
		sel_start_time_min[3][0] = 0;
		sel_start_time_min[3][1] = 0xFF;
		sel_start_time_min[3][2] = 0xFF;	
		sel_start_time_min[3][3] = 0xFF;	
		sel_start_time_hour[0][0] = 0;
		sel_start_time_hour[0][1] = 0xFF;
		sel_start_time_hour[0][2] = 0xFF;	
		sel_start_time_hour[0][3] = 0xFF;	
		sel_start_time_hour[1][0] = 0;
		sel_start_time_hour[1][1] = 0xFF;
		sel_start_time_hour[1][2] = 0xFF;	
		sel_start_time_hour[1][3] = 0xFF;	
		sel_start_time_hour[2][0] = 0;
		sel_start_time_hour[2][1] = 0xFF;
		sel_start_time_hour[2][2] = 0xFF;	
		sel_start_time_hour[2][3] = 0xFF;	
		sel_start_time_hour[3][0] = 0;
		sel_start_time_hour[3][1] = 0xFF;
		sel_start_time_hour[3][2] = 0xFF;	
		sel_start_time_hour[3][3] = 0xFF;
		
		//	
		for(int i= 0; i<32; i++)
		{
			relayCelendar[i] =0;
			pendingRtcIrq[i] =0;
			fUpdateRtcAlarm = 1;
		}
		//
	}
}


/*******************************************************************************
 *  function :   RECORD PASSWORD TO EEPROM
 ******************************************************************************/
/***/
__INLINE void recort_password_to_EEPROM(void)
{
	if(fUpdatePassword)
	{
		fUpdatePassword = 0;
		password[0] = tmpPassword[0];
		password[1] = tmpPassword[1];
		password[2] = tmpPassword[2];
		password[3] = tmpPassword[3];
		tmpPassword[0] = 0;
		tmpPassword[1] = 0;
		tmpPassword[2] = 0;
		tmpPassword[3] = 0;
		
		
		fRecordtoEEPROM = 0;
		UnlockPELOCK();
		FLASH->PECR |= FLASH_PECR_ERRIE | FLASH_PECR_EOPIE; /* enable flash interrupts */

		*(uint8_t *)(DATA_E2_ADDR+48) = password[0]; /* Write in data EEPROM */
		__WFI();
				*(uint8_t *)(DATA_E2_ADDR+49) = password[1]; /* Write in data EEPROM */
		__WFI();
				*(uint8_t *)(DATA_E2_ADDR+50) = password[2]; /* Write in data EEPROM */
		__WFI();
				*(uint8_t *)(DATA_E2_ADDR+51) = password[3]; /* Write in data EEPROM */
		__WFI();
				*(uint8_t *)(DATA_E2_ADDR+52) = fPasswordSetted; /* Write in data EEPROM */
		__WFI();
		
		LockNVM();
	}
}
/*******************************************************************************
 *  function :   RECORD TO EEPROM
 ******************************************************************************/
/***/
__INLINE void recort_to_EEPROM(void)
{
	if(fRecordtoEEPROM)
	{
		fRecordtoEEPROM = 0;
		UnlockPELOCK();
		FLASH->PECR |= FLASH_PECR_ERRIE | FLASH_PECR_EOPIE; /* enable flash interrupts */
		/* Check if the first address of the page is yet erased,this is only for this example */
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 1 selected work duration
	//--------------  
		EepromErase(DATA_E2_ADDR);
		__WFI();
		if (*(uint32_t *)(DATA_E2_ADDR) != (uint32_t)0)
		{
			error |= ERROR_ERASE;     // bu sekilde kullaranak her bir yazma islemi için ekrana error ve hata numarasi verilip yazdirilabilir
		}
		*(uint8_t *)(DATA_E2_ADDR) = sel_work_duration[0]; /* Write in data EEPROM */
		__WFI();
		if  (*(uint8_t *)(DATA_E2_ADDR) != sel_work_duration[0]) 
		{
			error |= ERROR_PROG_BYTE;
		}
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 2 selected work duration
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+1) = sel_work_duration[1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 3 selected work duration
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+2) = sel_work_duration[2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 4 selected work duration
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+3) = sel_work_duration[3]; /* Write in data EEPROM */
		__WFI();
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 1 aralikli sulama
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+4) = sel_delayed_irr_val[0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 2 aralikli sulama
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+5) = sel_delayed_irr_val[1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 3 aralikli sulama
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+6) = sel_delayed_irr_val[2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 4 aralikli sulama
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+7) = sel_delayed_irr_val[3]; /* Write in data EEPROM */
		__WFI();
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 1 aralikli sulama
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+8) = sel_manuel_work_duration[0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 2 aralikli sulama
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+9) = sel_manuel_work_duration[1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 3 aralikli sulama
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+10) = sel_manuel_work_duration[2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 4 aralikli sulama
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+11) = sel_manuel_work_duration[3]; /* Write in data EEPROM */
		__WFI();
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 1 özel sulama günleri // ilk deger 0x00 olsun  Bits --> ACTIVATED, Pt, Sa, Ca, Pr, Cu, Ct, Pz -->
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+12) = sel_special_irr_days[0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 2 özel sulama günleri // ilk deger 0x00 olsun  Bits --> ACTIVATED, Pt, Sa, Ca, Pr, Cu, Ct, Pz -->
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+13) = sel_special_irr_days[1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 3 özel sulama günleri // ilk deger 0x00 olsun  Bits --> ACTIVATED, Pt, Sa, Ca, Pr, Cu, Ct, Pz -->
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+14) = sel_special_irr_days[2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 4 özel sulama günleri // ilk deger 0x00 olsun  Bits --> ACTIVATED, Pt, Sa, Ca, Pr, Cu, Ct, Pz -->
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+15) = sel_special_irr_days[3]; /* Write in data EEPROM */
		__WFI();
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 1 Time 1 Start Time Min
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+16) = sel_start_time_min[0][0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 1 Time 2 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+17) = sel_start_time_min[0][1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 1 Time 3 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+18) = sel_start_time_min[0][2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 1 Time 4 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+19) = sel_start_time_min[0][3]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 2 Time 1 Start Time Min
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+20) = sel_start_time_min[1][0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 2 Time 2 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+21) = sel_start_time_min[1][1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 2 Time 3 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+22) = sel_start_time_min[1][2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 2 Time 4 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+23) = sel_start_time_min[1][3]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 3 Time 1 Start Time Min
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+24) = sel_start_time_min[2][0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 3 Time 2 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+25) = sel_start_time_min[2][1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 3 Time 3 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+26) = sel_start_time_min[2][2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 3 Time 4 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+27) = sel_start_time_min[2][3]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 4 Time 1 Start Time Min
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+28) = sel_start_time_min[3][0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 4 Time 2 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+29) = sel_start_time_min[3][1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 4 Time 3 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+30) = sel_start_time_min[3][2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 4 Time 4 Start Time Min
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+31) = sel_start_time_min[3][3]; /* Write in data EEPROM */
		__WFI();	
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 1 Time 1 Start Time Hour
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+32) = sel_start_time_hour[0][0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 1 Time 2 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+33) = sel_start_time_hour[0][1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 1 Time 3 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+34) = sel_start_time_hour[0][2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 1 Time 4 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+35) = sel_start_time_hour[0][3]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 2 Time 1 Start Time Hour
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+36) = sel_start_time_hour[1][0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 2 Time 2 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+37) = sel_start_time_hour[1][1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 2 Time 3 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+38) = sel_start_time_hour[1][2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 2 Time 4 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+39) = sel_start_time_hour[1][3]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 3 Time 1 Start Time Hour
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+40) = sel_start_time_hour[2][0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 3 Time 2 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+41) = sel_start_time_hour[2][1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 3 Time 3 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+42) = sel_start_time_hour[2][2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 3 Time 4 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+43) = sel_start_time_hour[2][3]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 1 Kaydet   -- Selenoid 4 Time 1 Start Time Hour
	//-------------- 	
		*(uint8_t *)(DATA_E2_ADDR+44) = sel_start_time_hour[3][0]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 2 Kaydet   -- Selenoid 4 Time 2 Start Time Houra
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+45) = sel_start_time_hour[3][1]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 3 Kaydet   -- Selenoid 4 Time 3 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+46) = sel_start_time_hour[3][2]; /* Write in data EEPROM */
		__WFI();
	//--------------  
	// DATA 4 Kaydet   -- Selenoid 4 Time 4 Start Time Hour
	//--------------  
		*(uint8_t *)(DATA_E2_ADDR+47) = sel_start_time_hour[3][3]; /* Write in data EEPROM */
		__WFI();	  
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------		
		LockNVM();
	}
}


/*******************************************************************************
 *  function :   READ FROM EEPROM
 ******************************************************************************/
/***/
__INLINE void read_from_EEPROM(void)
{
	if(fReadfromEEPROM)
	{	
		fReadfromEEPROM = 0;		
		UnlockPELOCK();
		FLASH->PECR |= FLASH_PECR_ERRIE | FLASH_PECR_EOPIE; /* enable flash interrupts */
		/* Check if the first address of the page is yet erased,this is only for this example */
	//--------------  
	//Selenoid selected work duration
	//--------------  
		sel_work_duration[0] = (*(uint8_t *)(DATA_E2_ADDR));
		sel_work_duration[1] = (*(uint8_t *)(DATA_E2_ADDR+1)); 
		sel_work_duration[2] = (*(uint8_t *)(DATA_E2_ADDR+2));
		sel_work_duration[3] = (*(uint8_t *)(DATA_E2_ADDR+3)); 
	//--------------  
	//--------------  
	//Selenoid aralikli sulama
	//--------------  
		sel_delayed_irr_val[0] = (*(uint8_t *)(DATA_E2_ADDR+4)); 
		sel_delayed_irr_val[1] = (*(uint8_t *)(DATA_E2_ADDR+5)); 
		sel_delayed_irr_val[2] = (*(uint8_t *)(DATA_E2_ADDR+6)); 
		sel_delayed_irr_val[3] = (*(uint8_t *)(DATA_E2_ADDR+7)); 
	//--------------  
	//--------------  
	//Selenoid manuel aralikli sulama
	//--------------  
		sel_manuel_work_duration[0] = (*(uint8_t *)(DATA_E2_ADDR+8)); 
		sel_manuel_work_duration[1] = (*(uint8_t *)(DATA_E2_ADDR+9)); 
		sel_manuel_work_duration[2] = (*(uint8_t *)(DATA_E2_ADDR+10)); 
		sel_manuel_work_duration[3] = (*(uint8_t *)(DATA_E2_ADDR+11)); 
	//--------------  	
	//--------------  
	//Selenoid özel sulama --> ilk deger 0x00 olsun  Bits --> ACTIVATED, Pt, Sa, Ca, Pr, Cu, Ct, Pz -->
	//--------------  
		sel_special_irr_days[0] = (*(uint8_t *)(DATA_E2_ADDR+12)); 
		sel_special_irr_days[1] = (*(uint8_t *)(DATA_E2_ADDR+13)); 
		sel_special_irr_days[2] = (*(uint8_t *)(DATA_E2_ADDR+14)); 
		sel_special_irr_days[3] = (*(uint8_t *)(DATA_E2_ADDR+15)); 
	//--------------  	
	//--------------  
	//Selenoid startbtime min --> Selenoid X start time min X
	//--------------  	
		sel_start_time_min[0][0] = (*(uint8_t *)(DATA_E2_ADDR+16)); 
		sel_start_time_min[0][1] = (*(uint8_t *)(DATA_E2_ADDR+17)); 
		sel_start_time_min[0][2] = (*(uint8_t *)(DATA_E2_ADDR+18)); 
		sel_start_time_min[0][3] = (*(uint8_t *)(DATA_E2_ADDR+19)); 
		sel_start_time_min[1][0] = (*(uint8_t *)(DATA_E2_ADDR+20)); 
		sel_start_time_min[1][1] = (*(uint8_t *)(DATA_E2_ADDR+21)); 
		sel_start_time_min[1][2] = (*(uint8_t *)(DATA_E2_ADDR+22)); 
		sel_start_time_min[1][3] = (*(uint8_t *)(DATA_E2_ADDR+23)); 
		sel_start_time_min[2][0] = (*(uint8_t *)(DATA_E2_ADDR+24)); 
		sel_start_time_min[2][1] = (*(uint8_t *)(DATA_E2_ADDR+25)); 
		sel_start_time_min[2][2] = (*(uint8_t *)(DATA_E2_ADDR+26)); 
		sel_start_time_min[2][3] = (*(uint8_t *)(DATA_E2_ADDR+27));
		sel_start_time_min[3][0] = (*(uint8_t *)(DATA_E2_ADDR+28)); 
		sel_start_time_min[3][1] = (*(uint8_t *)(DATA_E2_ADDR+29)); 
		sel_start_time_min[3][2] = (*(uint8_t *)(DATA_E2_ADDR+30)); 
		sel_start_time_min[3][3] = (*(uint8_t *)(DATA_E2_ADDR+31)); 	
	//--------------  	
	//--------------  
	//Selenoid start time hour --> Selenoid X start time hour X
	//--------------  	
		sel_start_time_hour[0][0] = (*(uint8_t *)(DATA_E2_ADDR+32)); 
		sel_start_time_hour[0][1] = (*(uint8_t *)(DATA_E2_ADDR+33)); 
		sel_start_time_hour[0][2] = (*(uint8_t *)(DATA_E2_ADDR+34)); 
		sel_start_time_hour[0][3] = (*(uint8_t *)(DATA_E2_ADDR+35)); 
		sel_start_time_hour[1][0] = (*(uint8_t *)(DATA_E2_ADDR+36)); 
		sel_start_time_hour[1][1] = (*(uint8_t *)(DATA_E2_ADDR+37)); 
		sel_start_time_hour[1][2] = (*(uint8_t *)(DATA_E2_ADDR+38)); 
		sel_start_time_hour[1][3] = (*(uint8_t *)(DATA_E2_ADDR+39)); 
		sel_start_time_hour[2][0] = (*(uint8_t *)(DATA_E2_ADDR+40)); 
		sel_start_time_hour[2][1] = (*(uint8_t *)(DATA_E2_ADDR+41)); 
		sel_start_time_hour[2][2] = (*(uint8_t *)(DATA_E2_ADDR+42)); 
		sel_start_time_hour[2][3] = (*(uint8_t *)(DATA_E2_ADDR+43));
		sel_start_time_hour[3][0] = (*(uint8_t *)(DATA_E2_ADDR+44)); 
		sel_start_time_hour[3][1] = (*(uint8_t *)(DATA_E2_ADDR+45)); 
		sel_start_time_hour[3][2] = (*(uint8_t *)(DATA_E2_ADDR+46)); 
		sel_start_time_hour[3][3] = (*(uint8_t *)(DATA_E2_ADDR+47));	
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------	
	//--------------  	
	//--------------  
	//Password and Password Status
	//--------------  
		password[0] = (*(uint8_t *)(DATA_E2_ADDR+48));
		password[1] = (*(uint8_t *)(DATA_E2_ADDR+49)); 
		password[2] = (*(uint8_t *)(DATA_E2_ADDR+50)); 
		password[3] = (*(uint8_t *)(DATA_E2_ADDR+51)); 
		fPasswordSetted = (*(uint8_t *)(DATA_E2_ADDR+52));	
		
		LockNVM();
	}
}

/*******************************************************************************
 *  function :   READ PASSWORD FROM EEPROM
 ******************************************************************************/
void updateInvalidPassWord(void)
{
	if(fUpdateInvPassCnt)
	{
		fUpdateInvPassCnt = 0;
		
		UnlockPELOCK();
		FLASH->PECR |= FLASH_PECR_ERRIE | FLASH_PECR_EOPIE; /* enable flash interrupts */

		*(uint8_t *)(DATA_E2_ADDR+53) = invalidPasswordCnt; /* Write in data EEPROM */
		__WFI();
	}
}
			



/*******************************************************************************
 *  function :   READ PASSWORD FROM EEPROM
 ******************************************************************************/
/***/
__INLINE void read_Password_from_EEPROM(void)
{
	//Password and Password Status
	//--------------  
		password[0] = (*(uint8_t *)(DATA_E2_ADDR+48));
		password[1] = (*(uint8_t *)(DATA_E2_ADDR+49)); 
		password[2] = (*(uint8_t *)(DATA_E2_ADDR+50)); 
		password[3] = (*(uint8_t *)(DATA_E2_ADDR+51)); 
		fPasswordSetted = (*(uint8_t *)(DATA_E2_ADDR+52));	
}

/******************************************************************************/
/*                 STM32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l0xx.s).                                               */
/******************************************************************************/

/**
  * Brief   This function handles FLASH interrupt request.
  *         It handles any kind of error even if not used in this example.
  * Param   None
  * Retval  None
  */
void FLASH_IRQHandler(void)
{
  if ((FLASH->SR & FLASH_SR_EOP) != 0)  /* (3) */
  {
    FLASH->SR = FLASH_SR_EOP; /* (4) */
  }
  /* Manage the error cases */
  else if ((FLASH->SR & FLASH_SR_FWWERR) != 0) /* Check Fetch while Write error */
  {
    error |= ERROR_FETCH_DURING_PROG; /* Report the error to the main progran */
    FLASH->SR = FLASH_SR_FWWERR; /* Clear the flag by software by writing it at 1*/
  }
  else if ((FLASH->SR & FLASH_SR_NOTZEROERR) != 0) /* Check Not Zero error */
  /* This error occurs if the address content was not cleared/erased 
     before the programming */
  {
    error |= ERROR_NOT_ZERO; /* Report the error to the main progran */
    FLASH->SR = FLASH_SR_NOTZEROERR; /* Clear the flag by software by writing it at 1*/
  }
  else if ((FLASH->SR & FLASH_SR_SIZERR) != 0) /* Check Size error */
  {
    error |= ERROR_SIZE; /* Report the error to the main progran */
    FLASH->SR = FLASH_SR_SIZERR; /* Clear the flag by software by writing it at 1*/
  }
  else if ((FLASH->SR & FLASH_SR_WRPERR) != 0) /* Check Write protection error */
  {
    error |= ERROR_WRITE_PROTECTION; /* Report the error to the main progran */
    FLASH->SR = FLASH_SR_WRPERR; /* Clear the flag by software by writing it at 1*/
  }
  else if ((FLASH->SR & FLASH_SR_RDERR) != 0) /* Check Read-out protection error */
  {
    error |= ERROR_READOUT_PROTECTION; /* Report the error to the main progran */
    FLASH->SR = FLASH_SR_RDERR; /* Clear the flag by software by writing it at 1*/
  }
  else if ((FLASH->SR & FLASH_SR_OPTVERR) != 0) /* Check Option valid error */
  {
    error |= ERROR_OPTION_NOT_VALID; /* Report the error to the main progran */
    FLASH->SR = FLASH_SR_OPTVERR; /* Clear the flag by software by writing it at 1*/
  }
  else if ((FLASH->SR & FLASH_SR_PGAERR) != 0) /* Check alignment error */
  {
    error |= ERROR_ALIGNMENT; /* Report the error to the main progran */
    FLASH->SR = FLASH_SR_PGAERR; /* Clear the flag by software by writing it at 1*/
  } 
  else
  {
    error |= ERROR_UNKNOWN; 
  }
}


/******************************************************************************/
/*                 ---------- PROGRAM CELENDAR --------------                  */
/******************************************************************************/
void updateRelayCelendar(uint8_t relayNumber)
{
	uint32_t minute = 0;
	uint32_t hour = 0;
	uint32_t today = 0;
	uint32_t dayOfWeek = 0;
	uint32_t tmpMin = 0;
	uint32_t tmpHour = 0;
	uint32_t RTC_PROGRAM_MASK = 0xFFFFFFFF;
	uint32_t RTC_START_TIME_MASK = 0xFFFFFFFF;
	uint32_t RTC_WORK_DURATION_MASK = 0xFFFFFFFF;	
	uint32_t IRRIGATE_NEXT_DATE_MASK = 0x00000000;	   // Sulama yeni güne sarkarsa diye sonradan eklendi.
	
	today = RTC->DR;
	
if(sel_delayed_irr_val[relayNumber-1] != 0) //delayed IRR programmed
{
	//delayed_irr_counter sayaci hergün bir arttirlir. sulama gününün katlarinda sulama yapar. örnegin 3 gün arayla sulanacaksa 0. gün, 3. gün 6.gün ...
	if ((delayed_irr_counter % sel_delayed_irr_val[relayNumber-1]) != 0) // arakli sulamanin oldugu katlardan birisiysek eger
	{
		RTC_PROGRAM_MASK = 0x00000000;
	}
	
}
else if(sel_special_irr_days[relayNumber-1] != 0) //specials days
{
	dayOfWeek = (today&RTC_DR_WDU)>>13;
	if(!((sel_special_irr_days[relayNumber-1]>>(dayOfWeek-1))&0x01))  // bugün sulama yoksa takvimi sifirla
	{
		RTC_PROGRAM_MASK = 0x00000000;
	}
} //else hergun çalisma aktif


if(sel_work_duration[relayNumber-1] == 0)
{
	RTC_WORK_DURATION_MASK = 0x00000000;
}



//Relay1_Time_1	
	minute=sel_start_time_min[relayNumber-1][0]+sel_work_duration[relayNumber-1];
	hour=sel_start_time_hour[relayNumber-1][0]+(minute/60);
	minute=minute%60;
	minute=bcd2cnv(minute);
	IRRIGATE_NEXT_DATE_MASK = 0x00000000;
	if(hour > 23)
	{
		IRRIGATE_NEXT_DATE_MASK = 0x01000000;	   // Sulama yeni güne sarkarsa siralamada sona kaysin diye sonradan eklendi.
	}
	hour=hour%24;
	hour=bcd2cnv(hour);
	tmpMin = bcd2cnv(sel_start_time_min[relayNumber-1][0]);
	tmpHour = bcd2cnv(sel_start_time_hour[relayNumber-1][0]);
	relayCelendar[8*(relayNumber-1)] = ((tmpMin << 8)| RTC_ALRMAR_MSK4)&RTC_PROGRAM_MASK&RTC_WORK_DURATION_MASK;   // aralikli sulama ve özel sulama seçilmisse 
	relayCelendar[8*(relayNumber-1)] |= (tmpHour << 16)&RTC_PROGRAM_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 1] = ((minute << 8)|RTC_ALRMAR_MSK4|IRRIGATE_NEXT_DATE_MASK)&RTC_PROGRAM_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 1] |= ((hour << 16)|IRRIGATE_NEXT_DATE_MASK)&RTC_PROGRAM_MASK&RTC_WORK_DURATION_MASK;
	
//Relay1_Time_2	
	if((sel_start_time_min[relayNumber-1][1] == 0xFF) | (sel_start_time_hour[relayNumber-1][1] == 0xFF))
	{
		RTC_START_TIME_MASK = 0x00000000;
	}
	else
	{
		RTC_START_TIME_MASK = 0xFFFFFFFF;
		minute=sel_start_time_min[relayNumber-1][1]+sel_work_duration[relayNumber-1];
		hour=sel_start_time_hour[relayNumber-1][1]+(minute/60);
		minute=minute%60;
		minute=bcd2cnv(minute);
		IRRIGATE_NEXT_DATE_MASK = 0x00000000;
		if(hour > 23)
		{
			IRRIGATE_NEXT_DATE_MASK = 0x01000000;	   // Sulama yeni güne sarkarsa siralamada sona kaysin diye sonradan eklendi.
		}
		hour=hour%24;
		hour=bcd2cnv(hour);
		tmpMin = bcd2cnv(sel_start_time_min[relayNumber-1][1]);
		tmpHour = bcd2cnv(sel_start_time_hour[relayNumber-1][1]);
	}
	relayCelendar[8*(relayNumber-1) + 2] = ((tmpMin << 8)| RTC_ALRMAR_MSK4)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 2] |= (tmpHour << 16)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 3] =  ((minute << 8)|RTC_ALRMAR_MSK4|IRRIGATE_NEXT_DATE_MASK)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 3] |=  ((hour << 16)|IRRIGATE_NEXT_DATE_MASK)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;

//Relay1_Time_3	
	if((sel_start_time_min[relayNumber-1][2] == 0xFF) | (sel_start_time_hour[relayNumber-1][2] == 0xFF))
	{
		RTC_START_TIME_MASK = 0x00000000;
	}
	else
	{
		RTC_START_TIME_MASK = 0xFFFFFFFF;
		minute=sel_start_time_min[relayNumber-1][2]+sel_work_duration[relayNumber-1];
		hour=sel_start_time_hour[relayNumber-1][2]+(minute/60);
		minute=minute%60;
		minute=bcd2cnv(minute);
		IRRIGATE_NEXT_DATE_MASK = 0x00000000;
		if(hour > 23)
		{
			IRRIGATE_NEXT_DATE_MASK = 0x01000000;	   // Sulama yeni güne sarkarsa siralamada sona kaysin diye sonradan eklendi.
		}
		hour=hour%24;
		hour=bcd2cnv(hour);
		tmpMin = bcd2cnv(sel_start_time_min[relayNumber-1][2]);
		tmpHour = bcd2cnv(sel_start_time_hour[relayNumber-1][2]);	
	}
	relayCelendar[8*(relayNumber-1) + 4] = ((tmpMin << 8)| RTC_ALRMAR_MSK4)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 4] |= (tmpHour << 16)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 5] =  ((minute << 8)|RTC_ALRMAR_MSK4|IRRIGATE_NEXT_DATE_MASK)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 5] |=  ((hour << 16)|IRRIGATE_NEXT_DATE_MASK)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;

//Relay1_Time_4	
	if((sel_start_time_min[relayNumber-1][3] == 0xFF) | (sel_start_time_hour[relayNumber-1][3] == 0xFF))
	{
		RTC_START_TIME_MASK = 0x00000000;
	}
	else
	{
		RTC_START_TIME_MASK = 0xFFFFFFFF;
		minute=sel_start_time_min[relayNumber-1][3]+sel_work_duration[relayNumber-1];
		hour=sel_start_time_hour[relayNumber-1][3]+(minute/60);
		minute=minute%60;
		minute=bcd2cnv(minute);
		IRRIGATE_NEXT_DATE_MASK = 0x00000000;
		if(hour > 23)
		{
			IRRIGATE_NEXT_DATE_MASK = 0x01000000;	   // Sulama yeni güne sarkarsa siralamada sona kaysin diye sonradan eklendi.
		}
		hour=hour%24;
		hour=bcd2cnv(hour);
		tmpMin = bcd2cnv(sel_start_time_min[relayNumber-1][3]);
		tmpHour = bcd2cnv(sel_start_time_hour[relayNumber-1][3]);	
	}
	relayCelendar[8*(relayNumber-1) + 6] = ((tmpMin << 8)| RTC_ALRMAR_MSK4)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 6] |= (tmpHour << 16)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 7] =  ((minute << 8)|RTC_ALRMAR_MSK4|IRRIGATE_NEXT_DATE_MASK)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
	relayCelendar[8*(relayNumber-1) + 7] |=  ((hour << 16)|IRRIGATE_NEXT_DATE_MASK)&RTC_PROGRAM_MASK&RTC_START_TIME_MASK&RTC_WORK_DURATION_MASK;
}



/******************************************************************************/
/*                 ---------- MANUEL WORK CELENDAR --------------                  */
/******************************************************************************/

void manuelAllWorkCelender(void)
{
	
	uint32_t currentTime = 0;
	uint8_t workingDelay = 0;
	currentTime = RTC->TR;
	uint32_t tmpSecUnits = 0;
	uint32_t tmpSecTens = 0;	
	uint32_t tmpMinUnits = 0;
	uint32_t tmpMinTens = 0;
	uint32_t tmpHourUnits = 0;
	uint32_t tmpHourTens = 0;	
	uint32_t tmpHour = 0;
	uint32_t tmpMin= 0;		
	uint32_t tmpSec= 0;
	
	for(int i= 0; i<32; i++)   // manuel kuruldugunda önce çalisan programi iptal et ve temizle
	{
		relayCelendar[i] =0;
		pendingRtcIrq[i] =0;
	}
	
	if(fRelay1Status)
	{
		fCloseRelay1 = 1;
	}
	if(fRelay2Status)
	{
		fCloseRelay2 = 1; // manuel kuruldugunda açik bir röle varsa kapat
	}
	if(fRelay3Status)
	{
		fCloseRelay3 = 1;
	}
	if(fRelay4Status)
	{
		fCloseRelay4 = 1;
	}	
	
	workingDelay = all_manuel_work_duration;

	tmpHourTens = (currentTime & RTC_TR_HT)>>20;
	tmpHourUnits = (currentTime & RTC_TR_HU)>>16;
	tmpMinTens = (currentTime & RTC_TR_MNT)>>12;
	tmpMinUnits = (currentTime & RTC_TR_MNU)>>8;
	tmpSecTens = (currentTime & RTC_TR_ST)>>4;
	tmpSecUnits =(currentTime & RTC_TR_SU);

	if( SELENOID_NUMBER  >= 1)    // NUMBERS OF SELENOID  >= 1
	{
		//röle 1	
		tmpSec = tmpSecTens *10 + tmpSecUnits + 2; 		// 2 sec delay
		tmpMin = tmpMinTens * 10 + tmpMinUnits + (tmpSec/60);
		tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60);
		tmpSec=tmpSec%60;
		tmpSec=bcd2cnv(tmpSec);		
		tmpMin=tmpMin%60;
		tmpMin=bcd2cnv(tmpMin);
		tmpHour=tmpHour%24;
		tmpHour=bcd2cnv(tmpHour);	
		pendingRtcIrq[24] = (0x80000000 |(tmpHour<<16) | (tmpMin<<8)  | (tmpSec)); // 3 saniye sonrasina kurun
		pendingRelay[24] = 1;
		tmpSec = tmpSecTens *10 + tmpSecUnits;
		tmpMin = tmpMinTens * 10 + tmpMinUnits + workingDelay + (tmpSec/60);
		tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
		tmpSec=tmpSec%60;
		tmpSec=bcd2cnv(tmpSec);	
		tmpMin=tmpMin%60;
		tmpMin=bcd2cnv(tmpMin);
		tmpHour=tmpHour%24;
		tmpHour=bcd2cnv(tmpHour);
		pendingRtcIrq[25] = (0x80000000 |(tmpHour<<16) | (tmpMin<<8)  | (tmpSec)); // kapat
		pendingRelay[25] = 1;
	}

	if( SELENOID_NUMBER  >= 2)    // NUMBERS OF SELENOID  >= 2
	{
		//röle 2
		tmpSec = tmpSecTens *10 + tmpSecUnits + 2; 		// 2 sec delay
		tmpMin = tmpMinTens * 10 + tmpMinUnits + workingDelay + (tmpSec/60);
		tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
		tmpSec=tmpSec%60;
		tmpSec=bcd2cnv(tmpSec);	
		tmpMin=tmpMin%60;
		tmpMin=bcd2cnv(tmpMin);
		tmpHour=tmpHour%24;
		tmpHour=bcd2cnv(tmpHour);
		pendingRtcIrq[26] = (0x80000000 |(tmpHour<<16) | (tmpMin<<8)  | (tmpSec)); // 3 saniye sonrasina kurun	
		pendingRelay[26] = 2;
		tmpSec = tmpSecTens *10 + tmpSecUnits;
		tmpMin = tmpMinTens * 10 + tmpMinUnits + 2*workingDelay + (tmpSec/60);
		tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
		tmpSec=tmpSec%60;
		tmpSec=bcd2cnv(tmpSec);	
		tmpMin=tmpMin%60;
		tmpMin=bcd2cnv(tmpMin);
		tmpHour=tmpHour%24;
		tmpHour=bcd2cnv(tmpHour);
		pendingRtcIrq[27] = (0x80000000 |(tmpHour<<16) | (tmpMin<<8)  | (tmpSec)); // kapat
		pendingRelay[27] = 2;
	}

	if( SELENOID_NUMBER  >= 3)    // NUMBERS OF SELENOID  >= 3
	{
		//röle 3
		tmpSec = tmpSecTens *10 + tmpSecUnits + 2; 		// 2 sec delay
		tmpMin = tmpMinTens * 10 + tmpMinUnits + 2*workingDelay + (tmpSec/60);
		tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
		tmpSec=tmpSec%60;
		tmpSec=bcd2cnv(tmpSec);	
		tmpMin=tmpMin%60;
		tmpMin=bcd2cnv(tmpMin);
		tmpHour=tmpHour%24;
		tmpHour=bcd2cnv(tmpHour);	
		pendingRtcIrq[28] = (0x80000000 |(tmpHour<<16) | (tmpMin<<8)  | (tmpSec)); // 3 saniye sonrasina kurun	
		pendingRelay[28] = 3;
		tmpSec = tmpSecTens *10 + tmpSecUnits;
		tmpMin = tmpMinTens * 10 + tmpMinUnits + 3*workingDelay + (tmpSec/60);
		tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
		tmpSec=tmpSec%60;
		tmpSec=bcd2cnv(tmpSec);	
		tmpMin=tmpMin%60;
		tmpMin=bcd2cnv(tmpMin);
		tmpHour=tmpHour%24;
		tmpHour=bcd2cnv(tmpHour);
		pendingRtcIrq[29] = (0x80000000 |(tmpHour<<16) | (tmpMin<<8)  | (tmpSec)); // kapat
		pendingRelay[29] = 3;
	}	

	if( SELENOID_NUMBER  >= 4)    // NUMBERS OF SELENOID  >= 4
	{
		//röle 4
		tmpSec = tmpSecTens *10 + tmpSecUnits + 2; 		// 2 sec delay
		tmpMin = tmpMinTens * 10 + tmpMinUnits + 3*workingDelay + (tmpSec/60);
		tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
		tmpSec=tmpSec%60;
		tmpSec=bcd2cnv(tmpSec);	
		tmpMin=tmpMin%60;
		tmpMin=bcd2cnv(tmpMin);
		tmpHour=tmpHour%24;
		tmpHour=bcd2cnv(tmpHour);	
		pendingRtcIrq[30] = (0x80000000 |(tmpHour<<16) | (tmpMin<<8)  | (tmpSec)); // 3 saniye sonrasina kurun	
		pendingRelay[30] = 4;
		tmpSec = tmpSecTens *10 + tmpSecUnits;
		tmpMin = tmpMinTens * 10 + tmpMinUnits + 4*workingDelay + (tmpSec/60);
		tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
		tmpSec=tmpSec%60;
		tmpSec=bcd2cnv(tmpSec);	
		tmpMin=tmpMin%60;
		tmpMin=bcd2cnv(tmpMin);
		tmpHour=tmpHour%24;
		tmpHour=bcd2cnv(tmpHour);
		pendingRtcIrq[31] = (0x80000000 |(tmpHour<<16) | (tmpMin<<8)  | (tmpSec)); // kapat
		pendingRelay[31] = 4;
	}	
}

/******************************************************************************/
/*                 ---------- MANUEL WORK CELENDAR --------------                  */
/******************************************************************************/

void manuelWorkCelender(void)
{
	
	uint32_t currentTime = 0;
	uint32_t workingDelay = 0;
	currentTime = RTC->TR;
	uint32_t tmpSecUnits = 0;
	uint32_t tmpSecTens = 0;	
	uint32_t tmpMinUnits = 0;
	uint32_t tmpMinTens = 0;
	uint32_t tmpHourUnits = 0;
	uint32_t tmpHourTens = 0;	
	uint32_t tmpHour = 0;
	uint32_t tmpMin= 0;		
	uint32_t tmpSec= 0;
	
	for(int i= 0; i<32; i++)   // manuel kuruldugunda önce çalisan programi iptal et ve temizle
	{
		relayCelendar[i] =0;
		pendingRtcIrq[i] =0;
	}
	
	if(fRelay1Status)
	{
		fCloseRelay1 = 1;
	}
	if(fRelay2Status)
	{
		fCloseRelay2 = 1; // manuel kuruldugunda açik bir röle varsa kapat
	}
	if(fRelay3Status)
	{
		fCloseRelay3 = 1;
	}
	if(fRelay4Status)
	{
		fCloseRelay4 = 1;
	}	
	
	workingDelay = sel_manuel_work_duration[selectedSelenoid-1];
	
	tmpHourTens = (currentTime & RTC_TR_HT)>>20;
	tmpHourUnits = (currentTime & RTC_TR_HU)>>16;
	tmpMinTens = (currentTime & RTC_TR_MNT)>>12;
	tmpMinUnits = (currentTime & RTC_TR_MNU)>>8;
	tmpSecTens = (currentTime & RTC_TR_ST)>>4;
	tmpSecUnits =(currentTime & RTC_TR_SU);
	

	tmpSec = tmpSecTens *10 + tmpSecUnits;
	tmpMin = tmpMinTens * 10 + tmpMinUnits + workingDelay + (tmpSec/60);
	tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
	
	tmpSec=tmpSec%60;
	tmpSec=bcd2cnv(tmpSec);			
	tmpMin=tmpMin%60;
	tmpMin=bcd2cnv(tmpMin);
	tmpHour=tmpHour%24;
	tmpHour=bcd2cnv(tmpHour);
	configureAlarmA(0x80000000 | (tmpHour<<16) | (tmpMin<<8)  | (tmpSec));
	if(selectedSelenoid == 1)
	{
		fOpenRelay_1_WithDelay = 1;
		relayOpenDelayMs = 2000;
	}
	else if(selectedSelenoid == 2)
	{
		fOpenRelay_2_WithDelay = 1;
		relayOpenDelayMs = 2000;
	}
	else if(selectedSelenoid == 3)
	{
		fOpenRelay_3_WithDelay = 1;
		relayOpenDelayMs = 2000;
	}
	else if(selectedSelenoid == 4)
	{
		fOpenRelay_4_WithDelay = 1;
		relayOpenDelayMs = 2000;
	}
}

/******************************************************************************/
/*                 ---------- PROGRAM CELENDAR --------------                  */
/******************************************************************************/

void programCelendar(void)
{
	updateRelayCelendar(1); // update Relay 1 celender
	updateRelayCelendar(2); // update Relay 2 celender
	updateRelayCelendar(3); // update Relay 3 celender
	updateRelayCelendar(4); // update Relay 4 celender
}


/******************************************************************************/
/*                 ---------- UPDATE RTC ALARM --------------                 */
/******************************************************************************/
uint8_t updateRTCAlarm(void)
{
	uint32_t currentTime = 0;
	uint32_t delayedpendingIRQTime = 0;
	uint8_t prgNum = 0;
	uint32_t workingDelay = 0;
	currentTime = RTC->TR;
	uint32_t tmpSecUnits = 0;
	uint32_t tmpSecTens = 0;		
	uint32_t tmpMinUnits = 0;
	uint32_t tmpMinTens = 0;
	uint32_t tmpHourUnits = 0;
	uint32_t tmpHourTens = 0;	
	uint32_t tmpHour = 0;
	uint32_t tmpMin= 0;		
	uint32_t tmpSec= 0;
	uint32_t endTimeHourDelay = 0;
	
	while(pendingRtcIrq[prgNum] == 0)
	{
		++prgNum;
		if(prgNum == 32)
		{
			break;
		}
	}
	
	if(prgNum == 32)
	{
		return prgNum;
	}
	else
	{
		if(currentTime > (pendingRtcIrq[prgNum]&0x7FFFFFFF))
		{
			if((currentTime & (RTC_TR_HT|RTC_TR_HU)) == ((pendingRtcIrq[prgNum]&0x7FFFFFFF) & (RTC_TR_HT|RTC_TR_HU)))
			{
				workingDelay = ((currentTime&RTC_TR_MNT)>>12)*10 + ((currentTime&RTC_TR_MNU)>>8) - ((pendingRtcIrq[prgNum]&(RTC_TR_MNT))>>12)*10 - ((pendingRtcIrq[prgNum]&RTC_TR_MNU)>>8);
			}
			else
			{
				endTimeHourDelay = ((currentTime & RTC_TR_HT)>>20)*10 + ((currentTime & RTC_TR_HU)>>16) - ((((pendingRtcIrq[prgNum]&0x7FFFFFFF) & RTC_TR_HT)>>20)*10 + (((pendingRtcIrq[prgNum]&0x7FFFFFFF) & RTC_TR_HU)>>16)) ;
				if( endTimeHourDelay == 1) // saat farki 1
				{
					workingDelay = 60  + ((currentTime&(RTC_TR_MNT))>>12)*10 + ((currentTime&(RTC_TR_MNU))>>8) - ((((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNT)>>12)*10 + (((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNU)>>8));
				}
				else if( endTimeHourDelay == 2)  // saat farki 2
				{
					workingDelay = 120  + (((currentTime&(RTC_TR_MNT))>>12)*10 + ((currentTime&(RTC_TR_MNU))>>8)) - ((((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNT)>>12)*10 + (((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNU)>>8));
				}		
				else if( endTimeHourDelay == 3)  // saat farki 3
				{
					workingDelay = 180  + (((currentTime&(RTC_TR_MNT))>>12)*10 + ((currentTime&(RTC_TR_MNU))>>8)) - ((((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNT)>>12)*10 + (((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNU)>>8));
				}				
				else if( endTimeHourDelay == 4)  // saat farki 4
				{
					workingDelay = 240  + (((currentTime&(RTC_TR_MNT))>>12)*10 + ((currentTime&(RTC_TR_MNU))>>8)) - ((((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNT)>>12)*10 + (((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNU)>>8));
				}				
				else if( endTimeHourDelay == 5)  // saat farki 5
				{
					workingDelay = 300  + (((currentTime&(RTC_TR_MNT))>>12)*10 + ((currentTime&(RTC_TR_MNU))>>8)) - ((((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNT)>>12)*10 + (((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNU)>>8));
				}				
				else if( endTimeHourDelay == 6)  // saat farki 6
				{
					workingDelay = 360  + (((currentTime&(RTC_TR_MNT))>>12)*10 + ((currentTime&(RTC_TR_MNU))>>8)) - ((((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNT)>>12)*10 + (((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNU)>>8));
				}				
				else if( endTimeHourDelay == 7)  // saat farki 7
				{
					workingDelay = 420  + (((currentTime&(RTC_TR_MNT))>>12)*10 + ((currentTime&(RTC_TR_MNU))>>8)) - ((((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNT)>>12)*10 + (((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNU)>>8));
				}		
				else  // saat farki > 8 // mümkün degil !!!!
				{
					workingDelay = 480  + (((currentTime&(RTC_TR_MNT))>>12)*10 + ((currentTime&(RTC_TR_MNU))>>8)) - ((((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNT)>>12)*10 + (((pendingRtcIrq[prgNum]&0x7FFFFFFF)&RTC_TR_MNU)>>8));
				}						
			}
			tmpHourTens = (currentTime & RTC_TR_HT)>>20;
			tmpHourUnits = (currentTime & RTC_TR_HU)>>16;
			tmpMinTens = (currentTime & RTC_TR_MNT)>>12;
			tmpMinUnits = (currentTime & RTC_TR_MNU)>>8;
			tmpSecTens = (currentTime & RTC_TR_ST)>>4;
			tmpSecUnits =(currentTime & RTC_TR_SU);
			

			tmpSec = tmpSecTens *10 + tmpSecUnits + 2;  //2 sec delay;
			tmpMin = tmpMinTens * 10 + tmpMinUnits + (tmpSec/60);
			tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
			
			tmpSec=tmpSec%60;
			tmpSec=bcd2cnv(tmpSec);			
			tmpMin=tmpMin%60;
			tmpMin=bcd2cnv(tmpMin);
			tmpHour=tmpHour%24;
			tmpHour=bcd2cnv(tmpHour);
	
			configureAlarmA(0x80000000 | (tmpHour<<16) | (tmpMin<<8)  | (tmpSec));			// gecikme oldugunda 2 saniye sonrasina eski sulamayi kur
			
			if(prgNum < 31) //31 den büyük olamaz zaten normalde (31 iken son 4. rölede bazen problem oldugu söylendi, tekrar bak bu kisma ??? )
			{
				delayedpendingIRQTime = pendingRtcIrq[prgNum + 1]&0x7FFFFFFF;
				tmpHourTens = (delayedpendingIRQTime & RTC_TR_HT)>>20;
				tmpHourUnits = (delayedpendingIRQTime & RTC_TR_HU)>>16;
				tmpMinTens = (delayedpendingIRQTime & RTC_TR_MNT)>>12;
				tmpMinUnits = (delayedpendingIRQTime & RTC_TR_MNU)>>8;
				tmpSecTens = (currentTime & RTC_TR_ST)>>4;
				tmpSecUnits =(currentTime & RTC_TR_SU);			
				 
				tmpSec = tmpSecTens *10 + tmpSecUnits + 2; // 2 seconds delay 
				tmpMin = tmpMinTens * 10 + tmpMinUnits + workingDelay + (tmpSec/60); ;
				tmpHour = tmpHourTens * 10 + tmpHourUnits + (tmpMin/60) ;
				
				tmpSec=tmpSec%60;
				tmpSec=bcd2cnv(tmpSec);		
				tmpMin=tmpMin%60;
				tmpMin=bcd2cnv(tmpMin);
				tmpHour=tmpHour%24;
				tmpHour=bcd2cnv(tmpHour);
				pendingRtcIrq[prgNum+1] = 0x80000000;
				pendingRtcIrq[prgNum+1] = (pendingRtcIrq[prgNum+1] | (tmpHour<<16)) | (tmpMin<<8) | (tmpSec);
			}
		}
		else
		{
			configureAlarmA(pendingRtcIrq[prgNum]);
		}
		
		pendingRtcIrq[prgNum] = 0; // alarm kurulduysa siralamadan sil
		
		return prgNum;
	}
}

void resetPendingRelay(void)
{
	pendingRelay[0] = 1;
	pendingRelay[1] = 1;
	pendingRelay[2] = 1;
	pendingRelay[3] = 1;
	pendingRelay[4] = 1;
	pendingRelay[5] = 1;
	pendingRelay[6] = 1;
	pendingRelay[7] = 1;
	pendingRelay[8] = 2;
	pendingRelay[9] = 2;
	pendingRelay[10] = 2;
	pendingRelay[11] = 2;
	pendingRelay[12] = 2;
	pendingRelay[13] = 2;
	pendingRelay[14] = 2;
	pendingRelay[15] = 2;
	pendingRelay[16] = 3;
	pendingRelay[17] = 3;
	pendingRelay[18] = 3;
	pendingRelay[19] = 3;
	pendingRelay[20] = 3;
	pendingRelay[21] = 3;
	pendingRelay[22] = 3;
	pendingRelay[23] = 3;
	pendingRelay[24] = 4;
	pendingRelay[25] = 4;
	pendingRelay[26] = 4;
	pendingRelay[27] = 4;
	pendingRelay[28] = 4;
	pendingRelay[29] = 4;
	pendingRelay[30] = 4;
	pendingRelay[31] = 4;
}


/******************************************************************************/
/*             ---------- SORT PENDING IRRIGATIONS --------------             */
/******************************************************************************/
void celenderSort(void)
{
	int i, j, temp, temp2 = 0;
	uint8_t array_size = 32;
	
	for (i = 0; i < (array_size); ++i)
	{
		pendingRtcIrq[i] = relayCelendar[i];
	}
	
	for (i = 0; i < (array_size); i=i+2)
	{
		for (j = 0; j < (array_size-2); j=j+2)
		{
			if (pendingRtcIrq[j] > pendingRtcIrq[j + 2])
			{
				temp = pendingRtcIrq[j];
				pendingRtcIrq[j] = pendingRtcIrq[j + 2];
				pendingRtcIrq[j + 2] = temp;
				temp = pendingRtcIrq[j+1];
				pendingRtcIrq[j+1] = pendingRtcIrq[j+3];
				pendingRtcIrq[j+3] = temp;
				
				temp2 = pendingRelay[j];
				pendingRelay[j] = pendingRelay[j + 2];
				pendingRelay[j + 2] = temp2;	
				temp2 = pendingRelay[j+1];
				pendingRelay[j+1] = pendingRelay[j+3];
				pendingRelay[j+3] = temp2;						
			}
		}
	}
}

/******************************************************************************/
/*           --------- UPDATE SELENOID POSITION IRRIGATIONS --------          */
/******************************************************************************/
void updateSelenoidPosition(uint8_t prgNum)
{
	if(prgNum == 32)
	{
		if(fRelay1Status)
		{
			fCloseRelay1 = 1;
		}
		if(fRelay2Status)
		{
			fCloseRelay2 = 1;
		}
		if(fRelay3Status)
		{
			fCloseRelay3 = 1;
		}
		if(fRelay4Status)
		{
			fCloseRelay4 = 1;
		}
		
		if((fStartManuelWork)||(fStartAllRelayManuelWork))
		{
			fUpdateRtcAlarm = 1; // manuel sulama bittinde tekrar planli sulamalari yap
			finitRtcAlarm = 1; // planlama yaptiktan sonra zamani geçmis sulamalari iptal et
			fStartManuelWork = 0;
			fStartAllRelayManuelWork = 0;
			startManuelWorkDelay = 0;
			fManuelRelayAdjust = 0;// gerek yok 
			fmanuelIrrigationAdjust = 0;  
			fManuelWorkDurationAdjust = 0;// gerek yok
			fManuelAllRelayWorkDurAdj = 0;
			fhomePage = 1;
			if(fPasswordSetted)
			{
				fhomePage = 0;
				fHomePageWithPassword = 1;
				fPasswordDigit1Adjust = 1;
				fDeviceLockedWithPassword = 1;
			}
		}
		
		if(fcelenderPlanDelayed) // ertesine güne sarkan sulama oldugunda gecee 00:00 da degil son sulama bittiginde plan yapilsin.
		{
			fcelenderPlanDelayed = 0;
			fUpdateRtcAlarm = 1;
			finitRtcAlarm = 1;	
		}
	}
	else
	{
		if(prgNum%2)  // program number tek sayi
		{
			//open current Relay
			if(pendingRelay[prgNum-1] == 1)
			{
				fOpenRelay1 = 1;
			}
			else if(pendingRelay[prgNum-1] == 2)
			{
				fOpenRelay2 = 1;
			}
			else if(pendingRelay[prgNum-1] == 3)
			{
				fOpenRelay3 = 1;
			}
			else if(pendingRelay[prgNum-1] == 4)
			{
				fOpenRelay4 = 1;
			}
		}
		else if(prgNum) // program number sifir degilse ve program number çift ise
		{
			//close current Relay
			if(pendingRelay[prgNum-1] == 1)
			{
				fCloseRelay1 = 1;
			}
			else if(pendingRelay[prgNum-1] == 2)
			{
				fCloseRelay2 = 1;
			}
			else if(pendingRelay[prgNum-1] == 3)
			{
				fCloseRelay3 = 1;
			}
			else if(pendingRelay[prgNum-1] == 4)
			{
				fCloseRelay4 = 1;
			}			
		}
	}
}

/******************************************************************************/
/*           ------------ CANCEL PASSED IRRIGATIONS ------------              */
/******************************************************************************/
void cancelPrevIrr(void)
{
	uint8_t i = 0;
	uint32_t currentTime = 0;
	currentTime = RTC->TR;
	
	for(i =0;i<32;i=i+2)
	{
		if(currentTime >= (pendingRtcIrq[i]& 0x7FFFFFFF))
		{
			pendingRtcIrq[i] = 0;
			pendingRtcIrq[i+1] = 0;
		}
	}
}


void myWatchDogBOMB(void)
{	
	uint8_t resetDate1 = 0;
	uint8_t resetDate2 = 0;
	uint8_t resetDate3 = 0;
	uint8_t resetDate4 = 0;
	uint8_t resetTime1 = 0;
	uint8_t resetTime2 = 0;
	uint8_t resetTime3 = 0;
	uint8_t resetTime4 = 0;
	resetDate1 = ((RTC->DR)&0x000000FF);	
	resetDate2 = ((RTC->DR)&0x0000FF00)>>8;	
	resetDate3 = ((RTC->DR)&0x00FF0000)>>16;		
	resetDate4 = ((RTC->DR)&0xFF000000)>>24;		
	resetTime1 = ((RTC->DR)&0x000000FF);		
	resetTime2 = ((RTC->TR)&0x0000FF00)>>8;	
	resetTime3 = ((RTC->TR)&0x00FF0000)>>16;	
	resetTime4 = ((RTC->TR)&0xFF000000)>>24;	
	UnlockPELOCK();
	FLASH->PECR |= FLASH_PECR_ERRIE | FLASH_PECR_EOPIE; /* enable flash interrupts */
		*(uint8_t *)(DATA_E2_ADDR+55) = resetDate1;
	__WFI();
		*(uint8_t *)(DATA_E2_ADDR+56) = resetDate2;
	__WFI();
		*(uint8_t *)(DATA_E2_ADDR+57) = resetDate3;
	__WFI();	
		*(uint8_t *)(DATA_E2_ADDR+58) = resetDate4;
	__WFI();
		*(uint8_t *)(DATA_E2_ADDR+59) = resetTime1;
	__WFI();
		*(uint8_t *)(DATA_E2_ADDR+60) = resetTime2;
	__WFI();		
		*(uint8_t *)(DATA_E2_ADDR+61) = resetTime3;
	__WFI();
		*(uint8_t *)(DATA_E2_ADDR+62) = resetTime4;
	__WFI();		
	LockNVM();
}
