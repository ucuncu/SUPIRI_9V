/*------------------------------------------------------------------------------------------------------
 * Name:    I2C.c
 * Purpose: Initializes and reads and writes to I2C
 * Date: 		6/18/15
 * Author:	Christopher Jordan - Denny
 *------------------------------------------------------------------------------------------------------
 * Note(s): The read and write sequence is specific to the ISK01A1, so these functions may not work
						for a different Devices I2C.
 *----------------------------------------------------------------------------------------------------*/

/*-------------------------------------------Include Statements---------------------------------------*/
#include "stm32l053xx.h"                  // Specific Device header
#include "I2C.h"
#include "interrupt.h"
#include "systick.h"
#include "nxp_lcd_driver.h"
/*-------------------------------------------Global Variables-----------------------------------------*/
uint32_t I2C1_RX_Data = 0;
/*-------------------------------------------Functions------------------------------------------------*/

void I2C_Init(void){
  /* Enable the peripheral clock of GPIOB */
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

	int SCL = 6;							//SCL pin on PORTB alt fnc 4
	int SDA = 7;							//SDA pin on PORTB alt fnc 4
	
	/*Enable Clock for I2C*/
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	
	I2C1->CR1 |= (1<<8);																								/*(1)*/
	//GPIOB->MODER|= GPIO_MODER_MODE6|GPIO_MODER_MODE7;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7; /* (2) */
	GPIOB->MODER = ~((~GPIOB->MODER) | ((1 << 2*SCL) + (1 << 2*SDA)));	/*(2)*/
	GPIOB->AFR[0] = 0x11000000;																					/*(3)*/
	GPIOB->OSPEEDR|=GPIO_OSPEEDER_OSPEED6|GPIO_OSPEEDER_OSPEED7;//predkosc 50MHZ	

	
	I2C1->CR1|=I2C_CR1_PE;                    //set PE
  I2C1->CR1&=~I2C_CR1_PE;                    //reset PE	
	long unsigned int timeOut = 0;
	while(I2C1->CR1&I2C_CR1_PE)		//while PE ==1
	{
		++timeOut;
		if(timeOut>timeOutVal)
		{
			break;
		}
	}
	//I2C1->TIMINGR = (uint32_t)0x00503D5A;	  //*	Standard Mode @100kHz with I2CCLK = 16MHz, rise time = 100ns, fall time = 10ns.(1)
	I2C1->TIMINGR = (uint32_t)0x00300619;   // 400kHZ
	I2C1->CR1 |= (I2C_CR1_PE);							//set PE
	timeOut = 0;
	while(!(I2C1->CR1&I2C_CR1_PE))            //while PE ==1
	{
		++timeOut;
		if(timeOut>timeOutVal)
		{
			break;
		}
	}
	//Set CR2 for 2-byte transfer for Device
	I2C1->CR2 |= (0x70<<0);  //  slave address
	
	I2C1->CR2 &=~ I2C_CR2_RD_WRN;                        //write
		
	/* Don't forget to also enable which interrupts you want in CR1 */
	NVIC_EnableIRQ(I2C1_IRQn);
	NVIC_SetPriority(I2C1_IRQn,0);	

}



/**
  \fn				void Reset_I2C(void)
  \brief		I2C Reset, clears and then sets I2C_CR1_PE
*/

void Reset_I2C(void){
	int x = 0;		//1 to set bit, 0 to clear bit
	
	I2C1->CR1 ^= (-x ^ I2C1->CR1) & I2C_CR1_PE;		//Clear bit for reset
	I2C1->CR1 |= I2C_CR1_PE;
}



void I2C_Write(uint32_t Register, uint32_t Data)
{
	//Start communication
	I2C1->CR2 |= I2C_CR2_START |  (2 << 16);
	long unsigned int timeOut = 0;
	while(I2C1->CR2 & I2C_CR2_START)
	{
		++timeOut;
		if(timeOut>timeOutVal)
		{
			Reset_I2C();
			break;
		}
	}
	
	//Check Tx empty before writing to it
	if((I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE)){
		I2C1->TXDR = Register;
	}

		//Wait for transfer to complete
	 timeOut = 0;
	while((I2C1->ISR & I2C_ISR_TXE) == 0)
	{
		++timeOut;
		if(timeOut>timeOutVal)
		{
			Reset_I2C();
			break;
		}
	}
	
		//Check Tx empty before writing to it
	if((I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE)){
		I2C1->TXDR = Data;
	}
	//Wait for transfer to complete
	timeOut = 0;
	while((I2C1->ISR & I2C_ISR_TXE) == 0)
	{
		++timeOut;
		if(timeOut>timeOutVal)
		{
			Reset_I2C();
			break;
		}
	}
	//Wait for transfer to complete
	
	
	//Send Stop Condition
	I2C1->CR2 |= I2C_CR2_STOP;	
	
	//Check to see if the bus is busy
	while((I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY)
	{
		++timeOut;
		if(timeOut>timeOutVal)
		{
			break;
		}
	}
	
	//Clear Stop bit flag
	I2C1->ICR |= I2C_ICR_STOPCF;
}

void I2C_Write_LCD(uint32_t Register, uint32_t *Data, uint32_t Lenght)
{
		long unsigned int timeOut = 0;
		//Start communication
	I2C1->CR2 |= I2C_CR2_START |  ((Lenght+1) << 16);
	while(I2C1->CR2 & I2C_CR2_START)
	{
		++timeOut;
		if(timeOut>timeOutVal)
		{
			break;
		}
	}
		
	
	
	//Check Tx empty before writing to it
	if((I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE)){
		I2C1->TXDR = Register;
	}
		//Wait for transfer to complete
	 timeOut = 0;
	while((I2C1->ISR & I2C_ISR_TXE) == 0)
	{
		++timeOut;
		if(timeOut>timeOutVal)
		{
			break;
		}
	}
	
	
	for(int i=0; i<Lenght; i++)      // TIMEOUT KOY, ERROR DURUMDA ÇIKIS KOY, WATCHDOG EKLE VSVS
	{
		//Check Tx empty before writing to it
		if((I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE))
		{
			I2C1->TXDR = Data[i];
		}
		//Wait for transfer to complete
		timeOut = 0;
		while((I2C1->ISR & I2C_ISR_TXE) == 0)
		{
			++timeOut;
			if(timeOut>timeOutVal)
			{
				break;
			}
		}
		//Wait for transfer to complete	
	}

	
	//Send Stop Condition
	I2C1->CR2 |= I2C_CR2_STOP;	
	
	//Check to see if the bus is busy
	timeOut = 0;
	while((I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY)
	{
		++timeOut;
		if(timeOut>timeOutVal)
		{
			break;
		}
	}
	
	//Clear Stop bit flag
	I2C1->ICR |= I2C_ICR_STOPCF;
	
}
