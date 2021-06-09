#ifndef BUTTON_H_
#define BUTTON_H_
/******************************************************************************/
/** \file       button.h
 *******************************************************************************
 *
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****** Header-Files **********************************************************/
#include "stm32l0xx.h"

/****** Macros ****************************************************************/
#define SELENOID_NUMBER  4    // NUMBERS OF SELENOID  1,2,3,4 available


/****** Function prototypes ****************************************************/
extern void button_init(void);
extern void butread( void );
extern void Configure_DBG(void);
extern void Configure_EXTI(void);
extern void clearAllDetailAdjustment(void);
extern uint32_t bcd2cnv(uint32_t bcdData);

/****** Implementation ********************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BUTTON_H_ */
