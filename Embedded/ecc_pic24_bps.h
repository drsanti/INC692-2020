
#ifndef __ECC_PIC24_BSP_H__
#define __ECC_PIC24_BSP_H__

#include <xc.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 16000000UL
#endif

#define LED0_TRIS TRISAbits.TRISA2
#define LED1_TRIS TRISAbits.TRISA4
#define LED2_TRIS TRISBbits.TRISB2
#define LED3_TRIS TRISBbits.TRISB3

#define LED0_LAT LATAbits.LATA2
#define LED1_LAT LATAbits.LATA4
#define LED2_LAT LATBbits.LATB2
#define LED3_LAT LATBbits.LATB3

#define LED0_PORT PORTAbits.RA2
#define LED1_PORT PORTAbits.RA4
#define LED2_PORT PORTBbits.RB2
#define LED3_PORT PORTBbits.RB3

void System_Init(void);
void CLOCK_Initialize(void);
void IO_Initialize(void);

#endif // __ECC_PIC24_BSP_H__
