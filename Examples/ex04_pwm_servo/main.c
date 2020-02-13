// ex04:

#include "ecc_pic24_bsp.h"


/**
 * Hardware connection for PWMs
 * - PWM_A --> LED2
 * - PWM_B --> LED3  
 * - PWM_C --> SCL  
 * - PWM_D --> SDA  
 */


// CUP Clock:	32 MHz
// TMR Clock:	32MZ/2 -> 16MHz/64 -> 250 kHz --> 4 uS;

/*
 250 kHz   -----> TMR2 -------> OC
 ( 4uS )          PR2=xx       OC1RS=??
                 (20mS)        (1.5mS)

*/
//                    (250 kHz) /     (50) = 5000 -> 20 mS
#define __PR2 		((32e6/2)/64) / (1/20e-3)

//                     (250 kHz)  /  (1000) 	// 250
#define __OC_MIN	((32e6/2)/64) / (1/1e-3)	// 1.0mS ->   0dg


//                     (250 kHz)  /  (~667) 	// 375
#define __OC_MID	((32e6/2)/64) / (1/1.5e-3)	// 105mS ->   90dg

//                     (250 kHz)  /  (500) 		// 500
#define __OC_MAX	((32e6/2)/64) / (1/2e-3)	// 2.0mS ->  180dg



// User input (deg 0->180) [ ? ] -> OCxRS
// 0   --> [?] --> OCxRS = 250
// 90  --> [?] --> OCxRS = 375
// 180 --> [?] --> OCxRS = 500


uint16_t __dt =  2;
uint16_t __sf =  40;
uint16_t __pr =  (16e6/64)/(1.0/20e-3);

void PWM_Init(void)
{
	//*******************************************
	//* Setup Timer2 for OC Clock source        *
	//*******************************************

	T2CONbits.TCKPS		= 2;			// 0->1:1, 1->1:8, 2->1:64, 3->1:256 prescaller
	T2CONbits.T32	 	= 0;			// 16-bit Timer
	T2CONbits.TCS	 	= 0;			// Internal clock (FOSC/2)
	T2CONbits.TGATE	 	= 0;			// Gated time accumulation disabled
	T2CONbits.TON		= 1;			// Start Timer
	PR2					= __PR2;		// Timer Period register, for 20 mS 
	IEC0bits.T2IE		= 0;			// Enable Timer2 Interrupt


	//---------OC1 [A, PWM_A, LED2] ------------------------------------------------------
	OC1R 				= 0;				// Go High (1)
	OC1RS 				= __OC_MIN;			// Go low (0) --> , for 1.5 mS (90 dg)
	OC1CONbits.OCM		= 5;			// Dual Compare, Continous pulse output
	OC1CONbits.OCTSEL 	= 0;			// Timer2 is the clock source for Output Compare x


	//---------OC2 [B, PWM_B, LED3] ------------------------------------------------------
	OC2R 				= 0;
	OC2RS 				= __OC_MAX;
	OC2CONbits.OCM		= 5;
	OC2CONbits.OCTSEL 	= 0;


	//---------OC3 [C: A+SHIFT] ------------------------------------------------------
	OC3R 				= 1 + __sf;			// Hi
	OC3RS 				= OC1RS + __sf;		// Lo
	OC3CONbits.OCM		= 5;				// Dual Compare, Continous pulse output
	OC3CONbits.OCTSEL 	= 0;				// Timer2 is the clock source for Output Compare x

	//---------OC4 [D: B+SHIFT] ------------------------------------------------------
	OC4R 				= OC3RS  + __dt;
	OC4RS 				= OC3R   - __dt;
	OC4CONbits.OCM		= 5;
	OC4CONbits.OCTSEL 	= 0;


	//*******************************************
	//* Connect COx to PORTB.x 		            *
	//*******************************************
	MCU_UnlockRegisters();
	//RPOR2bits.RP4R 		= 18;		// OC1: Connect Output Compare 1 to RP4 (PORTB.4)
	//RPOR2bits.RP5R 		= 19;		// OC2: Connect Output Compare 2 to RP5 (PORTB.5)
	//RPOR3bits.RP6R		= 20;		// OC3: Connect Output Compare 3 to RP6 (PORTB.6)
	//RPOR3bits.RP7R		= 21;		// OC4: Connect Output Compare 4 to RP7 (PORTB.7)

	RPOR1bits.RP2R 		= 18;		// OC1(18): Connect Output Compare 1 to RP2 (PORTB.2) -> LED2
	RPOR1bits.RP3R 		= 19;		// OC2(19): Connect Output Compare 2 to RP3 (PORTB.3) -> LED3

	RPOR4bits.RP8R		= 20;		// OC3(20): Connect Output Compare 3 to RP8 (PORTB.8) -> SCL
	RPOR4bits.RP9R		= 21;		// OC4(21): Connect Output Compare 4 to RP9 (PORTB.9) -> SDA
	MCU_LockRegisters();


}


int main(void)
{
    // Hardware Setup
    System_Init();


    PWM_Init();


	while(1);


    // Start
    //vTaskStartScheduler();
    return 0;
}
