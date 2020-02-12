// ex04:

#include "ecc_pic24_bsp.h"


uint16_t __dt =  2;
uint16_t __sf =  40;
uint16_t __pr =  (16e6/64)/(1.0/20e-3);

void PWM_Init(void)
{
	//*******************************************
	//* Setup Timer2 for OC Clock source        *
	//*******************************************
	//T2CONbits.TCKPS	 	= 0;		// 1:1 pre-scaler
	T2CONbits.TCKPS		= 2;		// 1:64 prescaller
	T2CONbits.T32	 	= 0;		// 16-bit Timer
	T2CONbits.TCS	 	= 0;		// Internal clock (FOSC/2)
	T2CONbits.TGATE	 	= 0;		// Gated time accumulation disabled
	T2CONbits.TON		= 1;		// Start Timer
	PR2					= __pr;		// Timer Period register, 100kHz, @FCY=16MHz(FOSC=32MHz)
	IEC0bits.T2IE		= 0;		// Enable Timer2 Interrupt


	//---------OC1 [A] ------------------------------------------------------
	OC1R 				= 1;		// Hi
	//OC1RS 			= __pr>>1;	// Lo
	OC1RS 				= __pr*(99.0/100.0);
	OC1CONbits.OCM		= 5;		// Dual Compare, Continous pulse output
	OC1CONbits.OCTSEL 	= 0;		// Timer2 is the clock source for Output Compare x


	//---------OC2 [B] ------------------------------------------------------
	// OC2R 			= OC1RS + __dt + 1 ;
	// OC2RS 			= __pr   - __dt;
	OC2R 				= 1;
	OC2RS 				= __pr*(5.0/100.0);
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
