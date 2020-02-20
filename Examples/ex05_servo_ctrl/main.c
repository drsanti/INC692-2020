// ex05:

#include "ecc_pic24_bsp.h"

float __duty = 0.5;

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
	PR2					= 0;			// Timer Period register
	IEC0bits.T2IE		= 0;			// Enable Timer2 Interrupt

	//---------OC1 [A, PWM_A, LED2] ------------------------------------------------------
	OC1R 				= 0;			// Go High (1)
	OC1RS 				= 0;			// Go low (0) --> , for 1.5 mS (90 dg)
	OC1CONbits.OCM		= 5;			// Dual Compare, Continous pulse output
	OC1CONbits.OCTSEL 	= 0;			// Timer2 is the clock source for Output Compare x


	//*******************************************
	//* Connect COx to PORTB.x 		            *
	//*******************************************
	MCU_UnlockRegisters();
	RPOR1bits.RP2R 		= 18;		// OC1(18): Connect Output Compare 1 to RP2 (PORTB.2) -> LED2
	RPOR1bits.RP3R 		= 19;		// OC2(19): Connect Output Compare 2 to RP3 (PORTB.3) -> LED3
	RPOR4bits.RP8R		= 20;		// OC3(20): Connect Output Compare 3 to RP8 (PORTB.8) -> SCL
	RPOR4bits.RP9R		= 21;		// OC4(21): Connect Output Compare 4 to RP9 (PORTB.9) -> SDA
	MCU_LockRegisters();
}

void PWM_SetFreq(float freq) {
	// PR 16-bit register used to control signal period. 
	PR2 = ((32e6/2)/64) / (freq) + 0.5;

	// Update the duty ratio based on the new freq.
	OC1RS = PR2*__duty;

	//char buff[32];
	//sprintf(buff, "PR2: %lu, OC1RS: %lu\r\n", (uint32_t)PR2, (uint32_t)OC1RS);
	//UART1_Write(buff);
}

void PWM_SetDuty(float duty) {
	__duty = duty;
	OC1RS = PR2*duty;
}

void PWM_ServoCtrl(float dg) {
	// min: 1/20 @ dg=0
	// max: 2/20 @ dg=180
	// 0   --> 1     --> /20 --> 1/20
	// 180 --> 2     --> /20 --> 2/20

	float duty = (1 + (dg/180.0)) / 20.0;
	PWM_SetDuty( duty );	

	// char buff[32];
	// sprintf(buff, "dg: %f, duty: %f\r\n", (double)dg, (double)duty);
    // UART1_Write(buff);
}

int main(void)
{
    System_Init();
    PWM_Init();

	// max: 2500 kHz
	// min: 3.9  Hz
	PWM_SetFreq(50);

	// max: <1.0
	// min: >0.0
	PWM_SetDuty(0.1);


	// New settings
	PWM_SetFreq(25);
	

	// Test Freq variation
	// uint32_t k = 0;
	// float freq = 5;
	// while(1){
	// 	PWM_SetFreq(freq);
	// 	k = 0;
	// 	freq += 1;
	// 	while(k++ < 5000);
	// }






	// max: 180 dg
	// min:   0 dg
	PWM_ServoCtrl(180);

    char* user = "1";

	float dg = atof(user); //Convert string to double.
	PWM_ServoCtrl(dg);

	while(1){};

	/*
	float dg  = 0.0;
	float dir = 1.0;
	uint32_t k = 0;
	while(1) {
		PWM_ServoCtrl(dg);
		dg += 0.5 * dir;

		if( dg > 180 || dg < 0.0) {
			dir *= -1;
		}
		k = 0;
		while(k++ < 5000);
	}
	*/
    return 0;
}
