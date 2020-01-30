#include "ecc_pic24_bps.h"

// CONFIG2
#pragma config POSCMOD = NONE //Primary Oscillator Select->Primary oscillator disabled
#pragma config I2C1SEL = PRI  //I2C1 Pin Location Select->Use default SCL1/SDA1 pins
#pragma config IOL1WAY = ON   //IOLOCK Protection->Once IOLOCK is set, cannot be changed
#pragma config OSCIOFNC = OFF //Primary Oscillator Output Function->OSC2/CLKO/RC15 functions as CLKO (FOSC/2)
#pragma config FCKSM = CSDCMD //Clock Switching and Monitor->Clock switching and Fail-Safe Clock Monitor are disabled
#pragma config FNOSC = FRCPLL //Oscillator Select->Fast RC Oscillator with PLL module (FRCPLL)
#pragma config SOSCSEL = SOSC //Sec Oscillator Select->Default Secondary Oscillator (SOSC)
#pragma config WUTSEL = LEG   //Wake-up timer Select->Legacy Wake-up Timer
#pragma config IESO = ON      //Internal External Switch Over Mode->IESO mode (Two-Speed Start-up) enabled

// CONFIG1
#pragma config WDTPS = PS32768 //Watchdog Timer Postscaler->1:32768
#pragma config FWPSA = PR128   //WDT Prescaler->Prescaler ratio of 1:128
#pragma config WINDIS = ON     //Watchdog Timer Window->Standard Watchdog Timer enabled,(Windowed-mode is disabled)
#pragma config FWDTEN = OFF    //Watchdog Timer Enable->Watchdog Timer is disabled
#pragma config ICS = PGx3      //Comm Channel Select->Emulator EMUC3/EMUD3 pins are shared with PGC3/PGD3
#pragma config BKBUG = OFF     //Background Debug->Device resets into Operational mode
#pragma config GWRP = OFF      //General Code Segment Write Protect->Writes to program memory are allowed
#pragma config GCP = OFF       //General Code Segment Code Protect->Code protection is disabled
#pragma config JTAGEN = OFF    //JTAG Port Enable->JTAG port is disabled

void CLOCK_Initialize(void)
{
    // RCDIV FRC/2; DOZE 1:8; DOZEN disabled; ROI disabled;
    CLKDIV = 0x3100;
    // TUN Center frequency;
    // OSCTUN = 0x00;
    // ADC1MD enabled; T3MD enabled; T4MD enabled; T1MD enabled; U2MD enabled; T2MD enabled; U1MD enabled; SPI2MD enabled; SPI1MD enabled; T5MD enabled; I2C1MD enabled;
    PMD1 = 0x00;
    // OC5MD enabled; IC5MD enabled; IC4MD enabled; IC3MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; OC4MD enabled;
    PMD2 = 0x00;
    // PMPMD enabled; RTCCMD enabled; CMPMD enabled; CRCPMD enabled; I2C2MD enabled;
    PMD3 = 0x00;
    // NOSC FRCPLL; SOSCEN disabled; OSWEN Switch is Complete;
    __builtin_write_OSCCONH((uint8_t)(0x01));
    __builtin_write_OSCCONL((uint8_t)(0x00));
}

void IO_Initialize(void)
{
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x0000;

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0x0003;
    TRISB = 0xFFF3;

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPU1 = 0x0000;
    CNPU2 = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    AD1PCFG = 0x0000;
}

void vApplicationIdleHook(void)
{
    vCoRoutineSchedule();
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;
    taskDISABLE_INTERRUPTS();
    for (;;)
        ;
}

void System_Init(void)
{
    CLOCK_Initialize();
    IO_Initialize();
}
