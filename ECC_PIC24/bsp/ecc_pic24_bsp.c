/*
 ***************************************************************
 * File Name: ecc_pic24_bsp.c
 ***************************************************************
 * Asst.Prof.Dr.Santi Nuratch
 * Embedded Computing and Control Laboratory (ECC-Lab)
 * Department of Control System and Instrumentation Engineering
 * Faculty of Engineering, KMUTT
 * Update: 13 February, 2020
 ***************************************************************
 */

#include "ecc_pic24_bsp.h"

// CONFIG2
#pragma config POSCMOD  = NONE      // Primary Oscillator Select->Primary oscillator disabled
#pragma config I2C1SEL  = PRI       // I2C1 Pin Location Select->Use default SCL1/SDA1 pins
#pragma config IOL1WAY  = OFF       // ON   //IOLOCK Protection->Once IOLOCK is set, cannot be changed
#pragma config OSCIOFNC = OFF       // Primary Oscillator Output Function->OSC2/CLKO/RC15 functions as CLKO (FOSC/2)
#pragma config FCKSM    = CSDCMD    // Clock Switching and Monitor->Clock switching and Fail-Safe Clock Monitor are disabled
#pragma config FNOSC    = FRCPLL    // Oscillator Select->Fast RC Oscillator with PLL module (FRCPLL)
#pragma config SOSCSEL  = SOSC      // Sec Oscillator Select->Default Secondary Oscillator (SOSC)
#pragma config WUTSEL   = LEG       // Wake-up timer Select->Legacy Wake-up Timer
#pragma config IESO     = ON        // Internal External Switch Over Mode->IESO mode (Two-Speed Start-up) enabled

// CONFIG1
#pragma config WDTPS    = PS32768   // Watchdog Timer Postscaler->1:32768
#pragma config FWPSA    = PR128     // WDT Prescaler->Prescaler ratio of 1:128
#pragma config WINDIS   = ON        // Watchdog Timer Window->Standard Watchdog Timer enabled,(Windowed-mode is disabled)
#pragma config FWDTEN   = OFF       // Watchdog Timer Enable->Watchdog Timer is disabled
#pragma config ICS      = PGx3      // Comm Channel Select->Emulator EMUC3/EMUD3 pins are shared with PGC3/PGD3
#pragma config BKBUG    = OFF       // Background Debug->Device resets into Operational mode
#pragma config GWRP     = OFF       // General Code Segment Write Protect->Writes to program memory are allowed
#pragma config GCP      = OFF       // General Code Segment Code Protect->Code protection is disabled
#pragma config JTAGEN   = OFF       // JTAG Port Enable->JTAG port is disabled

/* UnLock IO-Mapping */
void MCU_UnlockRegisters(void)
{
    __asm("MOV #OSCCON, w1"); // w1 holds ADDRESS of OSCCON
    __asm("MOV #0x46, w2");
    __asm("MOV #0x57, w3");
    __asm("MOV.b w2, [w1]");
    __asm("MOV.b w3, [w1]");
    __asm("BCLR OSCCON,#6"); // Clear OSCCON<6>
}

/* Lock IO-Mapping */
void MCU_LockRegisters(void)
{
    __asm("MOV #OSCCON, w1");
    __asm("MOV #0x46, w2");
    __asm("MOV #0x57, w3");
    __asm("MOV.b w2, [w1]");
    __asm("MOV.b w3, [w1]");
    __asm("BSET OSCCON, #6"); // Set OSCCON<6>
}

/* Initial MCU's clock and disable ANz*/
/*
void MCU_Init(void)
{
    CLKDIVbits.RCDIV    = 0;        // Post-scaler, 8MHz/1 = 8MHz (divide by 1), used for FRC Post-scaler Select bits
    CLKDIVbits.DOZEN    = 0;        // Enable DOZE
    CLKDIVbits.DOZE     = 0;
    AD1PCFG             = 0xFFFF;   // Disable all AN pins
}
*/

void CLOCK_Initialize(void)
{
    // // RCDIV FRC/2; DOZE 1:8; DOZEN disabled; ROI disabled;
    // CLKDIV = 0x3100;
    // // TUN Center frequency;
    // // OSCTUN = 0x00;
    // // ADC1MD enabled; T3MD enabled; T4MD enabled; T1MD enabled; U2MD enabled; T2MD enabled; U1MD enabled; SPI2MD enabled; SPI1MD enabled; T5MD enabled; I2C1MD enabled;
    // PMD1 = 0x00;
    // // OC5MD enabled; IC5MD enabled; IC4MD enabled; IC3MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; OC4MD enabled;
    // PMD2 = 0x00;
    // // PMPMD enabled; RTCCMD enabled; CMPMD enabled; CRCPMD enabled; I2C2MD enabled;
    // PMD3 = 0x00;
    // // NOSC FRCPLL; SOSCEN disabled; OSWEN Switch is Complete;
    // __builtin_write_OSCCONH((uint8_t)(0x01));
    // __builtin_write_OSCCONL((uint8_t)(0x00));

    // RCDIV FRC/1; DOZE 1:8; DOZEN disabled; ROI disabled;
    CLKDIV = 0x3000;
    // TUN Center frequency;
    OSCTUN = 0x00;
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

void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
    // data = U1RXREG;
}

void __attribute__((interrupt, auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0;
    //U1TXREG = data;
}

void __attribute__((interrupt, auto_psv)) _U2RXInterrupt(void)
{
    IFS1bits.U2RXIF = 0;
    // data = U2RXREG;
}

void __attribute__((interrupt, auto_psv)) _U2TXInterrupt(void)
{
    IFS1bits.U2TXIF = 0;
    //U2TXREG = data;
}

void Uart1_Init(uint32_t baurate)
{
    uint8_t fast_speed = (baurate > 9600) ? 1 : 0;

    //####################################
    //#         Unlock Registers         #
    //####################################
    MCU_UnlockRegisters(); //## Required

    //####################################
    //#      Assign U1RX To Pin RP12     #
    //####################################
    RPINR18bits.U1RXR = 12;
    TRISBbits.TRISB12 = 1;

    //####################################
    //#      Assign U1TX To Pin RP13     #
    //####################################
    RPOR6bits.RP13R   = 3;
    TRISBbits.TRISB13 = 0;


    //####################################
    //#          Lock Registers          #
    //####################################
    MCU_LockRegisters(); //## Required

    //************************************
    //* Make sure PR<3:0> are set as     *
    //* Digital Input/Output by writing  *
    //* '1' to corresponding bit in      *
    //* register AD1PCFG                 *
    //************************************
    //AD1PCFG |= 0x000F;
    //AD1PCFG |= 0x6000;		// PR<13:12> are digital Port. 0110 0000 0000 0000

    //####################################
    //#   Set UART1 as Normal Rx/Tx      #
    //####################################
    U1MODEbits.UARTEN   = 1; // UART1 Enable
    U1MODEbits.USIDL    = 0;  // Continue module operation in Idle mode
    U1MODEbits.IREN     = 0;   // IrDA encoder and decoder disabled
    U1MODEbits.RTSMD    = 0;  // UxRTS pin in Flow Control mode
    U1MODEbits.UEN      = 0;    // U1CTS and U1RTS/BCLKx pins controlled by PORT latches
    U1MODEbits.WAKE     = 0;   // No wake-up enabled
    U1MODEbits.LPBACK   = 0; // Loopback mode is disabled
    U1MODEbits.ABAUD    = 0;  // Baud rate measurement disabled or completed
    U1MODEbits.RXINV    = 0;  // U1RX Idle state is ‘1’
    //U1MODEbits.BRGH		= (UART1_FASTSPEED>0)?1:0;	// BRG generates 16 clocks per bit period (16x baud clock, Standard mode)
    U1MODEbits.BRGH     = fast_speed;
    U1MODEbits.PDSEL    = 0; // 8-bit data, no parity
    U1MODEbits.STSEL    = 0; // One Stop bit

    //####################################
    //#      Config UART's baudrate      #
    //####################################
    //>>k = (UART_FASTSPEED>0)? k=4.0 : 16.0;
    //>>U1BRG = (FCY/(k*UART_BAUDRATE))-1;
    //>>U1BRG = (UART_FASTSPEED>0)?(FCY/(4.0*UART_BAUDRATE))-1:(FCY/(16.0*UART_BAUDRATE))-1;
    //U1BRG = (UART1_FASTSPEED>0)?(FCY/(UART1_BAUDRATE*4.0))-1:(FCY/(UART1_BAUDRATE*16.0))-1;
    U1BRG = (fast_speed > 0) ? (FCY / (baurate << 2)) - 1 : (FCY / (baurate << 4)) - 1;
    // It uses just 2 instruction cycles, 0.5uS @ FOSC=8MHz.
    // The computation is done by compiler (no run-time is required)

    //####################################
    //#     Enable UART Transmission     #
    //####################################
    U1STAbits.UTXEN  = 1;   // Transmit enabled
    U1STAbits.UTXBRK = 0;   // Sync Break transmission disabled or completed

    U1STAbits.UTXISEL1 = 0; // Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the transmit buffer becomes empty
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.URXISEL1 = 0; // Interrupt is set when any character is received and transferred from the RSR to the receive buffer. Receive buffer has one or more characters.
    U1STAbits.URXISEL0 = 0;

    IEC0bits.U1RXIE = 1;
    IEC0bits.U1TXIE = 0;

    U1STAbits.UTXEN = 1; // Transmit enabled
}

void Uart2_Init(uint32_t baurate)
{
    uint8_t fast_speed = (baurate > 9600) ? 1 : 0;

    MCU_UnlockRegisters(); //## Required

    //####################################
    //#      Assign U2RX To Pin RP14     #
    //####################################
    RPINR19bits.U2RXR = 14;
    TRISBbits.TRISB14 = 1;

    //####################################
    //#      Assign U2TX To Pin RP15     #
    //####################################
    RPOR7bits.RP15R   = 5;
    TRISBbits.TRISB15 = 0;

    MCU_LockRegisters(); //## Required

    //####################################
    //#   Set UART2 as Normal Rx/Tx      #
    //####################################
    U2MODEbits.UARTEN   = 1;    // UART2 Enable
    U2MODEbits.USIDL    = 0;    // Continue module operation in Idle mode
    U2MODEbits.IREN     = 0;    // IrDA encoder and decoder disabled
    U2MODEbits.RTSMD    = 0;    // UxRTS pin in Flow Control mode
    U2MODEbits.UEN      = 0;    // U2CTS and U2RTS/BCLKx pins controlled by PORT latches
    U2MODEbits.WAKE     = 0;    // No wake-up enabled
    U2MODEbits.LPBACK   = 0;    // Loopback mode is disabled
    U2MODEbits.ABAUD    = 0;    // Baud rate measurement disabled or completed
    U2MODEbits.RXINV    = 0;    // U2RX Idle state is ‘1’
    //U2MODEbits.BRGH		= (UART2_FASTSPEED>0)?1:0;	// BRG generates 16 clocks per bit period (16x baud clock, Standard mode)
    U2MODEbits.BRGH     = fast_speed;
    U2MODEbits.PDSEL    = 0;    // 8-bit data, no parity
    U2MODEbits.STSEL    = 0;    // One Stop bit

    //####################################
    //#      Config UART's baudrate      #
    //####################################
    //>>k = (UART_FASTSPEED>0)? k=4.0 : 16.0;
    //>>U1BRG = (FCY/(k*UART_BAUDRATE))-1;
    //>>U1BRG = (UART_FASTSPEED>0)?(FCY/(4.0*UART_BAUDRATE))-1:(FCY/(16.0*UART_BAUDRATE))-1;
    //U2BRG = (UART2_FASTSPEED>0)?(FCY/(baurate*4.0))-1:(FCY/(baurate*16.0))-1;
    U2BRG = (fast_speed > 0) ? (FCY / (baurate << 2)) - 1 : (FCY / (baurate << 4)) - 1;
    // It uses just 2 instruction cycles, 0.5uS @ FOSC=8MHz.
    // The computation is done by compiler (no run-time is required)

    //####################################
    //#     Enable UART Transmission     #
    //####################################
    U2STAbits.UTXEN  = 1;   // Transmit enabled
    U2STAbits.UTXBRK = 0;   // Sync Break transmission disabled or completed

    U2STAbits.UTXISEL1 = 0; // Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the transmit buffer becomes empty
    U2STAbits.UTXISEL0 = 0;
    U2STAbits.URXISEL1 = 0; // Interrupt is set when any character is received and transferred from the RSR to the receive buffer. Receive buffer has one or more characters.
    U2STAbits.URXISEL0 = 0;

    IEC1bits.U2RXIE = 1;
    IEC1bits.U2TXIE = 0;

    U2STAbits.UTXEN = 1; // Transmit enabled
}

void UART1_Put(char c)
{
    while (U1STAbits.UTXBF);
    U1TXREG = c;
}
void UART1_Get(char *c)
{
    if (U1STAbits.OERR != 0)
        U1STAbits.OERR = 0;

    while (!U1STAbits.URXDA);
    *c = U1RXREG;
}
void UART1_Write(char *str)
{
    while (*str != 0)
    {
        while (U1STAbits.UTXBF);
        U1TXREG = *str;
        str++;
    }
}

void UART2_Put(char c)
{
    while (U2STAbits.UTXBF);
    U2TXREG = c;
}
void UART2_Get(char *c)
{
    if (U2STAbits.OERR != 0)
        U2STAbits.OERR = 0;

    while (!U2STAbits.URXDA);
    *c = U2RXREG;
}
void UART2_Write(char *str)
{
    while (*str != 0)
    {
        while (U2STAbits.UTXBF);
        U2TXREG = *str;
        str++;
    }
}

void System_Init(void)
{
    CLOCK_Initialize();
    IO_Initialize();

    Uart1_Init(115200 * 1);
    Uart2_Init(115200 * 1);

    LED0_LAT = LED1_LAT = LED2_LAT = LED3_LAT = 1;

}

/**
 * Print the given string to serial port (UART1)
 * string: A null-terminated string to be printed.
 */
void  rtl_print(const char * string) {
    while (*string != 0)
    {
        while (U1STAbits.UTXBF);
        U1TXREG = *string;
        string++;
    }
}

/**
 * Print the given string to serial port (UART1)
 * string: A null-terminated string to be printed.
 */
void rtl_printf(const char *format, ...) {
    char buff[128];
    va_list args;
    va_start(args, format);
    vsprintf(buff, format, args);
    va_end(args);
    buff[127] = 0;
    while (*buff != 0)
    {
        while (U1STAbits.UTXBF);
        U1TXREG = *buff;
        buff++;
    }
}
