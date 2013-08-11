/* 
 * File:   main.c
 * Author: Pavlo
 *
 * Created on August 3, 2013, 1:10 PM
 */


#include <xc.h>
#include "I2CdsPIC.h"
#include "MPU60xx.h"
#include <string.h>
#include <uart.h>

_FOSCSEL(FNOSC_FRC);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FWDT(FWDTEN_OFF);
_FICD(JTAGEN_OFF & ICS_PGD1);

void UART2Init();
void IMU2String(uint8_t *output, MPU6050_Data input);

MPU6050_Data imuData;
uint8_t buffer[256];

int main(void) {
    PLLFBD = 242; // M = 50 MIPS
    CLKDIVbits.PLLPOST = 0;
    CLKDIVbits.PLLPRE = 7;
    OSCTUN = 0;
    RCONbits.SWDTEN = 0;
    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to Primary (3?)
    __builtin_write_OSCCONL(0x01); // Start clock switching
    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1) {
    };


    T1CONbits.TON = 0;
    T1CONbits.TCS = 0;
    T1CONbits.TGATE = 0;
    T1CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    TMR1 = 0x00;
    PR1 = 156;
    IPC0bits.T1IP = 0x01;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;


    I2C_Init(9600);
    I2C1BRG = 0x4E;
    UART2Init();
    MPU60xx_Init();

    while (1) {
    };
    //Sit and Spin
}

void UART2Init(void) {
    U2MODEbits.UARTEN = 0;  // Disable the port
    U2MODEbits.USIDL = 0;   // Stop on idle
    U2MODEbits.IREN = 0;    // No IR decoder
    U2MODEbits.RTSMD = 0;   // Ready to send mode (irrelevant)
    U2MODEbits.UEN = 0;     // Only RX and TX
    U2MODEbits.WAKE = 1;    // Enable at startup
    U2MODEbits.LPBACK = 0;  // Disable loopback
    U2MODEbits.ABAUD = 0;   // Disable autobaud
    U2MODEbits.URXINV = 0;  // Normal operation (high is idle)
    U2MODEbits.PDSEL = 0;   // No parity 8 bit
    U2MODEbits.STSEL = 0;   // 1 stop bit
    U2MODEbits.BRGH = 0;

    IPC7 = 0x4400;
    // U2STA Register
    // ==============
    U2STAbits.URXISEL = 0;  // RX interrupt on every character
    U2STAbits.OERR = 0;     // clear overun error


    // U2BRG Register
    // ==============
    U2BRG = 26; //115200

    // Enable the port;
    U2MODEbits.UARTEN = 1;  // Enable the port
    U2STAbits.UTXEN = 1;    // Enable TX
}

IMU2String(uint8_t output, MPU6050_Data input) {
    buffer = "";
    uint8_t textBuff[16] = "";
    strcat(buffer, itoa(input.accelX, textBuff, 10));
    textBuff[16] = "";
    strcat(buffer, ",");
    strcat(buffer, itoa(input.accelY, textBuff, 10));
    textBuff[16] = "";
    strcat(buffer, ",");
    strcat(buffer, itoa(input.accelZ, textBuff, 10));
    textBuff[16] = "";
    strcat(buffer, ",");
    strcat(buffer, itoa(input.gyroX, textBuff, 10));
    textBuff[16] = "";
    strcat(buffer, ",");
    strcat(buffer, itoa(input.gyroY, textBuff, 10));
    textBuff[16] = "";
    strcat(buffer, ",");
    strcat(buffer, itoa(input.gyroZ, textBuff, 10));
    strcat(buffer, "\r\n\0");
}

/* Since we're just worried about getting data off of the IMU at a constant time
 * interval, we're going to just put it all of the stringification/sensing/sending
 * in this interrupt.  Usually a no-no, but this is fine.  Just don't use this in any
 * code that has any other timing requirements.
 */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    uint8_t txCount = 0;
    MPU60xx_Get6AxisData(&imuData);
    IMU2String(buffer, imuData);
    //Uart put chars here.
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
}

