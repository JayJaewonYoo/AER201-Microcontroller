/*
 * File:   main.c
 * Author: Jay Yoo
 */

// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

// PIC18F4620 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = ON         // Data EEPROM Code Protection bit (Data EEPROM code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Includes">
#include <xc.h>
#include <stdio.h>
#include "configBits.h"
#include "constants.h"
#include "lcd.h"
#include "GLCD_PIC.h"
#include "I2C.h"
#include "SPI_PIC.h"
#include "p18f4620.h"
#include "string.h"
#include "pconfig.h"
#include "pic18f4620.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Array Constants">
const char keys[] = "123A456B789C*0#D"; 
const char assembly4[4] = {1, 3, 5, 7};
const char assembly5[5] = {1, 2, 4, 5, 7};
const char assembly6[6] = {1, 2, 3, 5, 6, 7};

const char date_time[7] = {  0x0, // 45 Seconds 
                                0x42, // 08 Minutes
                                0x10, // 24 hour mode, set to 12:00
                                0x02, // 0x00 = Sunday, currently set to Wednesday
                                0x10, // 21st
                                0x04, // February
                                0x18};  // 2018
const char password1[5] = {'3', '7', '8', '1', '8'};
const char password2[5] = {'A', 'B', 'C', 'D', '1'};
const int RTC_organize[6] = {6, 5, 4, 2, 1, 0};
const char logNames[19][16] = {"Log Flag:       ",
                                "B Remaining:    ", "N Remaining:    ", "S Remaining:    ", "W Remaining:    ", 
                                "Date:           ", "Date:           ", "Date:           ", 
                                "Time:           ", "Time:           ", "Time:           ",
                                "Runtime:        ", "Runtime:        ",
                                "B Selected:     ", "N Selected:     ", "S Selected:     ", "W Selected:     ", 
                                "Fasteners/Step: ",
                                "Assembly Steps: "};

// Text:
const char jeremiah[10][16] = {"Jeremiah 29:11  ", "For I know the  ", "plans I have for", "you, declares   ", "the Lord, plans ", "to prosper you  ", "and not harm you", ", plans to give ", "you hope and a  ", "future.  Press #"};
const char instructions[8][16] = {"Press 1 for     ", "Bolts (B)       ", "Press 2 for     ", "Nuts (N)        ", "Press 3 for     ", "Spacers (S)     ", "Press A for     ", "Washers (W)     "};
// Passwords:
const char fast1[16] = {'B', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast2[16] = {'N', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast3[16] = {'S', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast4[16] = {'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast5[16] = {'B', 'N', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast6[16] = {'B', 'S', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast7[16] = {'B', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast8[16] = {'B', 'B', 'N', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast9[16] = {'B', 'B', 'S', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast10[16] = {'B', 'B', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast11[16] = {'B', 'N', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast12[16] = {'B', 'S', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast13[16] = {'B', 'W', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast14[16] = {'B', 'N', 'W', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast15[16] = {'B', 'S', 'W', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast16[16] = {'B', 'B', 'S', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast17[16] = {'B', 'B', 'N', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast18[16] = {'B', 'N', 'N', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast19[16] = {'B', 'N', 'N', 'N', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
const char fast20[16] = {'B', 'W', 'W', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};

const char GLCDpixel[37][24] = {"011010011101101110010110", "001001101010001000100010", "011010010001011010001111", "111000010111000100011110", "001101011001111100010001", 
                                "111110001110000110010110", "011010001110100110010110", "111100010010001001000100", "111110011111100110011111", "111110011111000100010001",
                                "011010011001111110011001", "111010011110100110011110", "011010011000100010010110", "111010011001100110011110", "111110001110100010001111", 
                                "111110001110100010001000", "111110001000101110011111", "100110011111100110011001", "111001000100010001001110", "111100100010001010101110", 
                                "100110101100101010011001", "100010001000100010001111", "100111111001100110011001", "100111011011100110011001", "011010011001100110010110", 
                                "111010011001111010001000", "011010011001100101100001", "111010011001111010011001", "011110000110000100011110", "111001000100010001000100", 
                                "100110011001100110010110", "101010101010101010100100", "101010101010101011101010", "101010101010010010101010", "100110011111000100011111", 
                                "111100010010010010001111", "000000000000000000000000"};
// 0 1 2 3 4 
// 5 6 7 8 9
// A B C D E
// F G H I J
// K L M N O
// P Q R S T
// U V W X Y
// Z SPACE
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Global Variables">
// Variables Involved in Interrupts:
unsigned int loopFlag;
unsigned int emergency = 0;
unsigned int operation = 0;
// Variables Involved in RTC:
unsigned char time[7]; // Create a byte array to hold time read from RTC
unsigned char i; // Loop counter
unsigned char start[6]; // YY/MM/DD/HH/MM/SS
unsigned char end[6]; // YY/MM/DD/HH/MM/SS
unsigned int runtime[6];
unsigned int tempend[6];
unsigned int tempstart[6];
unsigned int eightFlag = 0;

int numBremaining = 0, numNremaining = 0, numSremaining = 0, numWremaining = 0;
int numB = 0, numN = 0, numS = 0, numW = 0;
int nPerStep = 0;
int nAssembly = 0;

int load[40]; // Load from EEPROM. Contents:
    // 0      == log completion flag
    // 1      == number of bolts remaining
    // 2      == number of nuts remaining
    // 3      == number of spacers remaining
    // 4      == number of washers remaining
    // 5-10   == start date/time (YY/MM/DD/HH/MM/SS)
    // 11-12  == runtime (MM/SS)
    // 13-28  == parameter: fasteners sets for C1,...,C8      Note that load[13] has BN and load[14] has SW for C1
    // 29-36  == parameter: fastener sets per step for C1,...C8
    // 37     == parameter: number of assembly steps
    // 38     == emergency stopped? (1 = yes, 0 = no);
    // 39     == Number of stored logs - 1

unsigned char select;

unsigned int color = 0; // 0 -> dark, 0 -> light 

unsigned int step = 0;

unsigned int fastenerSet[16];
// each element is a 4 digit number that represents BNSW
unsigned int fastenerSetPerStep[8];
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Macros">
#define _XTAL_FREQ 40000000
#define TMR2PRESCALE 16
#define __bcd_to_num(num) (num & 0x0F) + ((num & 0xF0)>>4)*10
#define DELAY_TIME  4
// 4 is consistent
#define SOLENOID_DELAY  300
#define MICRO_LOOPS  70

#define baud  7200
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Function Prototypes">
void RTC_setTime(void);
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Important Notes">
// Important Notes:
// call I2C_Master_Init(100000) before every interaction with the RTC, and spiInit(16) before every interaction with the GLCD.
// DO NOT UNCOMMENT THE RTC_setTime() LINE IN THE RTC INITIALIZATION! 
    // This will reconfigure the RTC clock and setting it back requires you to to program it twice, once with the line uncommented then again with the line commented. The constant seconds
    // in date_time must be set to 39 seconds after you program. For example: you try to set the RTC at 23:00. You have to set the seconds to be at 39 seconds and then program the PIC 
    // right at 23:00 real time, twice. Program first time with the line uncommented and a second time with the line commented. 
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="EEPROM Functions">
unsigned char read_eeprom(unsigned char address) {
    // Read data from EEPROM
    // Input address of memory, return contents of memory. 
    
    EEADRH = (unsigned char)(address >> 8);
    EEADR = (unsigned char)address;
    
    // EECON1 set to 0 to select EEPROM memory over flash memory without changing registers
    EECON1bits.EEPGD = 0; // This causes bugs
    EECON1bits.CFGS = 0; // This causes bugs
    
    EECON1bits.RD = 1; // Set to read. 
    
    while(EECON1bits.RD == 1){continue;} // Wait for read
    
    return EEDATA;
}

void write_eeprom(unsigned short address, unsigned char input) {
    // Enable writing to EEPROM
    // Input address and data to store, output void. 
    
    EECON1bits.WREN = 1;
    
    // Address registers:
    EEADRH = (unsigned char)(address >> 8);
    EEADR = (unsigned char)address;
    
    EEDATA = input;
    
    // Enable bits:
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    
    di(); // Disable interrupts
    
    // Write password in to enable:
    EECON2 = 0x55;
    EECON2 = 0x0AA;
    EECON1bits.WR = 1;
    
    ei(); // Enable interrupts
    
    while(PIR2bits.EEIF == 0) {continue;} // wait until finish writing to EEPROM
    PIR2bits.EEIF = 0; // Reset interrupt flag
    EECON1bits.WREN = 0; // Disable writing to EEPROM
}

void init_eeprom(void) {
    // EEPROM has memory address from 0x00 to 0xFF
    // EEPROM has 256 bytes of available memory
    write_eeprom(0, 0x01);
}

void store_data(void) {    
    // stores in the following order:
        // number of fasteners remaining (B -> N -> S -> W)
        // start time of operation (year - > month -> day -> hour -> minute -> second)
        // runtime (minute -> second)
        // parameter: number of fasteners (B -> N -> S -> W)
        // parameter: number of fasteners per step
        // parameter: number of steps in assembly
    
    // Choose address:
    select = read_eeprom(0);
    unsigned short address = 1 + (select * 38); // 18 is the number of bytes being stored
    
    // Increment selection bits, clear if max
    if(select < 6) {
        unsigned char tempNum = select + 1;
        write_eeprom(0, tempNum);
    } else {
        write_eeprom(0, 0);
    }
    
    // Store completion flag and # of remaining fasteners:
    // 4 bytes
    unsigned char remaining = 0b10000000;// log completion flag
    write_eeprom(address, remaining);
    address += 1;
    
    remaining = numBremaining;
    write_eeprom(address, remaining);
    address += 1;
    
    remaining = numNremaining;
    write_eeprom(address, remaining);
    address += 1;
    
    remaining = numSremaining;
    write_eeprom(address, remaining);
    address += 1;
    
    remaining = numWremaining;
    write_eeprom(address, remaining);
    address += 1;
    
    // Store date/time:
    // 6 bytes
    for(unsigned int i = 0; i < 6; i++) {
        unsigned char dateByte = start[i];
        write_eeprom(address, dateByte);
        address += 1;
    }
    
    // Store runtime:
    // 2 bytes
    for(i = 4; i < 6; i++) {
        unsigned char dataByte = runtime[i];
        write_eeprom(address, dataByte);
        address += 1;
    }
    
    // Store parameters:
    // 6 bytes
    unsigned char quantity;
    for(i = 0; i < 16; i += 2) { 
        unsigned int tempValue = fastenerSet[i];
        tempValue = fastenerSet[i];
        quantity = tempValue;
        write_eeprom(address, quantity);
        address += 1;
        tempValue = fastenerSet[i+1];
        quantity = tempValue;
        write_eeprom(address, quantity);
        address += 1;
    }
    for(i = 0; i < 8; i++) {
        quantity = fastenerSetPerStep[i];
        write_eeprom(address, quantity);
        address += 1;
    }
    
    quantity = nAssembly;
    write_eeprom(address, quantity);
    address += 1;
    
    quantity = emergency;
    write_eeprom(address, quantity);
    
    address = 0xFF;
    quantity = (int)read_eeprom(address) + 1;
    write_eeprom(address, quantity);
}

void load_data(const unsigned int numLog) {
    // numLog is which log you're accessing. Most recent = 0, next = 1, etc.
    // data stored into variable load[19]
    
    unsigned short address = 1 + (numLog * 38);
    
    for(unsigned int i = 0; i < 39; i++) {
        load[i] = (int)read_eeprom(address);
        address += 1;
    }
    load[39] = (int)read_eeprom(0xFF);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Circuitry Integration Functions">

void DCbolts(void) {
    TRISCbits.RC7 = 0;
    SPEN = 1;
    __delay_ms(100);
    for(i = 0; i < 100; i++) {
        SPEN = 1;
        __delay_ms(1);
        SPEN = 0;
        __delay_ms(3);
    }
    SPEN = 1;
    TRISCbits.RC7 = 1;
}

void DCwashers(void) {
    // Output is RC0
    LATEbits.LATE0 = 1;
    __delay_ms(100);
    for(i = 0; i < 100; i++) {
        LATEbits.LATE0 = 1;
        __delay_ms(1);
        LATEbits.LATE0 = 0;
        __delay_ms(3);
    }
}

void stepperForward(void) {
        // Stepper:
    for(unsigned int u = 0; u < 20; u++) {
        
        LATBbits.LATB2 = 1;
        __delay_ms(DELAY_TIME);
        LATBbits.LATB0 = 0;
        __delay_ms(DELAY_TIME);
        LATBbits.LATB1 = 1;
        __delay_ms(DELAY_TIME);
        LATBbits.LATB2 = 0;
        __delay_ms(DELAY_TIME);
        LATBbits.LATB3 = 1;
        __delay_ms(DELAY_TIME);
        LATBbits.LATB1 = 0;
        __delay_ms(DELAY_TIME);
        LATBbits.LATB0 = 1;
        __delay_ms(DELAY_TIME);
        LATBbits.LATB3 = 0;
        __delay_ms(DELAY_TIME);
    }
    LATB = 0x00;
}
void stepperReverse(void) {
    // Rotates backwards for 45 degrees
    for(unsigned int u = 0; u < 20; u++) {
        LATBbits.LATB3 = 1;
        __delay_ms(DELAY_TIME);
        LATB = 0x00;
        LATBbits.LATB1 = 1;
        __delay_ms(DELAY_TIME);
        LATB = 0x00;
        LATBbits.LATB2 = 1;
        __delay_ms(DELAY_TIME);
        LATB = 0x00;
        LATBbits.LATB0 = 1;
        __delay_ms(DELAY_TIME);
        LATB = 0x00;
    }
}
void solenoidB(void) {
    // Solenoid:
    // Input from RA0, output to RB4
    LATBbits.LATB4 = 0;
    LATBbits.LATB4 = 1;
    __delay_ms(SOLENOID_DELAY);
    LATBbits.LATB4 = 0;
}
void solenoidN(void) {
    // Solenoid:
    // Input from RA0, output to RB5
    LATBbits.LATB5 = 0;
    LATBbits.LATB5 = 1;
    __delay_ms(SOLENOID_DELAY);
    LATBbits.LATB5 = 0;
}
void solenoidS(void) {
    // Solenoid:
    // Input from RA0, output to RB6
    LATBbits.LATB6 = 0;
    LATBbits.LATB6 = 1;
    __delay_ms(SOLENOID_DELAY);
    LATBbits.LATB6 = 0;
}
void solenoidW(void) {
    // Solenoid:
    // Input from RA0, output to RB7
    LATBbits.LATB7 = 0;
    LATBbits.LATB7 = 1;
    __delay_ms(SOLENOID_DELAY);
    LATBbits.LATB7 = 0;
}
void colorSensor(void) {
    // Frequency Recognizer
    // RC2 is input
    TRISC = 0b10000100;

    // Enable bits:
    INTCONbits.PEIE = 1;
    PIE2bits.CCP2IE = 1;
    PIE1bits.TMR1IE = 1;


    di();
    T1CONbits.TMR1CS = 0;
    T1CONbits.T1SYNC = 0;
    T1CONbits.T1CKPS1 = 1;
    T1CONbits.T1CKPS0 = 1;
    T1CONbits.T1RUN = 1;

    // Start bit of timer1
    TMR1H:TMR1L = 0x00;

    CCP1CONbits.CCP1M0 = 1;
    CCP1CONbits.CCP1M1 = 1;
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M3 = 0;
    T1CONbits.TMR1ON = 1;
    ei();

    unsigned int count;
    count = 0;
    //for(unsigned int m = 0; m < 10; m++) {
    for(unsigned int m = 0; m < 7; m++) {
        while((TMR1H < 0b11111111)  || (TMR1L < 0b11111111)) {
//        while((TMR1H < 0b01111111) || (TMR1L < 0b01111111)) {
        // while(!PIR1bits.TMR1IF) {
            if(CCP1IF) {
                count += 1;
                CCP1IF = 0;
            }
        }
        if(count < 0) {
            count *= -1;
        }
    }
    if(count > 1200) {
        color = 1;
    }
    else {
        color = 0;
    }
    TRISC = 0b10000000;
}
void DCmotor(unsigned int value) {
    // RC1 and RE0 are outputs
    TRISE = 0b00000100;
    LATCbits.LATC1 = 0;
    LATCbits.LATC0 = 0;
    if(value == 1) {
//        LATCbits.LATC1 = 0;
//        LATEbits.LATE0 = 1;
        LATCbits.LATC1 = 0;
        LATCbits.LATC0 = 1;
    }
    else if(value == 2) {
        LATCbits.LATC1 = 1;
        LATCbits.LATC0 = 0;
    }
    else if(value == 0) {
        LATCbits.LATC1 = 0;
        LATCbits.LATC0 = 0;
    }
}
unsigned int microswitchB(void) {
    if(PORTAbits.RA0) {
        return 1;
    }
    else {
        return 0;
    }
}
unsigned int microswitchN(void) {
    // RA1 is input
    //TRISA = 0b00000101;
    if(PORTAbits.RA1) {
        return 1;
    }
    else {
        return 0;
    }
}
unsigned int microswitchS(void) {
    // RA2 is input
    //TRISA = 0b00001001;
    if(PORTAbits.RA2) {
        return 1;
    }
    else {
        return 0;
    }
}
unsigned int microswitchW(void) {
    // RA3 is input
    //TRISA = 0b00010001;
    if(PORTAbits.RA3) {
        return 1;
    }
    else {
        return 0;
    }
}
void vibrationBN(unsigned int value) {
    // outputs is RA4
    if(value) {
        LATAbits.LATA4 = 1;
    }
    else {
        LATAbits.LATA4 = 0;
    }
}
void vibrationSW(unsigned int value) {
    // output is RA5
    if(value) {
        LATAbits.LATA5 = 1;
    }
    else {
        LATAbits.LATA5 = 0;
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="PC-PIC Communication">
// https://openlabpro.com/guide/uart-interfacing-with-pic-microcontroller/

// Notes:
    // \r\r outputs new lines
    // \r\f outputs some up arrow

void initUART(unsigned int value) {
    TRISCbits.RC6 = 0; // transmit
    
    BRGH = 1; // high baud rate
    SPBRG = value;
    // 251 worked for SPBRG value
    // SPBRG = (_XTAL_FREQ/(64 * baud)) - 1; // 9600 bits per second for 40 MHz clock
    SYNC = 0; // asynchronous
    SPEN = 1; // enable serial port pins
    //CREN = 1; // enable receive
    TXIE = 0; // disable transmit interrupts
    //RCIE = 0; // disable receive interrupts
    TX9 = 0; // 8/9 bit transmit
    //RX9 = 0; // 8/9 bit receive
    TXEN = 1; // enable transmission
}
void transmitCharUART(unsigned char input) {
    TXREG = input; // prepare transmit data
    while(!TRMT); // wait for transmit to not be busy
}
void transmitStringUART(unsigned char input[]) {
    // to create newline, send "\r\n" at the end of your string
    while(*input) {
        transmitCharUART(*input++);
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="GLCD Text Output">
void drawPixel(unsigned int XS, unsigned int YS) {
    // Outputs black 3x3 pixel at target location (XS, YS)
    unsigned int XE = XS + 3;
    unsigned int YE = YS + 3;
    glcdDrawRectangle(XS, XE, YS, YE, BLACK);
}
unsigned int selectLetter(unsigned char letter) {
    unsigned int res;
    unsigned int temp = letter - '0';
    if((temp == 0) || (temp == 1) || (temp == 2) || (temp == 3) || (temp == 4) || (temp == 5) || (temp == 6) || (temp == 7) || (temp == 8) || (temp == 9)) {
        res = temp;
    }
    else {
        res = letter - 55;
    }
    return res;
}
void GLCDtext(unsigned char string[9], unsigned int lineNumber) {
    // Max number of lines = 6
    // Max 7 letters including spaces per line
    // Applicable characters(not including space): 0 1 2 3 4 5 6 A B C D E F G H I J K L M N O P Q R S T U V W X Y 
    
    unsigned int horzLocation;
    unsigned int vertLocation;
    unsigned int tempIndex;
    for(unsigned int j = 0; j < 9; j++) {
        tempIndex = selectLetter(string[j]);
        // Testing to be removed: 
        if(string[j] == ',') {
            break;
        }
        vertLocation = 1 + lineNumber + (lineNumber * 18);
        horzLocation = (j * 12) + (j * 2) + 1;
        
        for(unsigned int k = 0; k < 6; k++) {
            for(unsigned int l = 0; l < 4;l ++) {
                if(GLCDpixel[tempIndex][((k*4) + l)] == '1') {
                    drawPixel(horzLocation, vertLocation);
                }
                horzLocation += 3;
            }
            vertLocation += 3;
            horzLocation = (j * 12) + (j * 2) + 1;
       }
    }
}
void GLCDprogressSetup(void) {
    unsigned int lineNumber = 5;
    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, WHITE);
    GLCDtext("OPERATION", 0);
    GLCDtext("IN ACTION", 1);
    unsigned int vertLocation = 1 + lineNumber + (lineNumber * 18);
    glcdDrawRectangle(0, GLCD_SIZE_HORZ, vertLocation, (vertLocation + 3), BLACK);
    glcdDrawRectangle(0, GLCD_SIZE_HORZ, (vertLocation + 18), (vertLocation + 21), BLACK);
    glcdDrawRectangle(0, 3, (vertLocation + 3), (vertLocation + 18), BLACK);
    glcdDrawRectangle((GLCD_SIZE_HORZ - 3), GLCD_SIZE_HORZ, (vertLocation + 3), (vertLocation + 18), BLACK);
    I2C_Master_Init(100000);
}
void GLCDprogress(void) {
    unsigned int lineNumber = 5;
    unsigned int vertLocation = 1 + lineNumber + (lineNumber * 18);
    glcdDrawRectangle((3 + step), (4 + step), (vertLocation + 3), (vertLocation + 18), GREEN);
    step += 1;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Other Functions">
unsigned int ASCIItoInt(unsigned char x) {
    unsigned char tempWord[2];
    sprintf(tempWord, "%02x", x);
    unsigned int res = ((tempWord[0] - '0') * 10) + (tempWord[1] - '0');
    return res;
}
// </editor-fold>

void main(void) {
    // <editor-fold defaultstate="collapsed" desc="Initialization">

    // <editor-fold defaultstate="collapsed" desc="Machine Configuration">
    /********************************* PIN I/O ********************************/
    /* Write outputs to LATx, read inputs from PORTx. Here, all latches (LATx)
     * are being cleared (set low) to ensure a controlled start-up state. */  
    LATA = 0x00;
    LATB = 0x00; 
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;

    /* After the states of LATx are known, the data direction registers, TRISx
     * are configured. 0 --> output; 1 --> input. Default is  1. */
    TRISA = 0b00001111;
    TRISB = 0xFF;
    
// for GLCD and RTC:    
    TRISC = 0b10000100; /* RC3 is SCK/SCL (SPI/I2C),
                         * RC4 is SDA (I2C),
                         * RC5 is SDA (SPI),
                         * RC6 and RC7 are UART TX and RX, respectively. */
    TRISD = 0b00000001; /* RD0 is the GLCD chip select (tri-stated so that it's
                         * pulled up by default),
                         * RD1 is the GLCD register select,
                         * RD2 is the character LCD RS,
                         * RD3 is the character LCD enable (E),
                         * RD4-RD7 are character LCD data lines. */
    TRISE = 0b00000100; /* RE2 is the SD card chip select (tri-stated so that
                         * it's pulled up by default).
                         * Note that the upper 4 bits of TRISE are special 
                         * control registers. Do not write to them unless you 
                         * read §9.6 in the datasheet */
    
    /************************** A/D Converter Module **************************/
    ADCON0 = 0x00;  // Disable ADC
    ADCON1 = 0b00001111; // Set all A/D ports to digital (pg. 222)
    // </editor-fold>
    
    nRBPU = 0; // Enable Port B Internal Pull Up Resistors
    
    INT1IE = 1; // Enable RB1 (keypad data available) interrupt
    
    ei(); // Enable all interrupts
    initLCD(); // Initialize LCD
    
    // <editor-fold defaultstate="collapsed" desc="RTC Initialization">
    I2C_Master_Init(100000); // Initialize I2C Master with 100 kHz clock
    
     /* Set the time in the RTC.
     * 
     * To see the RTC keep time, comment this line out after programming the PIC
     * directly before with this line included. */
    //RTC_setTime();
    

    // </editor-fold>    

    initGLCD(); // Initialize GLCD
    
    I2C_Master_Init(100000);
    init_eeprom();
    initUART(251);
    
    // Add debugging as necessary. 
    
    // GLCD rotation variable declared here just in case it is needed later.
    unsigned char rotation = 0, x = 0, y = 0;

    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="Introductory Message">
    // Have this all work at power on, not just at initial program. 
    spiInit(16);
    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
    GLCDtext(" STANDBY ", 2);
    GLCDtext("  MODE   ", 3);
    I2C_Master_Init(100000);
    unsigned char numShifts = 0; 
    loopFlag = 0;
    __lcd_display_control(1, 0, 0);

    lcd_set_cursor(3, 0);
    printf("Welcome to");
    lcd_set_cursor(3, 1);
    printf("Team 25's");
    for(unsigned int i = 0; i < 100; i++) {
        if(loopFlag) {
            break;
        }
        __delay_ms(25);
    }
    if(!loopFlag) {
        lcd_set_cursor(LCD_SIZE_HORZ + 2, 0);
        printf("Machine User");
        lcd_set_cursor(LCD_SIZE_HORZ + 3, 1);
        printf("Interface");
        
        // Shifting motion:
        numShifts = LCD_SIZE_HORZ;
        for(unsigned char i = 0; i < numShifts; i++){
            if(loopFlag) {
                break;
            }
            lcd_shift_display(1, LCD_LEFT);
            __delay_ms(100);
        }
        for(unsigned int i = 0; i < 100; i++) {
            if(loopFlag) {
                break;
            }
            __delay_ms(25);
        }
    }
    
    // </editor-fold>
    
    while(1) {
    // <editor-fold defaultstate="collapsed" desc="Standby Mode">   
        
        // <editor-fold defaultstate="collapsed" desc="Initial Menu">
        __delay_ms(100);
        __lcd_display_control(1, 0, 0);
        __lcd_clear();
        lcd_set_cursor(0, 0);
        printf("B:Start N:Time");
        __lcd_newline();
        printf("S:Logs  W:Debug");
        
        // Loop check:
        loopFlag = 0;
        
        // Key press wait:
        while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
        unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
        while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
        Nop();  // Apply breakpoint here to prevent compiler optimizations
        
        unsigned char temp = keys[keypress];
        vibrationBN(0);
        vibrationSW(0);
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="D - Debug Menu">
        if(temp == 'D') {
            spiInit(16);
            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
            GLCDtext("  DEBUG  ", 2);
            GLCDtext("  MODE   ", 3);
            I2C_Master_Init(100000);
            unsigned int debugFlag = 1;
            __lcd_clear();
            __lcd_home();
            printf("1=Stp 2=Sol 3=Se");
            __lcd_newline();
            printf("B=Vib 4=DC 5=Nxt");
            while(debugFlag) {
                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                Nop();  // Apply breakpoint here to prevent compiler optimizations
                unsigned char temp = keys[keypress];
                if(temp == '1') {
                    __lcd_clear();
                    __lcd_home();
                    printf("Stepper Test.");
                    __lcd_newline();
                    printf("1/2:For 3/A:Rev");
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    unsigned char temp = keys[keypress];
                    
                    
                    LATEbits.LATE1 = 1;
                    TRISB = 0x00;
                    
                    for(i = 0; i < 12; i++) {
                        stepperForward();
                    }
                    for(i = 0; i < 24; i++) {
                        stepperReverse();
                    }
                    LATEbits.LATE1 = 0;
                    TRISB = 0xFF;
                    __lcd_clear();
                    __lcd_home();
                    printf("Press to");
                    __lcd_newline();
                    printf("Continue.");
                    
                    
                    
                    
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    __lcd_clear();
                    __lcd_home();
                    printf("1=Stp 2=Sol 3=Se");
                    __lcd_newline();
                    printf("B=Vib 4=DC 5=Nxt");
                }
                else if(temp == '2') {
                    __lcd_clear();
                    __lcd_home();
                    printf("Solenoid Test.");
                    __lcd_newline();
                    printf("1:B 2:N 3:S B:W");
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    unsigned char temp = keys[keypress];
                    
                    
                    LATEbits.LATE1 = 1;
                    TRISB = 0x00;
                    
                    
                    if(temp == '1') {
                        solenoidB();
                    }
                    else if(temp == '2') {
                        solenoidN();
                    }
                    else if(temp == '3') {
                        solenoidS();
                    }
                    else if(temp == 'A') {
                        solenoidW();
                    }
                    __lcd_clear();
                    __lcd_home();
                    printf("Press to");
                    __lcd_newline();
                    printf("Continue.");
                    
                    
                    LATEbits.LATE1 = 0;
                    TRISB = 0xFF;
                    
                    
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    __lcd_clear();
                    __lcd_home();
                    printf("1=Stp 2=Sol 3=Se");
                    __lcd_newline();
                    printf("B=Vib 4=DC 5=Nxt");
                }
                else if(temp == '3') {
                    __lcd_clear();
                    __lcd_home();
                    printf("Count: ");
//                    for(i = 0; i < 8; i++) {
//                        stepperReverse();
//                    }
                    color = 0;
                    colorSensor();
                    
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    __lcd_clear();
                    __lcd_home();
                    printf("1=Stp 2=Sol 3=Se");
                    __lcd_newline();
                    printf("B=Vib 4=DC 5=Nxt");
                }
                else if(temp == 'A') {
                    __lcd_clear();
                    __lcd_home();
                    printf("Vibration Test.");
                    __lcd_newline();
                    printf("1=BN 2=SW");
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    unsigned char temp = keys[keypress];
                    
                    
                    LATEbits.LATE1 = 1;
                    TRISB = 0x00;
                    
                    
                    if(temp == '1') {
                        vibrationBN(1);
                        __delay_ms(5000);
                        vibrationBN(0);
                    }
                    else if(temp == '2') {
                        vibrationSW(1);
                        __delay_ms(5000);
                        vibrationSW(0);
                    }
                    
                    
                    LATEbits.LATE1 = 0;
                    TRISB = 0xFF;
                    
                                       
                    __lcd_clear();
                    __lcd_home();
                    printf("1=Stp 2=Sol 3=Se");
                    __lcd_newline();
                    printf("B=Vib 4=DC 5=Nxt");
                }
                else if(temp == '4') {
                    __lcd_clear();
                    __lcd_home();
                    printf("DC Test.");
                    __lcd_newline();
                    printf("1:B 2:W 3:DC");
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    unsigned char temp = keys[keypress];
                    
                    
                    LATEbits.LATE1 = 1;
                    TRISB = 0x00;
                    
                    if(temp == '1') {
                        for(unsigned int k = 0; k < 300; k++) {
                            DCbolts();
                            __delay_ms(750);
                            if(k == 10) {
                                break;
                            }
                        }
                    }
                    else if(temp == '2') {
                        DCwashers();
                    }
                    else if(temp == '3') {
                        DCmotor(1);
                        __delay_ms(2000);
                        DCmotor(0);
                        DCmotor(2);
                        __delay_ms(2000);
                        DCmotor(0);
                    }
                    
                    
                    LATEbits.LATE1 = 0;
                    TRISB = 0xFF;
                    
                    
                    __lcd_clear();
                    __lcd_home();
                    printf("1=Stp 2=Sol 3=Se");
                    __lcd_newline();
                    printf("B=Vib 4=DC 5=Nxt");
                }
                else if(temp == '5') {
                    __lcd_clear();
                    __lcd_home();
                    printf("6=Mic N=GLCD");
                    __lcd_newline();
                    printf("7=URT 8=DEV 9=Pv");
                }
                else if(temp == '6') {
                    __lcd_clear();
                    __lcd_home();
                    unsigned int microFlag = 0;
                    printf("Press B N S W");
                    
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    unsigned char temp = keys[keypress];
                    
                    
                    LATEbits.LATE1 = 1;
                    TRISB = 0x00;
                    __lcd_clear();
                    
                    if(temp == 'A') {
                        solenoidB();
                        for(i = 0; i < MICRO_LOOPS; i++) {
                            if(microswitchB()) {
                                microFlag = 1;
                                break;
                            }
                        }
                        __lcd_clear();
                        if(microFlag) {
                            printf("Found.");
                        }
                        else {
                            printf("Absent.");
                        }
                        microFlag = 0;
                    }
                    else if(temp == 'B') {
                        solenoidN();
                        for(i = 0; i < MICRO_LOOPS; i++) {
                            if(microswitchN()) {
                                microFlag = 1;
                                break;
                            }
                        }
                        __lcd_clear();
                        if(microFlag) {
                            printf("Found.");
                        }
                        else {
                            printf("Absent.");
                        }
                        microFlag = 0;
                    }
                    else if(temp == 'C') {
                        solenoidS();
                        for(i = 0; i < MICRO_LOOPS; i++) {
                            if(microswitchS()) {
                                microFlag = 1;
                                break;
                            }
                        }
                        __lcd_clear();
                        if(microFlag) {
                            printf("Found.");
                        }
                        else {
                            printf("Absent.");
                        }
                        microFlag = 0;
                    }
                    else if(temp == 'D') {
                        solenoidW();
                        for(i = 0; i < MICRO_LOOPS; i++) {
                            if(microswitchW()) {
                                microFlag = 1;
                                break;
                            }
                        }
                        __lcd_clear();
                        if(microFlag) {
                            printf("Found.");
                        }
                        else {
                            printf("Absent.");
                        }
                        microFlag = 0;
                    }
                    
                    LATEbits.LATE1 = 0;
                    TRISB = 0xFF;
                    
                    __lcd_newline();
                    printf("Press.");
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    __lcd_clear();
                    __lcd_home();
                    printf("6=Mic N=GLCD");
                    __lcd_newline();
                    printf("7=URT 8=DEV 9=Pv");
                }
                else if(temp == 'B') {
                    spiInit(16);
                    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
                    GLCDtext("012345678", 0);
                    GLCDtext("9ABCDEFGH", 1);
                    GLCDtext("IJKLMNOPQ", 2);
                    GLCDtext("RSTUVWX Y", 3);
                    GLCDtext("Z", 4);
                    I2C_Master_Init(100000);
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    spiInit(16);
                    GLCDprogressSetup();
                    I2C_Master_Init(100000);
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizationss
                    for(i = 0; i < 122; i++) {
                        spiInit(16);
                        GLCDprogress();
                        I2C_Master_Init(100000);
                        while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                        while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                        Nop();  // Apply breakpoint here to prevent compiler optimizations
                    }
                    spiInit(16);
                    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, GREEN);
                    GLCDtext("OPERATION", 0);
                    GLCDtext("SUCCESS  ", 1);
                    I2C_Master_Init(100000);
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    spiInit(16);
                    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
                    I2C_Master_Init(100000);
                    step = 0;
                    __lcd_clear();
                    __lcd_home();
                    printf("6=Mic N=GLCD");
                    __lcd_newline();
                    printf("7=URT 8=DEV 9=Pv");
                }
                else if(temp == '7') {
                    __lcd_clear();
                    __lcd_home();
                    printf("UART Test.");
//                    for(unsigned int k = 251; k < 257; k++) {
//                        initUART(k);
//                        __delay_ms(500);
//                        transmitStringUART("Hello\r\n");
//                        __delay_ms(500);
//                    }
                    transmitStringUART("Testing Transmission.\r\n");
                    __lcd_clear();
                    printf("Test Complete");
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    __lcd_clear();
                    __lcd_home();
                    printf("6=Mic N=GLCD");
                    __lcd_newline();
                    printf("7=URT 8=DEV 9=Pv");
                }
                else if(temp == '8') {
                    
                    
                    
                    __lcd_display_control(1, 1, 1);
                    __lcd_clear();
                    lcd_set_cursor(0, 0);
                    printf("*:erase  #:enter");
                    lcd_set_cursor(0, 1);
                    unsigned int codeFlag = 1;
                    unsigned char code[5] = { ' ' };
                    unsigned int cursorLocation = 0;

                    // Test code to insert to demonstrate typing functionality and responsiveness:
                    while(codeFlag == 1) {
                        while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                        unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                        while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                        Nop();  // Apply breakpoint here to prevent compiler optimizations

                        unsigned char temp = keys[keypress];

                        if(temp == '*' && cursorLocation != 0) {
                            cursorLocation -= 1;
                            lcd_shift_cursor(1, 0);
                            putch(' ');
                            lcd_shift_cursor(1, 0);
                            code[cursorLocation] = code[cursorLocation + 1];
                        } else if(temp == '#') {
                            codeFlag = 0;
                        } else if(temp == 'A') {
                            lcd_set_cursor(cursorLocation, 1);
                            putch('B');
                            code[cursorLocation] = 'B';
                            cursorLocation += 1;
                        } else if(temp == 'B') {
                            lcd_set_cursor(cursorLocation, 1);
                            putch('N');
                            code[cursorLocation] = 'N';
                            cursorLocation += 1;
                        } else if(temp == 'C') {
                            lcd_set_cursor(cursorLocation, 1);
                            putch('S');
                            code[cursorLocation] = 'S';
                            cursorLocation += 1;
                        } else if(temp == 'D') {
                            lcd_set_cursor(cursorLocation, 1);
                            putch('W');
                            code[cursorLocation] = 'W';
                            cursorLocation += 1;
                        } else {
                            lcd_set_cursor(cursorLocation, 1);
                            putch(temp);
                            code[cursorLocation] = temp;
                            cursorLocation += 1;
                        }
                    }
                    __lcd_clear();
                    lcd_set_cursor(0, 0);
                    codeFlag = 0;
                    for(unsigned int i = 0; i < 5; i++) {                
                        if(code[i] != password1[i]) {
                            codeFlag = 1;
                            break;
                        }
                    }
                    if(codeFlag == 0) {
                        __lcd_display_control(1, 0, 0);
                        unsigned int i = 0;

                        loopFlag = 0;             

                        for(i = 0; i < sizeof(jeremiah)/sizeof(jeremiah[0]); i += 2) {
                            if(loopFlag) {
                                break;
                            }
                            __lcd_clear();
                            lcd_set_cursor(0, 0);
                            for(unsigned int j = 0; j < 16; j++) {
                                if(loopFlag) {
                                    break;
                                }
                                lcd_set_cursor(j, 0);
                                printf("%c", jeremiah[i][j]);
                                lcd_set_cursor(j, 1);
                                printf("%c", jeremiah[i + 1][j]);
                            }
                            for(unsigned int k = 0; k < 10; k++) {
                                if(loopFlag) {
                                    break;
                                }
                                __delay_ms(600);
                            }
                        }
                        if(loopFlag == 0) {
                            while(1) {
                                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                                unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                                Nop();  // Apply breakpoint here to prevent compiler optimizations

                                unsigned char temp = keys[keypress];
                                if(temp == '#') {
                                    break;
                                }
                            }
                        }
                        else{
                            loopFlag = 0;
                        }
                        codeFlag = 1;
                    }
                    __lcd_clear();
                    __lcd_home();
                    printf("6=Mic N=GLCD");
                    __lcd_newline();
                    printf("7=URT 8=DEV 9=Pv");
                }
                else if(temp == '9') {
                    
                    
                    LATEbits.LATE1 = 1;
                    TRISB = 0x00;
                    
                    for(i = 0; i < 52; i++) { // 8 steps here
                        stepperReverse();
                    }
                    __lcd_clear();
                    __lcd_home();
                    printf("1=Stp 2=Sol 3=Se");
                    __lcd_newline();
                    printf("B=Vib 4=DC 5=Nxt");
                    
                    
                    
                    LATEbits.LATE1 = 0;
                    TRISB = 0xFF;
                }
                else if(temp == '*') {
                    debugFlag = 0;
                }
            }
        }
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="B - Date and Time">
        else if(temp == 'B') {
            spiInit(16);
            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
            GLCDtext(" DATE AND", 1);
            GLCDtext("  TIME   ", 2);
            I2C_Master_Init(100000);
            __lcd_clear();
            loopFlag = 0;
            while(!loopFlag){
                /* Reset RTC memory pointer. */
                I2C_Master_Start(); // Start condition
                I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
                I2C_Master_Write(0x00); // Set memory pointer to seconds
                I2C_Master_Stop(); // Stop condition

                /* Read current time. */
                I2C_Master_Start(); // Start condition
                I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
                for(i = 0; i < 6; i++){
                    time[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
                }
                time[6] = I2C_Master_Read(NACK); // Final Read with NACK
                I2C_Master_Stop(); // Stop condition
                /* Print received data to LCD. */
                __lcd_home();
                printf("Date: %02x/%02x/%02x", time[6],time[5],time[4]); // Print date in YY/MM/DD
                __lcd_newline();
                printf("Time: %02x:%02x:%02x", time[2],time[1],time[0]); // HH:MM:SS
                __delay_ms(1000);
            }
            loopFlag = 0;
        }
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="A - Operation Setup">
        else if(temp == 'A') {
            numBremaining = 0;
            numNremaining = 0;
            numSremaining = 0;
            numWremaining = 0;
            numB = 0;
            numN = 0;
            numS = 0;
            numW = 0;
            nPerStep = 0;
            nAssembly = 0;
            spiInit(16);
            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
            GLCDtext("OPERATION", 1);
            GLCDtext("  SETUP  ", 2);
            I2C_Master_Init(100000);

            unsigned int codeFlag = 1;
            unsigned int inputFlag = 1;
            __lcd_clear();
            __lcd_display_control(1, 1, 1);
            lcd_set_cursor(0, 0);
            lcd_set_cursor(0, 1);
            unsigned char code[16] = { 'X' };
            for(i = 0; i < 16; i++) {
                code[i] = 'X';
            }
            unsigned int cursorLocation = 0;
            
            
            
            // <editor-fold defaultstate="collapsed" desc="Number of Assembly Steps">
            __lcd_clear();
            lcd_set_cursor(0, 0);
            printf("Number of Steps:");
            codeFlag = 1;
            for(i = 0; i < 16; i++) {
                code[i] = 'X';
            }
            cursorLocation = 0;
            __lcd_display_control(1, 1, 1);
            __lcd_newline();
            while(codeFlag) {
                unsigned int secondFlag = 1;
                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                Nop();  // Apply breakpoint here to prevent compiler optimizations
                unsigned char temp = keys[keypress];
                
                if((temp == '*') && (cursorLocation != 0)) {
                    if(cursorLocation >= 16) {
                        lcd_shift_cursor(1, 1);
                    }
                    cursorLocation -= 1;
                    lcd_shift_cursor(1, 0);
                    putch(' ');
                    lcd_shift_cursor(1, 0);
                    code[cursorLocation] = 'X';
                    nAssembly = 0;
                } else if(temp == '#' && cursorLocation != 0) {
                    codeFlag = 0;
                } else if(cursorLocation < 16) {
                    lcd_set_cursor(cursorLocation, 1);
                    if((temp != '*') && (temp != '#') && (cursorLocation < 16)) {
                        if((temp == 'A') || (temp == 'B') || (temp == 'C') || (temp == 'D')) {
                            nAssembly = 1;
                            if(temp == 'A') {
                                putch('B');
                                temp = '1';
                            }
                            else if(temp == 'B') {
                                putch('N');
                                temp = '1';
                            }
                            else if(temp == 'C') {
                                putch('S');
                                temp = '1';
                            }
                            else if(temp == 'D') {
                                putch('W');
                                temp = '1';
                            }
                        }
                        else {
                            putch(temp);
                            nAssembly = temp - '0';
                        }
                        secondFlag = 0;
                    }
                    if(cursorLocation >= 15) {
                        lcd_set_cursor(cursorLocation, 1);
                    }
                    if(secondFlag == 0) {
                        code[cursorLocation] = temp;
                        cursorLocation += 1;
                    }
                }
                if(!codeFlag) {
                    if(code[1] != 'X') {
                        codeFlag = 1;
                    }
                    else {
                        if((code[0] != '4') && (code[0] != '5') && (code[0] != '6') && (code[0] != '7') && (code[0] != '8')) {
                            codeFlag = 1;
                        }
                    }
                    if(codeFlag) {
                        nAssembly = 0;
                        for(i = 0; i < 16; i++) {
                            code[i] = 'X';
                        }
                        __lcd_clear();
                        printf("Invalid Input.");
                        __lcd_newline();
                        printf("Press to Retry.");
                        while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                        while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                        Nop();  // Apply breakpoint here to prevent compiler optimizations
                        __lcd_clear();
                        __lcd_display_control(1, 1, 1);
                        lcd_set_cursor(0, 0);
                        printf("Number of Steps:");
                        lcd_set_cursor(0, 1);
                        for(i = 0; i < 16; i++) {
                            code[i] = 'X';
                        }
                        cursorLocation = 0;
                    }
                    else { 
                        nAssembly = code[0] - '0';
                    }
                }
            }
            codeFlag = 1;
            cursorLocation = 0;
            if(nAssembly == 8) {
                eightFlag = 1;
            }
            // </editor-fold>

            // <editor-fold defaultstate="collapsed" desc="Fastener User Input">
            unsigned int bound = nAssembly * 2;
            for(unsigned int l = 0; l < bound; l += 2) {
                numB = 0;
                numN = 0;
                numS = 0;
                numW = 0;
                unsigned int codeFlag = 1;
                __lcd_clear();
                __lcd_display_control(1, 1, 1);
                lcd_set_cursor(0, 0);
                unsigned int compartments[];
                if(nAssembly == 4) {
                    for(unsigned int j = 0; j < 4; j++) {
                        compartments[j] = assembly4[j];
                    }
                }
                else if(nAssembly == 5) {
                    for(unsigned int j = 0; j < 5; j++) {
                        compartments[j] = assembly5[j];
                    }
                }
                else if(nAssembly == 6) {
                    for(unsigned int j = 0; j < 6; j++) {
                        compartments[j] = assembly6[j];
                    }
                }
                else if(nAssembly == 7) {
                    for(unsigned int j = 0; j < 7; j++) {
                        compartments[j] = j + 1;
                    }
                }
                else if(nAssembly == 8) {
                    for(unsigned int j = 0; j < 8; j++) {
                        compartments[j] = j + 1;
                    }
                }
                printf("C%d Fastener Set:", compartments[l/2]);
                lcd_set_cursor(0, 1);
                unsigned char code[16] = { 'X' };
                for(i = 0; i < 16; i++) {
                    code[i] = 'X';
                }
                unsigned int cursorLocation = 0;
                while(codeFlag) {
                    unsigned int secondFlag = 1;
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    unsigned char temp = keys[keypress];

                    if((temp == '*') && (cursorLocation != 0)) {
                        if(cursorLocation >= 16) {
                            lcd_shift_cursor(1, 1);
                        }
                        cursorLocation -= 1;
                        lcd_shift_cursor(1, 0);
                        putch(' ');
                        lcd_shift_cursor(1, 0);
                        if(code[cursorLocation] == 'B') {
                            numB -= 1;
                        }
                        else if(code[cursorLocation] == 'N') {
                            numN -= 1;
                        }
                        else if(code[cursorLocation] == 'S') {
                            numS -= 1;
                        }
                        else if(code[cursorLocation] == 'W') {
                            numW -= 1;
                        }
                        code[cursorLocation] = 'X';
                    } else if(temp == '#' && cursorLocation != 0) {
                        codeFlag = 0;
                    } else if(cursorLocation < 16 && (temp != '#') && (temp != '*')) {
                        lcd_set_cursor(cursorLocation, 1);
                        if((temp == 'A') && (cursorLocation < 16)) {
                            temp = 'B';
                            putch(temp);
                            numB += 1;
                            secondFlag = 0;
                        }
                        else if((temp == 'B') && (cursorLocation < 16)) {
                            temp = 'N';
                            putch(temp);
                            numN += 1;
                            secondFlag = 0;
                        }
                        else if((temp == 'C') && (cursorLocation < 16)) {
                            temp = 'S';
                            putch(temp);
                            numS += 1;
                            secondFlag = 0;
                        }
                        else if((temp == 'D') && (cursorLocation < 16)) {
                            temp = 'W';
                            putch(temp);
                            numW += 1;
                            secondFlag = 0;
                        }
                        else {
                            putch(temp);
                            secondFlag = 0;
                        }
                        if(cursorLocation >= 15) {
                            lcd_set_cursor(cursorLocation, 1);
                        }
                        if(secondFlag == 0) {
                            code[cursorLocation] = temp;
                            cursorLocation += 1;
                        }
                    }
                    // <editor-fold defaultstate="collapsed" desc="Password Check">
                    if(!codeFlag) {
                        if(code[4] == 'X') {
                            for(i = 0; i < 4; i++) {
                                if(code[i] != fast1[i]) {
                                    break;
                                }
                                if(i == 3) {
                                    codeFlag = 1;
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast2[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast3[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast4[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast5[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast6[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast7[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast8[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast9[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast10[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast11[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast12[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast13[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast14[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast15[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast16[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast17[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast18[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast19[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                            if(!codeFlag) {
                                for(i = 0; i < 4; i++) {
                                    if(code[i] != fast20[i]) {
                                        break;
                                    }
                                    if(i == 3) {
                                        codeFlag = 1;
                                    }
                                }
                            }
                        }
                        // </editor-fold>
                        if(!codeFlag) {
                            numB = 0;
                            numN = 0;
                            numS = 0;
                            numW = 0;
                            for(i = 0; i < 16; i++) {
                                code[i] = 'X';
                            }
                            __lcd_clear();
                            printf("Invalid Input.");
                            __lcd_newline();
                            printf("Press to Retry.");
                            while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                            while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                            Nop();  // Apply breakpoint here to prevent compiler optimizations
                            __lcd_clear();
                            __lcd_display_control(1, 1, 1);
                            lcd_set_cursor(0, 0);
                            printf("C%d Fastener Set", compartments[l]);
                            lcd_set_cursor(0, 1);
                            for(i = 0; i < 16; i++) {
                                code[i] = 'X';
                            }
                            cursorLocation = 0;
                            codeFlag = 1;
                        }
                        else {
                            codeFlag = 0;
                        }
                    }
                }
                fastenerSet[l] = (numB * 10) + numN;
                fastenerSet[l + 1] = (numS * 10) + numW;
            }
            // </editor-fold>
            
            // <editor-fold defaultstate="collapsed" desc="Fastener Sets per Step">
            for(unsigned int l = 0; l < nAssembly; l++) {
                nPerStep = 0;
                __lcd_clear();
                __lcd_home();
                unsigned int compartments[];
                if(nAssembly == 4) {
                    for(unsigned int j = 0; j < 4; j++) {
                        compartments[j] = assembly4[j];
                    }
                }
                else if(nAssembly == 5) {
                    for(unsigned int j = 0; j < 5; j++) {
                        compartments[j] = assembly5[j];
                    }
                }
                else if(nAssembly == 6) {
                    for(unsigned int j = 0; j < 6; j++) {
                        compartments[j] = assembly6[j];
                    }
                }
                else {
                    for(unsigned int j = 0; j < nAssembly; j++) {
                        compartments[j] = j + 1;
                    }
                }
                printf("C%d Set/Step:", compartments[l]);
                unsigned int cursorLocation = 0;
                for(i = 0; i < 16; i++) {
                    code[i] = 'X';
                }
                codeFlag = 1;
                cursorLocation = 0;
                __lcd_display_control(1, 1, 1);
                __lcd_newline();
                while(codeFlag) {
                    unsigned int secondFlag = 1;
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    unsigned char temp = keys[keypress];

                    if((temp == '*') && (cursorLocation != 0)) {
                        if(cursorLocation >= 16) {
                            lcd_shift_cursor(1, 1);
                        }
                        cursorLocation -= 1;
                        lcd_shift_cursor(1, 0);
                        putch(' ');
                        lcd_shift_cursor(1, 0);
                        code[cursorLocation] = 'X';
                        nPerStep = 0;
                    } else if(temp == '#' && cursorLocation != 0) {
                        codeFlag = 0;
                    } else if(cursorLocation < 16) {
                        lcd_set_cursor(cursorLocation, 1);
                        if(((temp != '*') && (temp != '#')) && (cursorLocation < 16)) {
                            if(temp == 'A') {
                                temp = '9';
                                putch('B');
                            }
                            else if(temp == 'B') {
                                temp = '9';
                                putch('N');
                            }
                            else if(temp == 'C') {
                                temp = '9';
                                putch('S');
                            }
                            else if(temp == 'D') {
                                temp = '9';
                                putch('W');
                            }
                            else {
                                putch(temp);
                                nPerStep = temp - '0';
                            }
                            secondFlag = 0;
                        }
                        if(cursorLocation >= 15) {
                            lcd_set_cursor(cursorLocation, 1);
                        }
                        if(secondFlag == 0) {
                            code[cursorLocation] = temp;
                            cursorLocation += 1;
                        }
                    }
                    if(!codeFlag) {
                        if(code[1] != 'X') {
                            codeFlag = 1;
                        }
                        else {
                            if((code[0] != '1') && (code[0] != '2') && (code[0] != '3') && (code[0] != '4')) {
                                codeFlag = 1;
                            }
                        }
                        if(codeFlag) {
                            nPerStep = 0;
                            for(i = 0; i < 16; i++) {
                                code[i] = 'X';
                            }
                            __lcd_clear();
                            printf("Invalid Input.");
                            __lcd_newline();
                            printf("Press to Retry.");
                            while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                            while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                            Nop();  // Apply breakpoint here to prevent compiler optimizations
                            __lcd_clear();
                            __lcd_display_control(1, 1, 1);
                            lcd_set_cursor(0, 0);
                            printf("C%d Set/Step", compartments[l]);
                            lcd_set_cursor(0, 1);
                            for(i = 0; i < 16; i++) {
                                code[i] = 'X';
                            }
                            cursorLocation = 0;
                        }
                        else {
                            nPerStep = code[0] - '0';
                        }
                    }
                }
                fastenerSetPerStep[l] = nPerStep;
                if(eightFlag) {
                    nAssembly = 8;
                }
            }
            if(eightFlag) {
                nAssembly = 8;
                eightFlag = 0;
            }
            // </editor-fold>
            
            // <editor-fold defaultstate="collapsed" desc="Validity Check">
            unsigned int Btotal = 0;
            unsigned int Ntotal = 0;
            unsigned int Stotal = 0;
            unsigned int Wtotal = 0;
            unsigned int validityCheck = 0;
            
            // Check if for each compartment: 
                // B < 2
                // N < 3
                // S < 2
                // W < 4
            for(unsigned int j = 0; j < 16; j += 2) {
                Btotal = (fastenerSet[j]/10) * fastenerSetPerStep[j/2];
                Ntotal = (fastenerSet[j] - ((fastenerSet[j]/10) * 10)) * fastenerSetPerStep[j/2];
                Stotal = (fastenerSet[j + 1]/10) * fastenerSetPerStep[j/2];
                Wtotal = (fastenerSet[j + 1] - ((fastenerSet[j + 1]/10) * 10));
                
                if((Btotal > 2) || (Ntotal > 3) || (Stotal > 2) || (Wtotal > 4) || ((Btotal + Ntotal + Stotal + Wtotal) > 4)) {
                    validityCheck = 1;
                    break;
                }
            }
            
            if(validityCheck) {
                __lcd_clear();
                __lcd_home();
                printf("Invalid Inputs.");
                __lcd_newline();
                printf("Press to Continue.");
                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                Nop();  // Apply breakpoint here to prevent compiler optimizations
            }
            // </editor-fold>
            // </editor-fold>
            // </editor-fold>
            
            // <editor-fold defaultstate="collapsed" desc="Operation Mode">
            else {
                // <editor-fold defaultstate="collapsed" desc="Operation Mode Prep 1">
                if(nAssembly == 4) {
                    fastenerSet[12] = fastenerSet[6];
                    fastenerSet[13] = fastenerSet[7];
                    fastenerSet[8] = fastenerSet[4];
                    fastenerSet[9] = fastenerSet[5];
                    fastenerSet[4] = fastenerSet[2];
                    fastenerSet[5] = fastenerSet[3];
                    fastenerSet[2] = 0;
                    fastenerSet[3] = 0;
                    fastenerSet[6] = 0;
                    fastenerSet[7] = 0;
                    fastenerSet[10] = 0;
                    fastenerSet[11] = 0;
                    fastenerSet[14] = 0;
                    fastenerSet[15] = 0;
                    
//                    fastenerSet[6] = fastenerSet[3];
//                    fastenerSet[4] = fastenerSet[2];
//                    fastenerSet[2] = fastenerSet[1];
//                    fastenerSet[1] = 0;
//                    fastenerSet[3] = 0;
//                    fastenerSet[5] = 0;
//                    fastenerSet[7] = 0;

                    fastenerSetPerStep[6] = fastenerSetPerStep[3];
                    fastenerSetPerStep[4] = fastenerSetPerStep[2];
                    fastenerSetPerStep[2] = fastenerSetPerStep[1];
                    fastenerSetPerStep[1] = 0;
                    fastenerSetPerStep[3] = 0;
                    fastenerSetPerStep[5] = 0;
                    fastenerSetPerStep[7] = 0;
                }
                else if(nAssembly == 5) {
                    fastenerSet[12] = fastenerSet[8];
                    fastenerSet[13] = fastenerSet[9];
                    fastenerSet[8] = fastenerSet[6];
                    fastenerSet[9] = fastenerSet[7];
                    fastenerSet[6] = fastenerSet[4];
                    fastenerSet[7] = fastenerSet[5];
                    fastenerSet[4] = 0;
                    fastenerSet[5] = 0;
                    fastenerSet[10] = 0;
                    fastenerSet[11] = 0;
                    fastenerSet[14] = 0;
                    fastenerSet[15] = 0;
                    
//                    fastenerSet[6] = fastenerSet[4];
//                    fastenerSet[4] = fastenerSet[3];
//                    fastenerSet[3] = fastenerSet[2];
//                    fastenerSet[2] = 0;
//                    fastenerSet[5] = 0;
//                    fastenerSet[7] = 0;

                    fastenerSetPerStep[6] = fastenerSetPerStep[4];
                    fastenerSetPerStep[4] = fastenerSetPerStep[3];
                    fastenerSetPerStep[3] = fastenerSetPerStep[2];
                    fastenerSetPerStep[2] = 0;
                    fastenerSetPerStep[5] = 0;
                    fastenerSetPerStep[7] = 0;
                }
                else if(nAssembly == 6) {
                    fastenerSet[12] = fastenerSet[10];
                    fastenerSet[13] = fastenerSet[11];
                    fastenerSet[10] = fastenerSet[8];
                    fastenerSet[11] = fastenerSet[9];
                    fastenerSet[8] = fastenerSet[6];
                    fastenerSet[9] = fastenerSet[7];
                    fastenerSet[6] = 0;
                    fastenerSet[7] = 0;
                    fastenerSet[14] = 0;
                    fastenerSet[15] = 0;
                    
//                    fastenerSet[6] = fastenerSet[5];
//                    fastenerSet[5] = fastenerSet[4];
//                    fastenerSet[4] = fastenerSet[3];
//                    fastenerSet[3] = 0;
//                    fastenerSet[7] = 0;

                    fastenerSetPerStep[6] = fastenerSetPerStep[5];
                    fastenerSetPerStep[5] = fastenerSetPerStep[4];
                    fastenerSetPerStep[4] = fastenerSetPerStep[3];
                    fastenerSetPerStep[3] = 0;
                    fastenerSetPerStep[7] = 0;
                }
                else if(nAssembly == 7) {
                    fastenerSet[14] = 0;
                    fastenerSet[15] = 0;
//                    fastenerSet[7] = 0;
                    fastenerSetPerStep[7] = 0;
                }
                spiInit(16);
                glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, WHITE);
                GLCDtext("OPERATION", 1);
                GLCDtext("  MODE   ", 2);
                
                // Store start time:
                I2C_Master_Init(100000);
                /* Reset RTC memory pointer. */
                I2C_Master_Start(); // Start condition
                I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
                I2C_Master_Write(0x00); // Set memory pointer to seconds
                I2C_Master_Stop(); // Stop condition

                /* Read current time. */
                I2C_Master_Start(); // Start condition
                I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
                for(i = 0; i < 6; i++){
                    time[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
                }
                time[6] = I2C_Master_Read(NACK); // Final Read with NACK
                I2C_Master_Stop(); // Stop condition

                // Store time at which operation mode starts
                for(unsigned int i = 0; i < 6; i++) {
                    start[i] = time[RTC_organize[i]];
                }
                
                __lcd_display_control(1, 1, 1);
                __lcd_clear();
                __lcd_home();
                printf("Operation Mode");
                TRISB = 0x00;
                LATEbits.LATE1 = 1;
                
                // GLCD OPERATION MODE OUTPUT HERE
                spiInit(16);
                GLCDprogressSetup(); // 122 steps required to fill bar
                I2C_Master_Init(100000);
                unsigned int GLCDprogressCount = 0;
                // </editor-fold>

                // Process: 
//                Vibration Motors
//                If microswitch {push solenoid if <}
//                Compartments done
//                Retract ramp using DC (hit microswitch for max retraction)
//                Vibration motors vibrate again (microswitch triggers and push solenoid) until done -> reservoir
                
                // New timer based operation code:
                unsigned int B;
                unsigned int N;
                unsigned int S;
                unsigned int W;
                unsigned int Bmax;
                unsigned int Nmax;
                unsigned int Smax;
                unsigned int Wmax;
                unsigned int microFlag = 1;

                operation = 1;
                
                
                unsigned int skip = 0;
                
                
                
                if(skip != 1) {
                    vibrationBN(1);
                    vibrationSW(1);
                    for(i = 0; i < 6*8; i++) {
                        stepperForward();

                        if((i + 1) % 3 == 0) { // This might slow down rotation
                            spiInit(16);
                            GLCDprogress(); // 16 cumulative calls
                            I2C_Master_Init(100000);
                        }
                    }
                    color = 0;
                    TRISC = 0b10000100;

                    for(i = 0; i < 6*10; i++) { 
                        colorSensor();
                        if(color) {
                            break;
                        }
                        stepperForward();
                        if((i + 1) % 3 == 0) { // This might slow down rotation
                            spiInit(16);
                            GLCDprogress(); // 36 cumulative calls
                            I2C_Master_Init(100000);
                        }
                    }

                    if(!color) {
                        __lcd_clear();
                        __lcd_home();
                        printf("TAPE NOT FOUND");
                        spiInit(16);
                        glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, RED);
                        GLCDtext("TAPE NOT", 1);
                        GLCDtext("FOUND   ", 2);
                        I2C_Master_Init(100000);
                    }
                    else {
                        TRISC = 0b10000000;
                        for(i = 0; i < 3; i++) {
                            stepperForward();
                            if((i + 1) % 3 == 0) { // This might slow down rotation
                                spiInit(16);
                                GLCDprogress(); // 37 cumulative calls
                                I2C_Master_Init(100000);
                            }
                        }
                        for(i = 0; i < 13; i++) {
                            LATBbits.LATB2 = 1;
                            __delay_ms(DELAY_TIME);
                            LATBbits.LATB0 = 0;
                            __delay_ms(DELAY_TIME);
                            LATBbits.LATB1 = 1;
                            __delay_ms(DELAY_TIME);
                            LATBbits.LATB2 = 0;
                            __delay_ms(DELAY_TIME);
                            LATBbits.LATB3 = 1;
                            __delay_ms(DELAY_TIME);
                            LATBbits.LATB1 = 0;
                            __delay_ms(DELAY_TIME);
                            LATBbits.LATB0 = 1;
                            __delay_ms(DELAY_TIME);
                            LATBbits.LATB3 = 0;
                            __delay_ms(DELAY_TIME);

                            if((i + 1) % 3 == 0) { // This might slow down rotation
                                spiInit(16);
                                GLCDprogress(); // 41 cumulative calls 
                                I2C_Master_Init(100000);
                            }
                        }
                        vibrationBN(1);
                        vibrationSW(1);
                        for(unsigned int k = 0; k < 300; k++) {
                            DCbolts();
                            __delay_ms(3000);
                            if(k == 5) {
                                break;
                            }
                        }
                        __delay_ms(2000);
                        for(unsigned int p = 0; p < 16; p += 2) {
                            vibrationBN(1);
                            vibrationSW(1);
                            B = 0;
                            N = 0;
                            S = 0;
                            W = 0;
                            Bmax = (fastenerSet[p]/10) * fastenerSetPerStep[p/2];
                            Nmax = (fastenerSet[p] - ((fastenerSet[p]/10) * 10)) * fastenerSetPerStep[p/2];
                            Smax = (fastenerSet[p + 1]/10) * fastenerSetPerStep[p/2];
                            Wmax = (fastenerSet[p + 1] - (fastenerSet[p + 1]/10) * 10) * fastenerSetPerStep[p/2];
                            
                            // Adapt in final code, do for all instances of this print
                            __lcd_clear();
                            printf("Step %d N=%d", (p/2) + 1, Nmax);
                            __lcd_newline();
                            printf("B=%d S=%d W=%d", Bmax, Smax, Wmax);

                            spiInit(16);
                            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                            unsigned char tempGLCD1[9] = {'C', 'X', ' ', 'I', 'N', 'F', 'O', ' ',  ' '};
                            tempGLCD1[1] = ((p/2) + 1) + '0';
                            GLCDtext(tempGLCD1, 0);
                            GLCDtext("REMAINING", 1);
                            unsigned char tempGLCD2[9] = {'B', 'X', ' ', ' ', 'N', 'X', ' ', ' ', ' '};
                            unsigned char tempGLCD3[9] = {'S', 'X', ' ', ' ', 'W', 'X', ' ', ' ', ' '};
                            tempGLCD2[1] = (Bmax - B) + '0';
                            tempGLCD2[5] = (Nmax - N) + '0';
                            tempGLCD3[1] = (Smax - S) + '0';
                            tempGLCD3[5] = (Wmax - W) + '0';
                            GLCDtext(tempGLCD2, 3);
                            GLCDtext(tempGLCD3, 4);
                            I2C_Master_Init(100000);

                            GLCDprogressCount += Bmax + Nmax + Smax + Wmax;

                            while((B < Bmax) || (N < Nmax) || (S < Smax) || (W < Wmax)) {

                                
                                if(B < Bmax) {
                                    solenoidB();
                                    for(i = 0; i < MICRO_LOOPS; i++) {
                                        if(microswitchB()) {
                                            microFlag = 1;
                                        }
                                    }
                                    vibrationBN(1);
                                    microFlag = 1;
                                    B += 1;

                                    // This might slow down operation
                                    GLCDprogressCount += 1;
                                    spiInit(16);
                                    GLCDprogress();
                                    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                                    unsigned char tempGLCD1[9] = {'C', 'X', ' ', 'I', 'N', 'F', 'O', ' ',  ' '};
                                    tempGLCD1[1] = ((p/2) + 1) + '0';
                                    GLCDtext(tempGLCD1, 0);
                                    GLCDtext("REMAINING", 1);
                                    unsigned char tempGLCD2[9] = {'B', 'X', ' ', ' ', 'N', 'X', ' ', ' ', ' '};
                                    unsigned char tempGLCD3[9] = {'S', 'X', ' ', ' ', 'W', 'X', ' ', ' ', ' '};
                                    tempGLCD2[1] = (Bmax - B) + '0';
                                    tempGLCD2[5] = (Nmax - N) + '0';
                                    tempGLCD3[1] = (Smax - S) + '0';
                                    tempGLCD3[5] = (Wmax - W) + '0';
                                    GLCDtext(tempGLCD2, 3);
                                    GLCDtext(tempGLCD3, 4);
                                    I2C_Master_Init(100000);
                                }
                                if(N < Nmax) {
                                    solenoidN();
                                    for(i = 0; i < MICRO_LOOPS; i++) {
                                        if(microswitchN()) {
                                            microFlag = 1;
                                        }
                                    }
                                    vibrationBN(1);
                                    microFlag = 1;
                                    N += 1;

                                    // This might slow down operation
                                    GLCDprogressCount += 1;
                                    spiInit(16);
                                    GLCDprogress();
                                    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                                    unsigned char tempGLCD1[9] = {'C', 'X', ' ', 'I', 'N', 'F', 'O', ' ',  ' '};
                                    tempGLCD1[1] = ((p/2) + 1) + '0';
                                    GLCDtext(tempGLCD1, 0);
                                    GLCDtext("REMAINING", 1);
                                    unsigned char tempGLCD2[9] = {'B', 'X', ' ', ' ', 'N', 'X', ' ', ' ', ' '};
                                    unsigned char tempGLCD3[9] = {'S', 'X', ' ', ' ', 'W', 'X', ' ', ' ', ' '};
                                    tempGLCD2[1] = (Bmax - B) + '0';
                                    tempGLCD2[5] = (Nmax - N) + '0';
                                    tempGLCD3[1] = (Smax - S) + '0';
                                    tempGLCD3[5] = (Wmax - W) + '0';
                                    GLCDtext(tempGLCD2, 3);
                                    GLCDtext(tempGLCD3, 4);
                                    I2C_Master_Init(100000);
                                }
                                if(S < Smax) {
                                    solenoidS();
                                    for(i = 0; i < MICRO_LOOPS; i++) {
                                        if(microswitchS()) {
                                            microFlag = 1;
                                        }
                                    }
                                    vibrationSW(1);
                                    microFlag = 1;
                                    S += 1;

                                    // This might slow down operation
                                    GLCDprogressCount += 1;
                                    spiInit(16);
                                    GLCDprogress();
                                    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                                    unsigned char tempGLCD1[9] = {'C', 'X', ' ', 'I', 'N', 'F', 'O', ' ',  ' '};
                                    tempGLCD1[1] = ((p/2) + 1) + '0';
                                    GLCDtext(tempGLCD1, 0);
                                    GLCDtext("REMAINING", 1);
                                    unsigned char tempGLCD2[9] = {'B', 'X', ' ', ' ', 'N', 'X', ' ', ' ', ' '};
                                    unsigned char tempGLCD3[9] = {'S', 'X', ' ', ' ', 'W', 'X', ' ', ' ', ' '};
                                    tempGLCD2[1] = (Bmax - B) + '0';
                                    tempGLCD2[5] = (Nmax - N) + '0';
                                    tempGLCD3[1] = (Smax - S) + '0';
                                    tempGLCD3[5] = (Wmax - W) + '0';
                                    GLCDtext(tempGLCD2, 3);
                                    GLCDtext(tempGLCD3, 4);
                                    I2C_Master_Init(100000);
                                }
                                if(W < Wmax) {
                                    
                                    
                                for(unsigned int k = 0; k < 300; k++) {
                                    DCwashers();
                                    __delay_ms(2000);
                                    if(k == 2) {
                                        break;
                                    }
                                }
                                
                                __delay_ms(5000);
                                    
                                    solenoidW();
                                    for(i = 0; i < MICRO_LOOPS; i++) {
                                        if(microswitchW()) {
                                            microFlag = 1;
                                        }
                                    }
                                    vibrationSW(1);
                                    microFlag = 1;
                                    W += 1;

                                    // This might slow down operation
                                    GLCDprogressCount += 1;
                                    spiInit(16);
                                    GLCDprogress();
                                    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                                    unsigned char tempGLCD1[9] = {'C', 'X', ' ', 'I', 'N', 'F', 'O', ' ',  ' '};
                                    tempGLCD1[1] = ((p/2) + 1) + '0';
                                    GLCDtext(tempGLCD1, 0);
                                    GLCDtext("REMAINING", 1);
                                    unsigned char tempGLCD2[9] = {'B', 'X', ' ', ' ', 'N', 'X', ' ', ' ', ' '};
                                    unsigned char tempGLCD3[9] = {'S', 'X', ' ', ' ', 'W', 'X', ' ', ' ', ' '};
                                    tempGLCD2[1] = (Bmax - B) + '0';
                                    tempGLCD2[5] = (Nmax - N) + '0';
                                    tempGLCD3[1] = (Smax - S) + '0';
                                    tempGLCD3[5] = (Wmax - W) + '0';
                                    GLCDtext(tempGLCD2, 3);
                                    GLCDtext(tempGLCD3, 4);
                                    I2C_Master_Init(100000);
                                }
                            }
                            // Following possibly draws too much power from stepper because vibration functions for B and W call same pins as the ones used for B and N switches
                            for(unsigned int b = 0; b < 6; b++) {
                                stepperForward();
                                if((b + 1) % 3 == 0) { // This might slow down rotation
                                    spiInit(16);
                                    GLCDprogress(); // 57 + GLCDprogressCount cumulative calls
                                    I2C_Master_Init(100000);
                                }
                            }
                        }
                        __lcd_clear();
                        printf("Closing");
                        for(i = 0; i < 56; i++) { // 8 steps here
                            stepperReverse();

                            if((i + 1) % 3 == 0) { // This might slow down rotation
                                spiInit(16);
                                GLCDprogress(); // 74 + GLCDprogressCount cumulative calls
                                I2C_Master_Init(100000);
                            }
                        }
                        B = 0;
                        N = 0;
                        S = 0;
                        W = 0; 

                        unsigned int DCflag = 0;
                        if(DCflag) {
                            // DC:
                            spiInit(16);
                            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                            GLCDtext("OPERATION", 1);
                            GLCDtext("  MODE   ", 2);
                            I2C_Master_Init(100000);

                            __lcd_clear();
                            printf("Dispensing.");
                            DCmotor(2);
                            __delay_ms(4000);
                            __delay_ms(1500);
                            DCmotor(0);
                            unsigned int timer = 0;
                            Bmax = 0;
                            Nmax = 0;
                            Smax = 0;
                            Wmax = 0;
                            vibrationBN(1);
                            vibrationSW(1);
                            for(i = 0; i < 8; i++) {
                                Bmax += fastenerSet[i]/10;
                                Nmax += fastenerSet[i] - (Bmax * 10);
                                Smax += fastenerSet[i + 1]/10;
                                Wmax += fastenerSet[i + 1] - (Smax * 10);
                            }
                            Bmax = 20 - Bmax;
                            Nmax = 25 - Nmax;
                            Smax = 20 - Smax;
                            Wmax = 35 - Wmax;

                            // Initializing variables for GLCD progress bar:
                            unsigned int divider = (122 - (74 + GLCDprogressCount)) / (Bmax + Nmax + Smax + Wmax);
                            unsigned int track = 0;

                            B = 0;
                            N = 0;
                            S = 0;
                            W = 0;
                            while(((B < Bmax) || (N < Nmax) || (S < Smax) || (W < Wmax)) && (timer < 1)) {                             
                                __delay_ms(3000);
                                if(B < Bmax) {
                                    for(i = 0; i < 2; i++) {
                                        vibrationBN(0);
                                        solenoidB();
                                        for(i = 0; i < MICRO_LOOPS; i++) {
                                            if(microswitchB()) {
                                                microFlag = 1;
                                            }
                                        }
                                        if(microFlag) {
                                            vibrationBN(1);
                                            microFlag = 1;
                                            B += 1;

                                            // This might slow down operation
                                            spiInit(16);
                                            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                                            GLCDtext("DISPENSED", 0);
                                            unsigned char tempGLCD2[9] = {'B', 'X', ' ', ' ', 'N', 'X', ' ', ' ', ' '};
                                            unsigned char tempGLCD3[9] = {'S', 'X', ' ', ' ', 'W', 'X', ' ', ' ', ' '};
                                            tempGLCD2[1] = B + '0';
                                            tempGLCD2[5] = N + '0';
                                            tempGLCD3[1] = S + '0';
                                            tempGLCD3[5] = W + '0';
                                            GLCDtext(tempGLCD2, 1);
                                            GLCDtext(tempGLCD3, 2);
                                            I2C_Master_Init(100000);

                                            track += 1;
                                            if((track % divider)== 0) {
                                                spiInit(16);
                                                GLCDprogress();
                                                I2C_Master_Init(100000);
                                                GLCDprogressCount += 1;
                                            }
                                        }
                                    }
                                }
                                if(N < Nmax) {
                                    for(i = 0; i < 2;i ++) {
                                        vibrationBN(0);
                                        solenoidN();
                                        for(i = 0; i < MICRO_LOOPS; i++) {
                                            if(microswitchN()) {
                                                microFlag = 1;
                                            }
                                        }
                                        if(microFlag) {
                                            vibrationBN(1);
                                            microFlag = 1;
                                            N += 1;

                                            // This might slow down operation
                                            spiInit(16);
                                            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                                            GLCDtext("DISPENSED", 0);
                                            unsigned char tempGLCD2[9] = {'B', 'X', ' ', ' ', 'N', 'X', ' ', ' ', ' '};
                                            unsigned char tempGLCD3[9] = {'S', 'X', ' ', ' ', 'W', 'X', ' ', ' ', ' '};
                                            tempGLCD2[1] = B + '0';
                                            tempGLCD2[5] = N + '0';
                                            tempGLCD3[1] = S + '0';
                                            tempGLCD3[5] = W + '0';
                                            GLCDtext(tempGLCD2, 1);
                                            GLCDtext(tempGLCD3, 2);
                                            I2C_Master_Init(100000);

                                            track += 1;
                                            if((track % divider)== 0) {
                                                spiInit(16);
                                                GLCDprogress();
                                                I2C_Master_Init(100000);
                                                GLCDprogressCount += 1;
                                            }
                                            //timer = 0;
                                        }
                                    }
                                }
                                if(S < Smax) {
                                    for(i = 0; i < 2; i++) {
                                        vibrationSW(0);
                                        solenoidS();
                                        for(i = 0; i < MICRO_LOOPS; i++) {
                                            if(microswitchS()) {
                                                microFlag = 1;
                                            }
                                        }
                                        if(microFlag) {
                                            vibrationSW(1);
                                            microFlag = 1;
                                            S += 1;

                                            // This might slow down operation
                                            spiInit(16);
                                            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                                            GLCDtext("DISPENSED", 0);
                                            unsigned char tempGLCD2[9] = {'B', 'X', ' ', ' ', 'N', 'X', ' ', ' ', ' '};
                                            unsigned char tempGLCD3[9] = {'S', 'X', ' ', ' ', 'W', 'X', ' ', ' ', ' '};
                                            tempGLCD2[1] = B + '0';
                                            tempGLCD2[5] = N + '0';
                                            tempGLCD3[1] = S + '0';
                                            tempGLCD3[5] = W + '0';
                                            GLCDtext(tempGLCD2, 1);
                                            GLCDtext(tempGLCD3, 2);
                                            I2C_Master_Init(100000);

                                            track += 1;
                                            if((track % divider)== 0) {
                                                spiInit(16);
                                                GLCDprogress();
                                                I2C_Master_Init(100000);
                                                GLCDprogressCount += 1;
                                            }
                                            //timer = 0;
                                        }
                                    }
                                }
                                if(W < Wmax) {
                                    for(unsigned int k = 0; k < 300; k++) {
                                        DCwashers();
                                        __delay_ms(3000);
                                        if(k == 7) {
                                            break;
                                        }
                                    }

                                    __delay_ms(5000);

                                    for(i = 0; i < 3; i++) {

                                        vibrationSW(0);
                                        solenoidW();
                                        for(i = 0; i < MICRO_LOOPS; i++) {
                                            if(microswitchW()) {
                                                microFlag = 1;
                                            }
                                        }
                                        if(microFlag) {
                                            vibrationSW(1);
                                            microFlag = 1;
                                            W += 1;

                                            // This might slow down operation
                                            spiInit(16);
                                            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, 94, WHITE);
                                            GLCDtext("DISPENSED", 0);
                                            unsigned char tempGLCD2[9] = {'B', 'X', ' ', ' ', 'N', 'X', ' ', ' ', ' '};
                                            unsigned char tempGLCD3[9] = {'S', 'X', ' ', ' ', 'W', 'X', ' ', ' ', ' '};
                                            tempGLCD2[1] = B + '0';
                                            tempGLCD2[5] = N + '0';
                                            tempGLCD3[1] = S + '0';
                                            tempGLCD3[5] = W + '0';
                                            GLCDtext(tempGLCD2, 1);
                                            GLCDtext(tempGLCD3, 2);
                                            I2C_Master_Init(100000);

                                            track += 1;
                                            if((track % divider)== 0) {
                                                spiInit(16);
                                                GLCDprogress();
                                                I2C_Master_Init(100000);
                                                GLCDprogressCount += 1;
                                            }
                                            //timer = 0;
                                        }
                                    }
                                }
                                timer += 1;
                            }
                            vibrationBN(0);
                            vibrationSW(0);
    //                        DCmotor(2);
    //                        __delay_ms(500);
    //                        DCmotor(0);
                            numBremaining = B;
                            numNremaining = N;
                            numSremaining = S;
                            numWremaining = W;
                            TRISC = 0b10000100;
                            if((122 - 84 + GLCDprogressCount) > 0) {
                                for(i = 0; i < (122 - 81 + GLCDprogressCount); i++) {
                                    spiInit(16);
                                    GLCDprogress();
                                    I2C_Master_Init(100000);
                                }
                            }
                        }
                    }
                    spiInit(16);
                    glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, GREEN);
                    GLCDtext("OPERATION", 0);
                    GLCDtext("SUCCESS  ", 1);
                    I2C_Master_Init(100000);
                }
                operation = 0; 
                TRISB = 0xFF;
                LATEbits.LATE1 = 0;
                
                // <editor-fold defaultstate="collapsed" desc="Operation Mode Prep 2">
                // Store end time:

                I2C_Master_Init(100000);
                /* Reset RTC memory pointer. */
                I2C_Master_Start(); // Start condition
                I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
                I2C_Master_Write(0x00); // Set memory pointer to seconds
                I2C_Master_Stop(); // Stop condition

                /* Read current time. */
                I2C_Master_Start(); // Start condition
                I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
                for(i = 0; i < 6; i++){
                    time[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
                }
                time[6] = I2C_Master_Read(NACK); // Final Read with NACK
                I2C_Master_Stop(); // Stop condition
                
                // Store time at which operation mode ends
                for(unsigned int i = 0; i < 6; i++) {
                    end[i] = time[RTC_organize[i]];
                }
                for(i = 0; i < 6; i++) {
                    tempend[i] = ASCIItoInt(end[i]);
                    tempstart[i] = ASCIItoInt(start[i]);
                }

                // Calculating runtime:
                if((tempend[0] > tempstart[0]) && (tempend[1] < tempstart[1])) {
                    tempend[0] = tempend[0] - 1;
                    tempend[1] = tempend[1] + 12;
                }
                if((tempend[1] > tempstart[1]) && (tempend[2] < tempstart[2])) {
                    if((tempstart[1] == 1) || (tempstart[1] == 3) || (tempstart[1] == 5) || (tempstart[1] == 7) || (tempstart[1] == 8) || (tempstart[1] == 10) || (tempstart[1] == 12)) {
                        tempend[2] = tempend[2] + 31;
                    } 
                    else if (tempstart[1] == 2) {
                        tempend[2] = tempend[2] + 28;
                    } 
                    else {
                        tempend[2] = tempend[2] + 30;
                    }
                    tempend[1] = tempend[1] - 1;
                }
                if((tempend[2] > tempstart[2]) && (tempend[3] < tempstart[3])) {
                    tempend[2] = tempend[2] - 1;
                    tempend[3] = tempend[3] + 24;
                }
                if((tempend[3] > tempstart[3]) && (tempend[4] < tempstart[4])) {
                    tempend[3] = tempend[3] - 1;
                    tempend[4] = tempend[4] + 60;
                }
                if((tempend[4] > tempstart[4]) && (tempend[5] < tempstart[5])) {
                    tempend[4] = tempend[4] - 1;
                    tempend[5] = tempend[5] + 60;
                }
                
                for(i = 0; i < 6; i++) {
                    runtime[i] = tempend[i] - tempstart[i];
                }
                
                // Store to EEPROM:
                store_data();
                TRISB = 0xFF;
                
                // </editor-fold>
                // </editor-fold>            

                // <editor-fold defaultstate="collapsed" desc="Standby Mode 2">
                spiInit(16);
                glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
                GLCDtext(" STANDBY ", 2);
                GLCDtext("  MODE   ", 3);
                I2C_Master_Init(100000);
                
                // <editor-fold defaultstate="collapsed" desc="PC Interface">
                unsigned int tempInt;
                unsigned char tempChar;
                
                transmitStringUART("__________________________________\r\n");
                transmitStringUART("Number of Fasteners Remaining:\r\n");
                transmitStringUART("Bolts: ");

                tempInt = numBremaining;
                tempChar = tempInt + '0';

                transmitCharUART(tempChar);
                transmitStringUART("\r\nNuts: ");

                tempInt = numNremaining;
                tempChar = tempInt + '0';

                transmitCharUART(tempChar);
                transmitStringUART("\r\nSpacers: ");

                tempInt = numSremaining;
                tempChar = tempInt + '0';

                transmitCharUART(tempChar);
                transmitStringUART("\r\nWashers: ");

                tempInt = numWremaining;
                tempChar = tempInt + '0';

                transmitCharUART(tempChar);

                unsigned char tempSprintf[63];
                transmitStringUART("\r\n\r\nStart Date ");
                sprintf(tempSprintf[63], "(YY/MM/DD): %02x/%02x/%02x", start[0], start[1], start[2]);
                transmitStringUART(tempSprintf[63]);

                transmitStringUART("\r\n\r\nStart Time ");
                sprintf(tempSprintf[63], "(HH:MM:SS): %02x:%02x:%02x", start[3], start[4], start[5]);
                transmitStringUART(tempSprintf[63]);

                transmitStringUART("\r\n\r\nRuntime (MM:SS): ");
                if(runtime[4] < 10) {
                    transmitCharUART('0');
                }
                tempInt = runtime[4];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);
                transmitCharUART(':');
                if(runtime[5] < 10) {
                    transmitCharUART('0');
                }
                tempInt = runtime[5];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);

                transmitStringUART("\r\n\r\nNumber of Assembly Steps: ");
                tempInt = nAssembly;
                tempChar = tempInt +'0';
                transmitCharUART(tempChar);
                
                transmitStringUART("\r\n\r\nFastener Set:");

                transmitStringUART("\r\nCompartment 1: ");

                unsigned int tempInt2;
                Btotal = (fastenerSet[0]/10);
                Ntotal = fastenerSet[0] - (Btotal * 10);
                Stotal = fastenerSet[1]/10;
                Wtotal = fastenerSet[1] - (Stotal * 10);
                
                for(unsigned int j = 0; j < Btotal; j++) {
                    transmitCharUART('B');
                }
                for(unsigned int j = 0; j < Ntotal; j++) {
                    transmitCharUART('N');
                }
                for(unsigned int j = 0; j < Stotal; j++) {
                    transmitCharUART('S');
                }
                for(unsigned int j = 0; j < Wtotal; j++) {
                    transmitCharUART('W');
                }

                transmitStringUART("\r\nCompartment 2: ");

                Btotal = (fastenerSet[2]/10);
                Ntotal = fastenerSet[2] - (Btotal * 10);
                Stotal = fastenerSet[3]/10;
                Wtotal = fastenerSet[3] - (Stotal * 10);

                for(unsigned int j = 0; j < Btotal; j++) {
                    transmitCharUART('B');
                }
                for(unsigned int j = 0; j < Ntotal; j++) {
                    transmitCharUART('N');
                }
                for(unsigned int j = 0; j < Stotal; j++) {
                    transmitCharUART('S');
                }
                for(unsigned int j = 0; j < Wtotal; j++) {
                    transmitCharUART('W');
                }

                transmitStringUART("\r\nCompartment 3: ");

                Btotal = (fastenerSet[4]/10);
                Ntotal = fastenerSet[4] - (Btotal * 10);
                Stotal = fastenerSet[5]/10;
                Wtotal = fastenerSet[5] - (Stotal * 10);

                for(unsigned int j = 0; j < Btotal; j++) {
                    transmitCharUART('B');
                }
                for(unsigned int j = 0; j < Ntotal; j++) {
                    transmitCharUART('N');
                }
                for(unsigned int j = 0; j < Stotal; j++) {
                    transmitCharUART('S');
                }
                for(unsigned int j = 0; j < Wtotal; j++) {
                    transmitCharUART('W');
                }

                transmitStringUART("\r\nCompartment 4: ");

                Btotal = (fastenerSet[6]/10);
                Ntotal = fastenerSet[6] - (Btotal * 10);
                Stotal = fastenerSet[7]/10;
                Wtotal = fastenerSet[7] - (Stotal * 10);

                for(unsigned int j = 0; j < Btotal; j++) {
                    transmitCharUART('B');
                }
                for(unsigned int j = 0; j < Ntotal; j++) {
                    transmitCharUART('N');
                }
                for(unsigned int j = 0; j < Stotal; j++) {
                    transmitCharUART('S');
                }
                for(unsigned int j = 0; j < Wtotal; j++) {
                    transmitCharUART('W');
                }

                transmitStringUART("\r\nCompartment 5: ");

                Btotal = (fastenerSet[8]/10);
                Ntotal = fastenerSet[8] - (Btotal * 10);
                Stotal = fastenerSet[9]/10;
                Wtotal = fastenerSet[9] - (Stotal * 10);

                for(unsigned int j = 0; j < Btotal; j++) {
                    transmitCharUART('B');
                }
                for(unsigned int j = 0; j < Ntotal; j++) {
                    transmitCharUART('N');
                }
                for(unsigned int j = 0; j < Stotal; j++) {
                    transmitCharUART('S');
                }
                for(unsigned int j = 0; j < Wtotal; j++) {
                    transmitCharUART('W');
                }

                transmitStringUART("\r\nCompartment 6: ");

                Btotal = (fastenerSet[10]/10);
                Ntotal = fastenerSet[10] - (Btotal * 10);
                Stotal = fastenerSet[11]/10;
                Wtotal = fastenerSet[11] - (Stotal * 10);

                for(unsigned int j = 0; j < Btotal; j++) {
                    transmitCharUART('B');
                }
                for(unsigned int j = 0; j < Ntotal; j++) {
                    transmitCharUART('N');
                }
                for(unsigned int j = 0; j < Stotal; j++) {
                    transmitCharUART('S');
                }
                for(unsigned int j = 0; j < Wtotal; j++) {
                    transmitCharUART('W');
                }

                transmitStringUART("\r\nCompartment 7: ");

                Btotal = (fastenerSet[12]/10);
                Ntotal = fastenerSet[12] - (Btotal * 10);
                Stotal = fastenerSet[13]/10;
                Wtotal = fastenerSet[13] - (Stotal * 10);

                for(unsigned int j = 0; j < Btotal; j++) {
                    transmitCharUART('B');
                }
                for(unsigned int j = 0; j < Ntotal; j++) {
                    transmitCharUART('N');
                }
                for(unsigned int j = 0; j < Stotal; j++) {
                    transmitCharUART('S');
                }
                for(unsigned int j = 0; j < Wtotal; j++) {
                    transmitCharUART('W');
                }

                transmitStringUART("\r\nCompartment 8: ");

                Btotal = (fastenerSet[14]/10);
                Ntotal = fastenerSet[14] - (Btotal * 10);
                Stotal = fastenerSet[15]/10;
                Wtotal = fastenerSet[15] - (Stotal * 10);

                for(unsigned int j = 0; j < Btotal; j++) {
                    transmitCharUART('B');
                }
                for(unsigned int j = 0; j < Ntotal; j++) {
                    transmitCharUART('N');
                }
                for(unsigned int j = 0; j < Stotal; j++) {
                    transmitCharUART('S');
                }
                for(unsigned int j = 0; j < Wtotal; j++) {
                    transmitCharUART('W');
                }

                transmitStringUART("\r\n\r\nFastener Set Per Assembly Step:");
                transmitStringUART("\r\nCompartment 1: ");
                tempInt = fastenerSetPerStep[0];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);
                transmitStringUART("\r\nCompartment 2: ");
                tempInt = fastenerSetPerStep[1];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);
                transmitStringUART("\r\nCompartment 3: ");
                tempInt = fastenerSetPerStep[2];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);
                transmitStringUART("\r\nCompartment 4: ");
                tempInt = fastenerSetPerStep[3];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);
                transmitStringUART("\r\nCompartment 5: ");
                tempInt = fastenerSetPerStep[4];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);
                transmitStringUART("\r\nCompartment 6: ");
                tempInt = fastenerSetPerStep[5];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);
                transmitStringUART("\r\nCompartment 7: ");
                tempInt = fastenerSetPerStep[6];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);
                transmitStringUART("\r\nCompartment 8: ");
                tempInt = fastenerSetPerStep[7];
                tempChar = tempInt + '0';
                transmitCharUART(tempChar);
                transmitStringUART("\r\n__________________________________\r\n");
                // </editor-fold>
                
                __lcd_clear();
                printf("Show Info?");
                __lcd_newline();
                printf("Enter:Y  Else:N");

                // Return GLCD to yellow. 

                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                Nop();  // Apply breakpoint here to prevent compiler optimizations

                unsigned char temp = keys[keypress];

                if(temp == '#') { 
                    __lcd_display_control(1, 0, 0);
                    __lcd_clear();
                    __lcd_home();
                    printf("Began: %02x/%02x/%02x", start[0], start[1], start[2]);
                    __lcd_newline();
                    printf("       %02x:%02x:%02x", start[3], start[4], start[5]);

                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release

                    __lcd_clear();
                    __lcd_home();
                    printf("Run Time(MM:SS):");
                    __lcd_newline();
                    if(runtime[4] < 10) {
                        printf("0");
                    }
                    printf("%d:", runtime[4]);
                    if(runtime[5] < 10) {
                        printf("0");
                    }
                    printf("%d", runtime[5]);

                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release

                    __lcd_clear();
                    __lcd_home();
                    printf("No. Remaining");
                    __lcd_newline();
                    printf("Fasteners:");
                    loopFlag = 0;
                    for(unsigned int i = 0; i < 200; i++) {
                        if(loopFlag) {
                            break;
                        }
                        __delay_ms(25);
                    }
                    __lcd_clear();
                    __lcd_home();
                    printf("Remaining:");
                    __lcd_newline();
                    printf("B:%d N:%d S:%d W:%d", numBremaining, numNremaining, numSremaining, numWremaining);
                    __delay_ms(300);

                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    
                    __lcd_clear();
                    lcd_set_cursor(0, 0);
                    printf("No. Steps:");
                    __lcd_newline();
                    printf("%d", nAssembly);
                    __delay_ms(300);

                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    
                    __lcd_clear();
                    __lcd_home();
                    printf("Fastener Set");
                    __lcd_newline();
                    printf("For Each Step");
                    loopFlag = 0;
                    for(unsigned int i = 0; i < 200; i++) {
                        if(loopFlag) {
                            break;
                        }
                        __delay_ms(25);
                    }
                    for(unsigned int j = 0; j < 16; j += 2) {      
                        unsigned int temp;
                        Btotal = (fastenerSet[j]/10);
                        Ntotal = fastenerSet[j] - (Btotal * 10);
                        Stotal = fastenerSet[j + 1]/10;
                        Wtotal = fastenerSet[j + 1] - (Stotal * 10);
                        __lcd_clear();
                        __lcd_home();
                        printf("C%d Fastener Set:", (j/2) + 1);
                        __lcd_newline();
                        printf("B:%d N:%d S:%d W:%d", Btotal, Ntotal, Stotal, Wtotal);
                        __delay_ms(300);
                        
                        while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                        unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                        while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    }
                    for(unsigned int j = 0; j < 8; j++){
                        __lcd_clear();
                        __lcd_home();
                        printf("C%d Sets/Step:", j + 1);
                        __lcd_newline();
                        printf("%d", fastenerSetPerStep[j]);
                        __delay_ms(300);

                        while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                        unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                        while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    }
                }
            }
        }
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="C - Logs">
        else if(temp == 'C') {
            // <editor-fold defaultstate="collapsed" desc="EEPROM - PC">
            spiInit(16);
            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
            GLCDtext("  LOGS  ", 2);
            I2C_Master_Init(100000);
            __lcd_clear();
            __lcd_home();
            printf("1:PC Transfer");
            __lcd_newline();
            printf("2:Logs Else:Back");
            while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
            unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
            while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
            Nop();  // Apply breakpoint here to prevent compiler optimizations

            unsigned char temp = keys[keypress];
            if(temp == '1') {
                spiInit(16);
                glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
                GLCDtext("PIC TO PC", 2);
                I2C_Master_Init(100000);
                __lcd_clear();
                __lcd_home();
                printf("0 = Most Recent");
                __lcd_newline();
                printf("Select Log: ");
                unsigned int codeFlag = 0;
                unsigned int selectLog = 10;
                while(codeFlag == 0) {
                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                    unsigned char temp = keys[keypress];

                    if((temp == '#') && (selectLog != 10)) {
                        load_data(1);
                        if((load[39] + 1) < selectLog) {
                            __lcd_clear();
                            __lcd_home();
                            printf("Absent Log.");
                            __lcd_newline();
                            printf("Press: Continue");
                            transmitStringUART("Absent Log.\r\n");
                            while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                            while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                            Nop();  // Apply breakpoint here to prevent compiler optimizations
                        }
                        else {
                            unsigned int newLog = (load[20] + 1) - selectLog;
                            load_data(newLog);
                            if(load[1] == 255) {
                                __lcd_clear();
                                __lcd_home();
                                printf("Absent Log.");
                                __lcd_newline();
                                printf("Press: Continue");
                                transmitStringUART("Absent Log.\r\n");
                                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                                Nop();  // Apply breakpoint here to prevent compiler optimizations
                            }
                            else {
                                unsigned char tempString[9] = "LOG      ";
                                tempString[4] = selectLog + '0';
                                spiInit(16);
                                GLCDtext(tempString, 1);
                                I2C_Master_Init(100000);
                                
                                // <editor-fold defaultstate="collapsed" desc="PC User Interface">
                                unsigned int tempInt = selectLog;
                                unsigned char tempChar = selectLog + '0';
                                
                                transmitStringUART("__________________________________\r\n");
                                transmitStringUART("Log: ");
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\n\r\n");
                                transmitStringUART("Number of Fasteners Remaining:\r\n");
                                transmitStringUART("Bolts: ");
                                
                                tempInt = load[1];
                                tempChar = tempInt + '0';
                                
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nNuts: ");
                                
                                tempInt = load[2];
                                tempChar = tempInt + '0';
                                
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nSpacers: ");
                                
                                tempInt = load[3];
                                tempChar = tempInt + '0';
                                
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nWashers: ");
                                
                                tempInt = load[4];
                                tempChar = tempInt + '0';
                                
                                transmitCharUART(tempChar);
                                
                                unsigned char tempSprintf[63];
                                transmitStringUART("\r\n\r\nStart Date (YY/MM/DD): ");
                                sprintf(tempSprintf[63], "%02x/%02x/%02x", load[5], load[6], load[7]);
                                transmitStringUART(tempSprintf[63]);
                                
                                transmitStringUART("\r\n\r\nStart Time (HH:MM:SS): ");
                                sprintf(tempSprintf[63], "%02x:%02x:%02x", load[8], load[9], load[10]);
                                transmitStringUART(tempSprintf[63]);
                                
                                transmitStringUART("\r\n\r\nRuntime (MM:SS): ");
                                if(load[11] < 10) {
                                    transmitCharUART('0');
                                }
                                tempInt = load[11];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                transmitCharUART(':');
                                if(load[12] < 10) {
                                    transmitCharUART('0');
                                }
                                tempInt = load[12];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                
                                transmitStringUART("\r\n\r\nNumber of Assembly Steps: ");
                                tempInt = load[37];
                                tempChar = tempInt +'0';
                                transmitCharUART(tempChar);
                                
                                transmitStringUART("\r\n\r\nFastener Set:");
                                
                                transmitStringUART("\r\nCompartment 1: ");
                                
                                unsigned int Btotal = (load[13]/10);
                                unsigned int Ntotal = load[13] - (Btotal * 10);
                                unsigned int Stotal = (load[14]/10);
                                unsigned int Wtotal = load[14] - (Stotal * 10);
                                for(unsigned int j = 0; j < Btotal; j++) {
                                    transmitCharUART('B');
                                }
                                for(unsigned int j = 0; j < Ntotal; j++) {
                                    transmitCharUART('N');
                                }
                                for(unsigned int j = 0; j < Stotal; j++) {
                                    transmitCharUART('S');
                                }
                                for(unsigned int j = 0; j < Wtotal; j++) {
                                    transmitCharUART('W');
                                }
                                
                                transmitStringUART("\r\nCompartment 2: ");
                                
                                unsigned int Btotal = (load[15]/10);
                                unsigned int Ntotal = load[15] - (Btotal * 10);
                                unsigned int Stotal = (load[16]/10);
                                unsigned int Wtotal = load[16] - (Stotal * 10);
                                
                                for(unsigned int j = 0; j < Btotal; j++) {
                                    transmitCharUART('B');
                                }
                                for(unsigned int j = 0; j < Ntotal; j++) {
                                    transmitCharUART('N');
                                }
                                for(unsigned int j = 0; j < Stotal; j++) {
                                    transmitCharUART('S');
                                }
                                for(unsigned int j = 0; j < Wtotal; j++) {
                                    transmitCharUART('W');
                                }
                                
                                transmitStringUART("\r\nCompartment 3: ");
                                
                                unsigned int Btotal = (load[17]/10);
                                unsigned int Ntotal = load[17] - (Btotal * 10);
                                unsigned int Stotal = (load[18]/10);
                                unsigned int Wtotal = load[18] - (Stotal * 10);
                                
                                for(unsigned int j = 0; j < Btotal; j++) {
                                    transmitCharUART('B');
                                }
                                for(unsigned int j = 0; j < Ntotal; j++) {
                                    transmitCharUART('N');
                                }
                                for(unsigned int j = 0; j < Stotal; j++) {
                                    transmitCharUART('S');
                                }
                                for(unsigned int j = 0; j < Wtotal; j++) {
                                    transmitCharUART('W');
                                }
                                
                                transmitStringUART("\r\nCompartment 4: ");
                                
                                unsigned int Btotal = (load[19]/10);
                                unsigned int Ntotal = load[19] - (Btotal * 10);
                                unsigned int Stotal = (load[20]/10);
                                unsigned int Wtotal = load[20] - (Stotal * 10);
                                
                                for(unsigned int j = 0; j < Btotal; j++) {
                                    transmitCharUART('B');
                                }
                                for(unsigned int j = 0; j < Ntotal; j++) {
                                    transmitCharUART('N');
                                }
                                for(unsigned int j = 0; j < Stotal; j++) {
                                    transmitCharUART('S');
                                }
                                for(unsigned int j = 0; j < Wtotal; j++) {
                                    transmitCharUART('W');
                                }
                                
                                transmitStringUART("\r\nCompartment 5: ");
                                
                                unsigned int Btotal = (load[21]/10);
                                unsigned int Ntotal = load[21] - (Btotal * 10);
                                unsigned int Stotal = (load[22]/10);
                                unsigned int Wtotal = load[22] - (Stotal * 10);
                                
                                for(unsigned int j = 0; j < Btotal; j++) {
                                    transmitCharUART('B');
                                }
                                for(unsigned int j = 0; j < Ntotal; j++) {
                                    transmitCharUART('N');
                                }
                                for(unsigned int j = 0; j < Stotal; j++) {
                                    transmitCharUART('S');
                                }
                                for(unsigned int j = 0; j < Wtotal; j++) {
                                    transmitCharUART('W');
                                }
                                
                                transmitStringUART("\r\nCompartment 6: ");
                                
                                unsigned int Btotal = (load[23]/10);
                                unsigned int Ntotal = load[23] - (Btotal * 10);
                                unsigned int Stotal = (load[24]/10);
                                unsigned int Wtotal = load[24] - (Stotal * 10);
                                
                                for(unsigned int j = 0; j < Btotal; j++) {
                                    transmitCharUART('B');
                                }
                                for(unsigned int j = 0; j < Ntotal; j++) {
                                    transmitCharUART('N');
                                }
                                for(unsigned int j = 0; j < Stotal; j++) {
                                    transmitCharUART('S');
                                }
                                for(unsigned int j = 0; j < Wtotal; j++) {
                                    transmitCharUART('W');
                                }
                                
                                transmitStringUART("\r\nCompartment 7: ");
                                
                                unsigned int Btotal = (load[25]/10);
                                unsigned int Ntotal = load[25] - (Btotal * 10);
                                unsigned int Stotal = (load[26]/10);
                                unsigned int Wtotal = load[26] - (Stotal * 10);
                                
                                for(unsigned int j = 0; j < Btotal; j++) {
                                    transmitCharUART('B');
                                }
                                for(unsigned int j = 0; j < Ntotal; j++) {
                                    transmitCharUART('N');
                                }
                                for(unsigned int j = 0; j < Stotal; j++) {
                                    transmitCharUART('S');
                                }
                                for(unsigned int j = 0; j < Wtotal; j++) {
                                    transmitCharUART('W');
                                }
                                
                                transmitStringUART("\r\nCompartment 8: ");
                                
                                unsigned int Btotal = (load[27]/10);
                                unsigned int Ntotal = load[27] - (Btotal * 10);
                                unsigned int Stotal = (load[28]/10);
                                unsigned int Wtotal = load[28] - (Stotal * 10);
                                
                                for(unsigned int j = 0; j < Btotal; j++) {
                                    transmitCharUART('B');
                                }
                                for(unsigned int j = 0; j < Ntotal; j++) {
                                    transmitCharUART('N');
                                }
                                for(unsigned int j = 0; j < Stotal; j++) {
                                    transmitCharUART('S');
                                }
                                for(unsigned int j = 0; j < Wtotal; j++) {
                                    transmitCharUART('W');
                                }
                                
                                transmitStringUART("\r\n\r\nFastener Set Per Assembly Step:");
                                transmitStringUART("\r\nCompartment 1: ");
                                tempInt = load[29];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nCompartment 2: ");
                                tempInt = load[30];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nCompartment 3: ");
                                tempInt = load[31];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nCompartment 4: ");
                                tempInt = load[32];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nCompartment 5: ");
                                tempInt = load[33];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nCompartment 6: ");
                                tempInt = load[34];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nCompartment 7: ");
                                tempInt = load[35];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\nCompartment 8: ");
                                tempInt = load[36];
                                tempChar = tempInt + '0';
                                transmitCharUART(tempChar);
                                transmitStringUART("\r\n__________________________________\r\n");      
                                // </editor-fold>
                            }
                        }

                        codeFlag = 1;
                    }
                    else if((temp == '0') || (temp == '1') || (temp == '2') || (temp == '3') || (temp == '4') || (temp == '5') || (temp == '6')) {
                        selectLog = temp - '0';
                        lcd_set_cursor(13, 1);
                        putch(temp);
                    }
                    else if(temp == '*') {
                        lcd_set_cursor(13, 1);
                        putch(' ');
                        selectLog = 10;
                    }
                }
            }
            // </editor-fold>
            
            // <editor-fold defaultstate="collapsed" desc="EEPROM - PIC">
            else if(temp == '2') {
                spiInit(16);
                glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
                GLCDtext("  EEPROM ", 2);
                I2C_Master_Init(100000);
                __lcd_clear();
                __lcd_home();
                printf("0 = Most Recent");
                __lcd_newline();
                printf("Select Log: ");
                unsigned int codeFlag = 0;
                unsigned int log = 10;
                while(codeFlag == 0) {

                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                    unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                    Nop();  // Apply breakpoint here to prevent compiler optimizations

                    unsigned char temp = keys[keypress];

                    if((temp == '#') && (log != 10)) {
                        load_data(1);
                        if((load[39] + 1) < log) {
                            __lcd_clear();
                            __lcd_home();
                            printf("Absent Log.");
                            __lcd_newline();
                            printf("Press: Continue");
                            transmitStringUART("Absent Log.\r\n");
                            while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                            while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                            Nop();  // Apply breakpoint here to prevent compiler optimizations
                        }
                        else {
                            unsigned int newLog = (load[20] + 1) - log;
                            load_data(newLog);
                            if(load[1] == 255) {
                                __lcd_clear();
                                __lcd_home();
                                printf("Absent Log.");
                                __lcd_newline();
                                printf("Press: Continue");
                                transmitStringUART("Absent Log.\r\n");
                                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                                Nop();  // Apply breakpoint here to prevent compiler optimizations
                            }
                            else {
                                unsigned char tempString[6] = "Log: ";
                                spiInit(16);
                                tempString[5] = log + '0';
                                GLCDtext(tempString, 1);
                                I2C_Master_Init(100000);
                                
                                __lcd_clear();
                                __lcd_home();
                                printf("Log %d Remaining:", log);
                                __lcd_newline();
                                printf("B:%d N:%d S:%d W:%d", load[1], load[2], load[3], load[4]);
                                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                                Nop();  // Apply breakpoint here to prevent compiler optimizations
                                    
                                __lcd_clear();
                                __lcd_home();
                                printf("Log %d  %02x/%02x/%02x", log, load[5], load[6], load[7]);
                                __lcd_newline();
                                printf("Began: %02x:%02x:%02x", load[8], load[9], load[10]);
                                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                                Nop();  // Apply breakpoint here to prevent compiler optimizations
                                
                                __lcd_clear();
                                __lcd_home();
                                printf("Log %d Run Time:", log);
                                __lcd_newline();
                                if(load[11] < 10) {
                                    printf("0");
                                }
                                printf("%d:", load[11]);
                                if(load[12] < 10) {
                                    printf("0");
                                }
                                printf("%d", load[12]);
                                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                                Nop();  // Apply breakpoint here to prevent compiler optimizations
                                
                                unsigned int Btotal;
                                unsigned int Ntotal;
                                unsigned int Stotal;
                                unsigned int Wtotal;
                                unsigned int x = 0;
                                for(unsigned int l = 13; l < 28; l += 2) {
                                    Btotal = (load[l]/10);
                                    Ntotal = load[l] - (Btotal * 10);
                                    Stotal = (load[l + 1]/10);
                                    Wtotal = load[l + 1] - (Stotal * 10);
                                    __lcd_clear();
                                    __lcd_home();
                                    printf("Log %d, C%d", log, l - (12 + x));
                                    __lcd_newline();
                                    printf("Set B:%dN:%dS:%dW:%d", Btotal, Ntotal, Stotal, Wtotal);
                                    x += 1;
                                    
                                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                                }
                                for(i = 29; i < 37; i++) {
                                    __lcd_clear();
                                    __lcd_home();
                                    printf("Log %d, C%d", log, i - 28);
                                    __lcd_newline();
                                    printf("Sets/Step: ");
                                    printf("%d", load[i]);       
                                    
                                    while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                                    while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                                    Nop();  // Apply breakpoint here to prevent compiler optimizations
                                }
                                __lcd_clear();
                                __lcd_home();
                                printf("Log %d", log);
                                __lcd_newline();
                                printf("Assembly Steps:%d", load[37]);
                                while(PORTBbits.RB1 == 0){  continue;   } // Waiting for key press
                                while(PORTBbits.RB1 == 1){  continue;   } // Wait for key press release
                                Nop();  // Apply breakpoint here to prevent compiler optimizations
                            }
                        }

                        codeFlag = 1;
                    }
                    else if((temp == '0') || (temp == '1') || (temp == '2') || (temp == '3') || (temp == '4') || (temp == '5') || (temp == '6')) {
                        log = temp - '0';
                        lcd_set_cursor(13, 1);
                        putch(temp);
                    }
                    else if(temp == '*') {
                        lcd_set_cursor(13, 1);
                        putch(' ');
                        log = 10;
                    }
                }
            }
            // </editor-fold>
        }
        spiInit(16);
        glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, YELLOW);
        GLCDtext(" STANDBY ", 2);
        GLCDtext("  MODE   ", 3);
        I2C_Master_Init(100000);
        // </editor-fold>
    }
    return;
}

// <editor-fold defaultstate="collapsed" desc="Interrupts">
// Interrupt Handler::
void interrupt interruptHandler(void){
    // Interrupts:
    if(INT1IF){
        // Interrupt handler for RB1 (keypad press)
        loopFlag = 1;
        unsigned char keypress = (PORTB & 0xF0) >> 4; // Read 4-bit key press
        unsigned char temp = keys[keypress];
//        if((temp == '*') && operation) {
//            // Emergency stop
//            emergency = 1;
//            spiInit(16);
//            glcdDrawRectangle(0, GLCD_SIZE_HORZ, 0, GLCD_SIZE_VERT, RED);
//            GLCDtext("OPERATION", 2);
//            GLCDtext("CANCELLED", 3);
//            I2C_Master_Init(100000);
//            __lcd_clear();
//            __lcd_home();
//            printf("OPERATION");
//            __lcd_newline();
//            printf("CANCELLED");
//            __delay_ms(2000);
//            vibrationN(0);
//        }
        INT1IF = 0;  // Clear interrupt flag bit
    }
    // 
    if(PIR1bits.TMR1IF) {   
        // Interrupt handler for timer1 overflow
        PIR1bits.TMR1IF = 0; //  Clear interrupt flag bit
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="RTC Set Time Function">
void RTC_setTime(void){
    /* Writes the date_time array to the RTC memory.
     *
     * Arguments: none
     *
     * Returns: none
     */
    
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); // Set memory pointer to seconds
    
    /* Write array. */
    for(char i=0; i<7; i++){
        I2C_Master_Write(date_time[i]);
    }
    
    I2C_Master_Stop(); //Stop condition
}
// </editor-fold>