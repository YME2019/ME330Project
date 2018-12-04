/*
 * File:   Main.c
 * Author: cbrettj
 *
 * Created on December 3, 2018, 6:05 PM
 */


#include "xc.h"
#pragma config FNOSC = FRC       // 8 MHz FRC oscillator
#define FCY 4000000uL
#include <libpic30.h>
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

//Define Various useful names and such
#define BUTTONS_PUSHED _RA0 
#define COLOR_SORT ADC1BUF1 
#define BLACK_AMMO ADC1BUF14 
#define WHITE_AMMO ADC1BUF15 
#define FRONT_IR ADC1BUF12
#define RIGHT_IR ADC1BUF11
#define R_MOTOR_DIR _LATB2
#define L_MOTOR_DIR _LATA2
#define LED _LATA4
#define BLACK_SOLENOID _LATB7
#define WHITE_SOLENOID _LATB8
#define GUN _LATA6 // Potentially OC1R instead
#define STEPPER_SIGNAL OC2R
#define SORTER OC3R



void configPinIO(void)
{
    TRISA=0x2B;
    ANSA=0x0A;
    TRISB=0xF018;
    ANSB=0x3010;
}
void configADC(void)
{
    


	/*** Select Voltage Reference Source ***/
	// use AVdd for positive reference
	_PVCFG = 00;		// AD1CON2<15:14>, pg. 212-213 datasheet
	// use AVss for negative reference
	_NVCFG = 0;			// AD1CON2<13>


	/*** Select Analog Conversion Clock Rate ***/
	// make sure Tad is at least 600ns, see Table 29-41 datasheet
	_ADCS = 0b00000011;	// AD1CON3<7:0>, pg. 213 datasheet


	/*** Select Sample/Conversion Sequence ***/
	// use auto-convert
	_SSRC = 0b0111;		// AD1CON1<7:4>, pg. 211 datasheet
	// use auto-sample
	_ASAM = 1;			// AD1CON1<2>
	// choose a sample time >= 1 Tad, see Table 29-41 datasheet
	_SAMC = 0b00001;		// AD1CON3<12:8>


	/*** Choose Analog Channels to be Used ***/
	// scan inputs
	_CSCNA = 1;			// AD1CON2<10>
	// choose which channels to scan, e.g. for ch AN12, set _CSS12 = 1;
	_CSS1 = 1;			// AD1CSSH/L, pg. 217
    _CSS14 = 1;
    _CSS15 = 1;
    _CSS12 = 1;
    _CSS11 = 1;
    
	/*** Select How Results are Presented in Buffer ***/
	// set 12-bit resolution
	_MODE12 = 1;		// AD1CON1<10>
	// use absolute decimal format
	_FORM = 00;			// AD1CON1<9:8>
	// load results into buffer determined by converted channel, e.g. ch AN12 
    // results appear in ADC1BUF12
	_BUFREGEN = 1;		// AD1CON2<11>


	/*** Select Interrupt Rate ***/
	// interrupt rate should reflect number of analog channels used, e.g. if 
    // 5 channels, interrupt every 5th sample
	_SMPI = 0b00101;		// AD1CON2<6:2>


	/*** Turn on A/D Module ***/
	_ADON = 1;			// AD1CON1<15>
}
void configPWM1(void)
{
    //-----------------------------------------------------------
    // CONFIGURE PWM1 USING OC1 (on pin 14)
    
    // Clear control bits initially
    OC1CON1 = 0;
    OC1CON2 = 0;
   
    // Set period and duty cycle
    OC1R = 2000;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC1RS = 3999;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC1CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
                                // (self synchronization) -- Although we
                                // selected the system clock to determine
                                // the rate at which the PWM timer increments,
                                // we could have selected a different source
                                // to determine when each PWM cycle initiates.
                                // From the FRM: When the SYNCSEL<4:0> bits
                                // (OCxCON2<4:0>) = 0b11111, they make the
                                // timer reset when it reaches the value of
                                // OCxRS, making the OCx module use its
                                // own Sync signal.
    OC1CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC1CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    
    
    //-----------------------------------------------------------
    // RUN
}
void configPWM2 (void)
{
     //-----------------------------------------------------------
    // CONFIGURE PWM2 USING OC2 (on pin 4)
    
    // Clear control bits initially
    OC2CON1 = 0;
    OC2CON2 = 0;
   
    // Set period and duty cycle
    OC2R = 0;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC2RS = 4443;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC2CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC2CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
                                // (self synchronization) -- Although we
                                // selected the system clock to determine
                                // the rate at which the PWM timer increments,
                                // we could have selected a different source
                                // to determine when each PWM cycle initiates.
                                // From the FRM: When the SYNCSEL<4:0> bits
                                // (OCxCON2<4:0>) = 0b11111, they make the
                                // timer reset when it reaches the value of
                                // OCxRS, making the OCx module use its
                                // own Sync signal.
    OC2CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    
    
    //-----------------------------------------------------------
    // RUN
}
void configPWM3 (void)
{
     //-----------------------------------------------------------
    // CONFIGURE PWM3 USING OC3 (on pin 5)
    
    // Clear control bits initially
    OC3CON1 = 0;
    OC3CON2 = 0;
   
    // Set period and duty cycle
    OC3R = 2000;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC3RS = 3999;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC3CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC3CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
                                // (self synchronization) -- Although we
                                // selected the system clock to determine
                                // the rate at which the PWM timer increments,
                                // we could have selected a different source
                                // to determine when each PWM cycle initiates.
                                // From the FRM: When the SYNCSEL<4:0> bits
                                // (OCxCON2<4:0>) = 0b11111, they make the
                                // timer reset when it reaches the value of
                                // OCxRS, making the OCx module use its
                                // own Sync signal.
    OC3CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC3CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    
    
    //-----------------------------------------------------------
    // RUN
}
void driveFWD (void)
{
    R_MOTOR_DIR=0;
    L_MOTOR_DIR=1;
    STEPPER_SIGNAL=2222;
    
}
void driveBACK (void)
{
    R_MOTOR_DIR=1;
    L_MOTOR_DIR=0;
    STEPPER_SIGNAL=2222;
    
}
void driveSTOP (void)
{
   
    STEPPER_SIGNAL=0;
    R_MOTOR_DIR=0;
    L_MOTOR_DIR=0;
    
}
void driveCCW (void)
{
    R_MOTOR_DIR=0;
    L_MOTOR_DIR=0;
    STEPPER_SIGNAL=2222;
    
}
void driveCW (void)
{
    R_MOTOR_DIR=1;
    L_MOTOR_DIR=1;
    STEPPER_SIGNAL=2222;
    
}
int main() {
    //Finite State Machine
        //Define State Variables
        enum {INITIALIZE, ORIENT, LOAD_NAV, LOAD, AIM, FIRE} state;
        state=INITIALIZE;
    //Switch Statement
       
        while(1)
        {
            switch(state){
            case INITIALIZE:
                configPinIO();
                LED=1;
                __delay_us(200000)
                configADC();
                LED=0;
                __delay_us(200000)
                configPWM1();
                configPWM2();
                configPWM3();
                LED=1;
                __delay_us(100000)
                LED=0;
                __delay_us(100000)
                        LED=1;
                __delay_us(100000)
                LED=0;
                __delay_us(100000)
                        LED=1;
                __delay_us(100000)
                LED=0;
                __delay_us(100000)
                        LED=1;
                __delay_us(100000)
                LED=0;
                __delay_us(100000)
                        LED=1;
                __delay_us(100000)
                LED=0;
                __delay_us(100000)
                LED=1;
                __delay_us(5000000)
                state=ORIENT;
                break;
            case ORIENT:
                LED=0;
                __delay_us(100000);
                driveFWD();
                __delay_us(5000000)
                driveBACK();
                __delay_us(5000000)
                driveSTOP();
                __delay_us(5000000)
                driveCCW();
                __delay_us(5000000)
                driveCW();
                __delay_us(5000000)
                driveSTOP();
                while(1)
                {   __delay_us(250000)
                    LED=1;
                    __delay_us(250000)
                    LED=0; 
                }
                break;
            case LOAD_NAV:
                                break;
            case LOAD:
                break;
            case AIM:
                break;
            case FIRE:
                break;
        }
    
       
        
        }
    
    return 0;
}
