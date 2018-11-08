/**
 * @file hough_PIC.c
 * @brief Standard Hough Transform Implemantation on C language
          Platform: PIC16F18875 - Microchip (MPLAB X - XC8 C Compiler)
          Input : 8-bit 20x20 Matrix
          Output: Accumulator Matrix passed through serial output
          Other : ...
 * @author $Author:$ de Souza, Joao Wellington Mendes; Brito, Messyo Sousa
 * @version $Revision:$
 * @date $Date:$ Created on 04/11/2018 and Last Update on 07/11/2018
 */

////////////////////////////////////////////////////////////////////////////////
//                  PIC16F18875 - CONFIGURATION BIT SETTINGS                  //
////////////////////////////////////////////////////////////////////////////////
//===== CONFIG1 =====//
//External Oscillator mode selection bits (Oscillator not enabled)
#pragma config FEXTOSC = OFF
//Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config RSTOSC = HFINT32
//Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CLKOUTEN = OFF
//Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config CSWEN = ON
//Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)
#pragma config FCMEN = ON

//===== CONFIG2 =====//
//Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config MCLRE = ON
//Power-up Timer Enable bit (PWRT disabled)
#pragma config PWRTE = OFF
//Low-Power BOR enable bit (ULPBOR disabled)
#pragma config LPBOREN = OFF
//Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BOREN = ON
//Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config BORV = LO
//Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config ZCD = OFF
//Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config PPS1WAY = ON
//Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)
#pragma config STVREN = ON

//===== CONFIG3 =====//
//WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTCPS = WDTCPS_31
//WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTE = OFF
//WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCWS = WDTCWS_7
//WDT input clock selector (Software Control)
#pragma config WDTCCS = SC

//===== CONFIG4 =====//
//UserNVM self-write protection bits (Write protection off)
#pragma config WRT = OFF
//Scanner Enable bit (Scanner module is available for use)
#pragma config SCANE = available
//Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)
#pragma config LVP = ON

//===== CONFIG5 =====//
//UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CP = OFF
//DataNVM code protection bit (Data EEPROM code protection disabled)
#pragma config CPD = OFF

//OBS1: #pragma config statements should precede project file includes.
//OBS2: Use project enums instead of #define for ON and OFF.

////////////////////////////////////////////////////////////////////////////////
//                                  Libraries                                 //
////////////////////////////////////////////////////////////////////////////////
#include <xc.h>
#include <pic16f18875.h>
#include <stdio.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
//                                   Macros                                   //
////////////////////////////////////////////////////////////////////////////////
#define _XTAL_FREQ 32000000
#define F_CPU 32000000/64//#define Baud_value(baud_rate) (((float)(F_CPU)/(float)baud_rate)-1)
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)//calculus for UART serial tramission rate

// Since the inputImage is default, we calculate the Accumulator size
// The width and height of the Hough accumulator must be:
#define ACCU_WIDTH      (180) //ACCU_WIDTH  = 0 to 180 degrees
#define ACCU_HEIGHT     (56)  //ACCU_HEIGHT = ceil(2*D) - 1 and D = sqrt(WIDTH^2 + HEIGHT^2)
#define WIDTH           (20)  //Default input image width
#define HEIGHT          (20)  //Default input image height
#define THRESH_VALUE    (200) //Standard thresh value

////////////////////////////////////////////////////////////////////////////////
//                              Global Variables                              //
////////////////////////////////////////////////////////////////////////////////
//Input image stored in '_ROM unsigned char DataInput[400]' (20x20 = 400 bytes)
const unsigned char DataInput[400] = {
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255, 0 ,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255, 0 ,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255, 0 ,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255, 0 ,255,255,255,255,255,255,255,
255,255,255,255, 0 ,255,255,255,255,255,255, 0 ,255,255,255,255,255,255,255,255,
255,255,255,255,255, 0 ,255,255,255,255, 0 ,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255, 0 ,255,255, 0 ,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255, 0 , 0 ,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255, 0 , 0 ,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255, 0 ,255,255, 0 ,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255, 0 ,255,255,255,255, 0 ,255,255,255,255,255,255,255,255,255,
255,255,255,255, 0 ,255,255,255,255,255,255, 0 ,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255, 0 ,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255, 0 ,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255, 0 ,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255, 0 ,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
};

////////////////////////////////////////////////////////////////////////////////
//                                 Prototypes                                 //
////////////////////////////////////////////////////////////////////////////////
void startProcessLED(void);
void endProcessLED(void);
void putch(unsigned char data);
void init_uart(void);
void houghTransform(void);

////////////////////////////////////////////////////////////////////////////////
//                                Source Code                                 //
////////////////////////////////////////////////////////////////////////////////

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: The main method will call all other methods aiming to perform all
 *         operations prescribed. Also, the main method receives as input the
 *         arguments.
 * @param:  void
 * @return: int
 */
int main(void) {
    init_uart();
    startProcessLED();// Indicates that process will start now
    houghTransform(); // Algorithm in process
    endProcessLED();  // Indicates that process have ended

    return 0;
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: This method set bit LATA7 to high. This indicates that process will 
 *         start now.
 * @param:  void
 * @return: void
 */
void startProcessLED(void){
    //Initial Setup
    TRISA = 0x0F;
    LATA = 0x00;
    //Start process
    LATAbits.LATA7 = 1;// Bit LATA7 HIGH
    return;
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: This method set bit LATA6 to high and LATA7 to low. This indicates 
 *         that process have ended.
 * @param:  void
 * @return: void
 */
void endProcessLED(void){
    //End Process
    LATAbits.LATA7 = 0;// Bit LATA7 LOW
    LATAbits.LATA6 = 1;// Bit LATA6 HIGH
    return;
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: The 'putch' method is called by 'printf' to send each character of 
 *         the formatted text to stdout.
 *         More information: http://microchipdeveloper.com/xc8:console-printing
 * @param:  unsigned char
 * @return: void
 */
void putch(unsigned char data) {
    TXREG = data;// send one character
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: This method initialize UART by enabling transmitter and serial port.
 *         More information: http://microchipdeveloper.com/xc8:console-printing
 * @param:  void
 * @return: void
 */
void init_uart(void) {
    TXSTAbits.TXEN = 1; // Enable transmitter
    RCSTAbits.SPEN = 1; // Enable serial port
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: This method performs all the calculus related to the Hough Transform.
 *         This occurs from the image scanning and calculation of the
 *         rho = xcos (theta) + ysin (theta) [0 < theta < 180] for pixels that
 *         display the low level (<THRESH_VALUE).
 *         In this version we made some adjustments to the algorithm return the 
 *         value of each accumulator matrix pixel in each iteration and transmit 
 *         each pixel in UART serial output. This increases the algorithm 
 *         processing cost but presented a memory-saving because there is no 
 *         longer a need to store the accumulator matrix.
 *         More information: http://homepages.inf.ed.ac.uk/rbf/HIPR2/hough.htm
 *                           PATENT US3069654A - Paul V C Hough
 * @param:  void
 * @return: void
 */
void houghTransform(void){
    // Calculate each pixel of Accumulator by calculating rho of each image pixel 
    // with hight level (>THRESH_VALUE) and comparing with current value of rhoD
    // by adding with ACCU_HEIGHT/2 (or D).
    // OBS.: This is an adjustment to avoid storage of the accumulator.
	
    int rhoD,theta,j,i;//Iteration variables
    float rho,cosTheta,sinTheta;//Calculus variables
    unsigned char accumulator_pixel;//Accumulator pixel variable
    unsigned char *inputImage;//Pointer to DataInput
    inputImage=(unsigned char *)DataInput;//Get reference of DataInput
    
    for(theta=0; theta<ACCU_WIDTH; theta++){
        //Avoid calculating these same values for cosine
        cosTheta=cos(theta*M_PI/180);
        //Avoid calculating these same values for sine
        sinTheta=cos(90 - (theta*M_PI/180));
        //OBS.: sin(x) = cos(90-x) (Do this to save program space)
        for(rhoD=0; rhoD<ACCU_HEIGHT; rhoD++){
            //Accumulator pixel reset
            accumulator_pixel = 0;
            for(j=0; j<WIDTH; j++){
                for(i=0; i<HEIGHT; i++){
                    if(inputImage[ (j*HEIGHT) + i] < THRESH_VALUE){
                        //rho = xcos(theta) + ysin(theta) [theta is in radians]
                        rho = ( (j)*cosTheta ) + ( (i)*sinTheta );
                        
                        //rho+D must be equal to the current rhoD
                        if(ceil(rho + ACCU_HEIGHT/2)==rhoD){
                            //accumulator[rho+D,theta]++
                            accumulator_pixel++;
                        }
                    }
                }
            }
            //UART_OUT - accumulator_pixel out ACCU_WIDTH X ACCU_HEIGHT Times
            printf("%u",accumulator_pixel);
        }
        printf("\n");
    }
}