/**
 * @file hough_PIC.c
 * @brief Standard Hough Transform Implemantation on C language
          Platform: PIC16F18875 - Microchip (MPLAB X - XC8 C Compiler)
          Input : ...
          Output: ...
          Other : ...
 * @author $Author:$ de Souza, Joao Wellington Mendes; Brito, Messyo Sousa
 * @version $Revision:$
 * @date $Date:$ Created on 05/11/2018 and Last Update on 05/11/2018
 */
// Production -> Set Configuration Bits;
// PIC16F18875 Configuration Bit Settings
// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

////////////////////////////////////////////////////////////////////////////////
//                                  Libraries                                 //
////////////////////////////////////////////////////////////////////////////////
#include <pic16f18875.h>
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
//                                   Macros                                   //
////////////////////////////////////////////////////////////////////////////////
#define _XTAL_FREQ 32000000
#define F_CPU 32000000/64//#define Baud_value(baud_rate) (((float)(F_CPU)/(float)baud_rate)-1)
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)//calculo do taxa de transmissão serial para Uart

//#define M_PI            (3.14159265358979323846)//PI value approximation
#define THRESH_VALUE    (200)//Standard thresh value
#define GRAYSCALE       (255)//Levels of grayscale
#define ROWS            (20)//Default size for rows
#define COLS            (20)//Default size for cols

////////////////////////////////////////////////////////////////////////////////
//                              Global Variables                              //
////////////////////////////////////////////////////////////////////////////////
unsigned char inputImage[400];//20x20 Matrix

////////////////////////////////////////////////////////////////////////////////
//                                 Prototypes                                 //
////////////////////////////////////////////////////////////////////////////////
void houghTransform();

////////////////////////////////////////////////////////////////////////////////
//                                Source Code                                 //
////////////////////////////////////////////////////////////////////////////////

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: The main method will call all other methods aiming to perform all
 *         operations prescribed. Also, the main method receives as input the
 *         arguments.
 * @param: void
 * @return: int
 */
int main(void) {
    TRISA = 0x0F;
    LATA = 0x00;
    LATAbits.LATA7 = 1;// Start process - Bit LATA7 HIGH
    
    //Process
    
    LATAbits.LATA7 = 0;
    LATAbits.LATA6 = 1;// End process   - Bit LATA6 HIGH
    return 0;
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: This method performs all the calculus related to the Hough Transform.
 *         This occurs from the image scanning and calculation of the
 *         rho = xcos (theta) + ysin (theta) [0 < theta < 180] for pixels that
 *         display the high level (>THRESH_VALUE). It is worth mentioning that
 *         if the image passed as an input is not binary, a threshold will
 *         be applied to it with a default threshold value set in THRESH_VALUE.
 *         More information: http://homepages.inf.ed.ac.uk/rbf/HIPR2/hough.htm
 *                           PATENT US3069654A - Paul V C Hough
 * @param: void
 * @return: void
 */
void houghTransform(void){
    // Calculate the height and width of the Hough accumulator
    // accu_width  = 0 to 180 degrees
    // accu_height = 2*D - 1 and D = sqrt(height^2 + width^2)
    //accumulator->cols        = ceil(2*(sqrt(ROWS*ROWS + COLS*COLS))) - 1;

    // Go to each pixel with hight level (>THRESH_VALUE) and calculate Rho to each Theta
	double rho,cosTheta,sinTheta;
	int rhoD,theta,j,i;
    int accu_height = ceil(2*(sqrt(ROWS*ROWS + COLS*COLS))) - 1;
    for(theta=0; theta<180; theta++){
        cosTheta=cos(theta*M_PI/180);
        sinTheta=sin(theta*M_PI/180);
        for(rhoD=0; rhoD<accu_height; rhoD++){
            for(j=0; j<COLS; j++){
                for(i=0; i<ROWS; i++){
                    if(inputImage[ (j*ROWS) + i] > THRESH_VALUE){
                        // rho = xcos(theta) + ysin(theta) [theta is in radians]
                        rho = ( (j)*cosTheta ) + ( (i)*sinTheta );
                        if(ceil(rho + accu_height/2)==rhoD){
                            //accumulator->pM[ (int)(rhoD*180.0) + 180-theta-1]++;
                        }
                    }
                }
            }
        }
    }
}