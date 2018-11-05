/**
 * @file hough_PIC.c
 * @brief Standard Hough Transform Implemantation on C language
          Platform: PIC16F18875 - Microchip (MPLAB X - XC8 C Compiler)
          Input : Path to an image, Type of the image (Canny or Original) and
                  Output type (Binary or 8 Bits)
          Output: Hough Transform Matrix (Accumulator image [PGM])
          Other: More information about the use of this code is present in
                 instructions method
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
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
//                                   Macros                                   //
////////////////////////////////////////////////////////////////////////////////
#define _XTAL_FREQ 32000000
#define F_CPU 32000000/64//#define Baud_value(baud_rate) (((float)(F_CPU)/(float)baud_rate)-1)
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)//calculo do taxa de transmissão serial para Uart

//#define M_PI            (3.14159265358979323846)//PI value approximation
#define IMAGE_PATH_SIZE (100)//Standard size for image path
#define THRESH_VALUE    (200)//Standard thresh value
#define MAX_ROWS        (20)//Maximum size for rows
#define MAX_COLS        (20)//Maximum size for cols

////////////////////////////////////////////////////////////////////////////////
//                              Global Variables                              //
////////////////////////////////////////////////////////////////////////////////
int j=0;

////////////////////////////////////////////////////////////////////////////////
//                                  Structs                                   //
////////////////////////////////////////////////////////////////////////////////
typedef struct{
  char magicNumber[3];
  int rows,cols,grayscale;
  unsigned char pM[MAX_ROWS*MAX_COLS];
}Matrix;

////////////////////////////////////////////////////////////////////////////////
//                                 Prototypes                                 //
////////////////////////////////////////////////////////////////////////////////
void instructions(void);
void houghTransform(Matrix *image, Matrix *accumulator, char out_type);

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

//    LATAbits.LATA4 = 0;
//    LATAbits.LATA5 = 1;
//    LATAbits.LATA6 = 1;            
//    LATAbits.LATA7 = 0;
//    while (1);
    
    while (1) {
        j++;
      LATA = 0x90;
//    LATAbits.LATA4 = 0;
//    LATAbits.LATA5 = 1;
//    LATAbits.LATA6 = 1;            
//    LATAbits.LATA7 = 0;
    __delay_ms(500);  
      LATA = 0x60;
//    LATAbits.LATA4 = 1;
//    LATAbits.LATA5 = 0;
//    LATAbits.LATA6 = 0;            
//    LATAbits.LATA7 = 1;
    __delay_ms(500);
    }
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: Print on screen the instructions to use this program.
 * @param: void
 * @return: void
 */
void instructions(void){
  printf("\n");
  printf(" **************************************************************************** \n");
  printf("|                               [Instructions]                               |\n");
  printf("| Compile: arm-unknown-linux-gnueabi-gcc src/hough_ARM.c -o objs/hough_ARM.o |\n");
  printf("| Command: ./objs/hough_ARM.o <type> <out_type> <image_name>                 |\n");
  printf("| Warning: The PGM image must be in 'images' folder                          |\n");
  printf("| Example: ./objs/hough_ARM.o canny binary lines                             |\n");
  printf("| Example: ./objs/hough_ARM.o original 8bits lines                           |\n");
  printf("|                                                                            |\n");
  printf("| <type>       : canny or original                                           |\n");
  printf("| <out_type>   : binary or 8bits                                             |\n");
  printf("| <image_name> : Any PGM image                                               |\n");
  printf("|                                                                            |\n");
  printf("| OBS: The output image will be generated in 'results' folder                |\n");
  printf(" **************************************************************************** \n");
  printf("\n");
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: This method performs all the calculus related to the Hough Transform.
 *         This occurs from the image scanning and calculation of the
 *         rho = xcos (theta) + ysin (theta) [0 < theta < 180] for pixels that
 *         display the high level (>THRESH_VALUE). It is worth mentioning that
 *         if the image passed as an input is not binarized, a threshold will
 *         be applied to it with a default threshold value set in THRESH_VALUE.
 *         More information: http://homepages.inf.ed.ac.uk/rbf/HIPR2/hough.htm
 *                           PATENT US3069654A - Paul V C Hough
 * @param: Matrix *image       - input image Matrix
           Matrix *accumulator - variable to store the accumulator
           char out_type       - output type chosen
 * @return: void
 */
void houghTransform(Matrix *image, Matrix *accumulator, char out_type){
    // Calculate the height and width of the Hough accumulator
    // accu_width  = 0 to 180 degrees
    // accu_height = 2*D - 1 and D = sqrt(height^2 + width^2)
	accumulator->rows        = 180;
    accumulator->cols        = ceil(2*(sqrt(image->rows*image->rows + image->cols*image->cols))) - 1;
    accumulator->grayscale   = image->grayscale;
    strcpy(accumulator->magicNumber,image->magicNumber);

    // Go to each pixel with hight level (>THRESH_VALUE) and calculate Rho to each Theta
	float rho;
	int theta,i,j;
	for(j=0; j<image->cols; j++){
		for(i=0; i<image->rows; i++){
			if(image->pM[ (j*image->rows) + i] > THRESH_VALUE){
				for(theta=0; theta<180; theta++){
					// rho = xcos(theta) + ysin(theta) [theta is in radians]
                    rho = ( (j)*cos((theta)*M_PI/180.0) ) + ( (i)*sin(theta*M_PI/180.0) );

                    if(out_type=='b'){
                        // accumulator(theta,rho+D) = High Level
                        accumulator->pM[ (int)((ceil(rho + accumulator->cols/2) * 180.0)) + 180-theta-1] = accumulator->grayscale;
                    }
                    else{
                        // accumulator(theta,rho+D)++
                        accumulator->pM[ (int)((ceil(rho + accumulator->cols/2) * 180.0)) + 180-theta-1]++;
                    }
				}
			}
		}
	}
}