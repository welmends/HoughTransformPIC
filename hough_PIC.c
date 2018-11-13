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
#define TX LATCbits.LATC6//PIN that we use to trasmit serial on PIC

// Since the inputImage is default, we calculate the Accumulator size
// The width and height of the Hough accumulator must be:
#define ACCU_WIDTH      (180) //ACCU_WIDTH  = 180 to 1 degrees
#define ACCU_HEIGHT     (55)  //ACCU_HEIGHT = 2*D - 1 and D = sqrt(WIDTH^2 + HEIGHT^2)
#define WIDTH           (20)  //Default input image width
#define HEIGHT          (20)  //Default input image height
#define THRESH_VALUE    (1)   //Standard thresh value
#define SIMULATOR       (1)   //1 - Use Simulator | 0 - Don't use Simulator

////////////////////////////////////////////////////////////////////////////////
//                              Global Variables                              //
////////////////////////////////////////////////////////////////////////////////
//Input image stored in 'const unsigned char DataInput[400]' (20x20 = 400 bytes)
const unsigned char DataInput[400] = {
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,
0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,
0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

////////////////////////////////////////////////////////////////////////////////
//                                 Prototypes                                 //
////////////////////////////////////////////////////////////////////////////////
void init(void);
void startProcessLED(void);
void endProcessLED(void);
void putch(unsigned char data);
void sendByteThroughPin(unsigned char byte);
void UARTTransmitter(unsigned char byte, int theta);
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
    init();           // Initialize
    startProcessLED();// Indicates that process will start now
    houghTransform(); // Algorithm in process
    endProcessLED();  // Indicates that process have ended    
    while(1);         // Prevent from starting over

    return 0;
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: This method initialize some registers and pins.
 * @param:  void
 * @return: void
 */
void init(void) {
    //Initial Setup
    TRISA = 0x0F;
    TRISC = 0x00;
    LATA = 0x00;
    //Simulator
    TXSTAbits.TXEN = 1;//Enable transmitter
    RCSTAbits.SPEN = 1;//Enable serial port
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: This method set bit LATA7 to high. This indicates that process will 
 *         start now.
 * @param:  void
 * @return: void
 */
void startProcessLED(void){
    //Start process
    LATAbits.LATA7 = 1;// Bit LATA7 HIGH
    LATAbits.LATA6 = 0;// Bit LATA6 LOW
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
 *         the formatted text to stdout.(Simulator only)
 *         More information: http://microchipdeveloper.com/xc8:console-printing
 * @param:  unsigned char
 * @return: void
 */
void putch(unsigned char data) {
    while(!TRMT);//Waiting for previous data to transmit completely
    TXREG = data;//Writing data to Transmit Register and starts transmission
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: The 'sendBytePin' method implements UART protocol to transmit a byte. 
 *         This protocol is used to transmit and receive bytes between interfaces
 *         and here we implement the transmit part. The delay we use was defined
 *         by 1/9600 sec (104us), and 9600 was the baud rate used by default.
 *         More information: https://www.solitontech.com/uart-protocol-validation-service/ 
 * @param:  unsigned char
 * @return: void
 */
void sendBytePin(unsigned char byte){
	TX = 1;//Idle
    __delay_us(104);// 1/9600 sec ~= 104 us
	TX = 1;//Idle
	__delay_us(104);
	TX = 0;//Start bit
	__delay_us(104);
    unsigned char i;
	for(i=0;i<=7;i++){
		TX = (byte);//Data bits
		byte=byte>>1;
		__delay_us(104);
	}
	TX = 1;//Stop bit
	__delay_us(104);
	TX = 1;//Idle
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: Here we separate when we transmit with the simulator and when we 
 *         transmit on PIC TX Pin. Also, we set the end of the line and space 
 *         between pixels of the output matrix.
 * @param:  unsigned char
 * @return: void
 */
void UARTTransmitter(unsigned char byte, int theta){
    if(SIMULATOR){//SIMULATOR macro equals 1
        printf("%u",byte);//transmit byte (pixel) (Simulator) 
        if(theta==1){
            printf("\n");//transmit byte (pixel) (Simulator) 
        }else{
            printf(" ");//space between pixels (Simulator) 
        }
    }else{//SIMULATOR macro equals 0
        sendBytePin(byte);//transmit byte (pixel) (PIC))
        if(theta==1){
            sendBytePin('\n');//end of output matrix line (PIC)
        }else{
            sendBytePin(' ');//space between pixels (PIC)
        }
    }
}

/**
 * @author: Joao Wellington and Messyo Sousa
 * @brief: This method performs all the calculus related to the Hough Transform.
 *         This occurs from the image scanning and calculation of the
 *         rho = xcos (theta) + ysin (theta) [0 < theta < 180] for pixels that
 *         display the high level (THRESH_VALUE==1).
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
    // Calculate each pixel of Accumulator by calculating rho of each image 
    // pixel with high level (THRESH_VALUE==1) and comparing with current value 
    // of rhoD by adding with ACCU_HEIGHT/2 (or D).
    // OBS.: This is an adjustment to avoid storage of the accumulator matrix.
	
    int rhoD,theta,j,i;//Iteration variables
    float rho,cosTheta,sinTheta;//Calculus variables
    unsigned char accumulator_pixel;//Accumulator pixel variable
    unsigned char *inputImage;//Pointer to DataInput
    inputImage=(unsigned char *)DataInput;//Get reference of DataInput
    
    for(rhoD=0; rhoD<ACCU_HEIGHT; rhoD++){
        for(theta=ACCU_WIDTH; theta>=1; theta--){
            //Avoid calculating these same values for cosine
            cosTheta = cos(theta*M_PI/180);
            //Avoid calculating these same values for sine
            //OBS.: sin(x) = cos(90-x) (Do this to save program space)
            sinTheta = cos((90-theta)*M_PI/180);
            //Accumulator pixel reset
            accumulator_pixel = 0;
            for(j=0; j<WIDTH; j++){
                for(i=0; i<HEIGHT; i++){
                    if(inputImage[ (j*HEIGHT) + i] == THRESH_VALUE){
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
            //Transmit byte by byte (pixel by pixel) with UART Serial Output
            UARTTransmitter(accumulator_pixel, theta);
        }
    }
}