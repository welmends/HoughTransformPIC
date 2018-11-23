# Hough Transform - C Implementation on PIC16F18875 Platform

Hough Transform Implementation in C language that provides the Hough Accumulator Matrix as serial output (UART)

### Prerequisites

- XC8 Compiler on MPLAB X IDE

- PIC16F18875

- Curiosity board (Optional)

### Compiling

- Open the MPLAB X IDE

- Include the folder project

- Clean and Build the project

### Executing

#### Method 1
- Select in project properties to run as Simulator with UART I/O Output Enabled and run as Debugger mode

#### Method 2
- Select in project properties to run as Microchip Starter Kit then select Curiosity and program the PIC16F18875. After that run the python script(*readPICSerial.py*) holding the Curiosity reset button and release the button after the script is executed so that the application starts in the PIC and the reading by the serial begins. The python script will generate a file named *accumulator.txt* that is the output of the algorithm.

### Scripts

- Run *houghComparison.m* on MATLAB after executing in *Method 2* and have generated *accumulator.txt* file to view a comparison between the accumulator generated from MATLAB implementation and this implementation on PIC.

- Run *getLines.m* on MATLAB after executing in *Method 2* and have generated *accumulator.txt* file to view a comparison between the lines plotted in the inputImage generated from MATLAB implementation and this implementation on PIC.
