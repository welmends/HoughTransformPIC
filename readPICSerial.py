## Serial RX to recieve data from PIC
#Run: sudo python3 readPICSerial.py
#Requirements: pyserial, serial and Serial

#libs
import serial
from serial import Serial
import array

#control variables
#Invert accumulator size because of the optimization
width     = 55 #180
height    = 180#55 #20x20 input image
contW     = 0
contH     = 0
pixel     = 0
condition = True

#main
if __name__ == "__main__":
    #init serial.Serial on rx (port on /dev/ttyUSB0)
    rx = serial.Serial(port = "/dev/ttyUSB0", baudrate = 9600, timeout = 0)
    #open .txt file to write accumulator matrix
    imageFile = open("accumulator.txt","w")

    #while do not receive the last pixel
    while condition:
        #try to get something in the serial
        pixel = rx.read(1)
        #caught something in the serial
        if(pixel != b''):
            #print(ord(pixel))
            if(contW==0 and contH==0):
                print('Reading from PIC Serial...')
            contW+=1
            #write in file
            imageFile.write(str(ord(pixel)))
            if(contW==width):
                print('Line '+str(contH+1)+'/'+str(height))
                #write in file
                imageFile.write("\n")
                contW=0
                contH+=1
            else:
                #write in file
                imageFile.write(" ")
            if(contH==height):
                #last pixel
                condition = False
    #close .txt file of accumulator matrix
    imageFile.close()
    print('Image saved!')
