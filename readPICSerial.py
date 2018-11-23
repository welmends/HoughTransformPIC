#Run: sudo python3 readPICSerial.py
#Requirements: pyserial, serial and Serial

import serial
from serial import Serial
import array

width   = 180
height  = 55 #20x20 input image
contW   = 0
contH   = 0

if __name__ == "__main__":
    rx = serial.Serial(port = "/dev/ttyUSB0", baudrate = 9600, timeout = 0)
    imageFile = open("accumulator.txt","w")
    pixel = 0
    condition = True
    while condition:
        pixel = rx.read(1)
        if(pixel != b''):
            if(contW==0 and contH==0):
                print('Reading from PIC Serial...')
            contW+=1
            imageFile.write(str(ord(pixel)))
            #print(ord(pixel))
            if(contW==width):
                imageFile.write("\n")
                contW=0
                contH+=1
            else:
                imageFile.write(" ")
            if(contH==height):
                condition = False

    imageFile.close()
    print('Image saved!')
