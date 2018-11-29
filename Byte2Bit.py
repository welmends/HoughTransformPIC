## Generate a Bit-Pixel Matrix from Byte-Pixel Matrix
#Run: sudo python3 Byte2Bit.py

def readFromFile(path):
    imageFile = open(path,"r")
    inputImage = imageFile.read()
    imageFile.close()
    return inputImage

def getArrayOfByte(inputImage):
    bytes=[]
    byte=[]
    cont=0
    imgSize=0
    for pixel in inputImage:
        if(pixel=='0' or pixel=='1'):
            imgSize+=1
            cont+=1
            if(cont==8):
                byte.append(int(pixel))
                bytes.append(byte)
                byte=[]
                cont=0;
            else:
                byte.append(int(pixel))
    if(len(byte)!=0):
        print("Error: Input image size must by divisible by 8 (inputImage length => "+str(imgSize)+")")
        print("Example: 20x20 image has 400 elements and 400 is divisible by 8 (400/8=50)")
        exit(-1)
    return bytes

def convertBinaryByteInByte(bytes):
    array = []
    for byte in bytes:
        value = 0
        for j in range(len(byte)):
            if(byte[j]==1):
                value+=(2**(7-j))
        array.append(value)
    return array

def writeArrayToFile(array):
    arrayFile = open("Byte2Bit_Array.txt","w")
    for i in range(len(array)):
        arrayFile.write(str(array[i]))
        if(i!=len(array)-1):
            arrayFile.write(",")
    arrayFile.close()
    return

if __name__ == "__main__":
    #read inputImage
    inputImage = readFromFile("inputImage.txt")

    #generate an array of Bytes from input image
    bytes = getArrayOfByte(inputImage)

    #convert binary bytes to bytes array
    array = convertBinaryByteInByte(bytes)

    #write array to file
    writeArrayToFile(array)
