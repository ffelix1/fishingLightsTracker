import csv,serial,datetime,os,cgitb,sys
from time import sleep
import numpy as np
#import RPi.GPIO as GPIO


cgitb.enable()
sys.path.insert(0, "usr/bin/espeak")
sys.path.insert(0, "usr/bin/raspistill")
den_datetime = datetime.datetime.now()
ser = serial.Serial('/dev/ttyACM', 9600)
time.sleep(5)


#asking for number of passes
numberOfPasses = int(input("Input the number of satellite passes needed (integer values): "))
print("--- You selected " + str(numberOfPasses) + " number of passes. ---")
count = 0
actualCount = 1
timeValues = np.zeros((numberOfPasses,), dtype=int)
azimuth = np.zeros((numberOfPasses,), dtype=float)
zenith = np.zeros((numberOfPasses,), dtype=float)


#asking for the exact times and the azimuth and zenith values
while count < numberOfPasses:
    timeValues[count] = int(input("Please enter the time for the number" + str(actualCount) + " pass in the format MMDDHHMMSS: "))
    azimuth[count] = int(input("Please enter the azimuth angle for the number" + str(actualCount) + " pass in degrees (decimals allowed): "))
    zenith[count] = int(input("Please enter the zenith angle for the number" + str(actualCount) + " pass in degrees (decimals allowed)"))
    actualCount += 1
    count += 1

    
#opening file to write data
filename = input("Type the name of the file that you want the file be exported to: ")
f = open(filename,'w')


#reinitiating values    
count = 0
actualCount = 1


#automation system
while count < numberOfPasses:
    f.write("Pass Number " + count + "\n")
    state = True
    while state == True:
        currentTime = int(den_datetime.strftime("%m%d%H%M%S"))
        #giving commands for servos
        if timeValues[count]-100 == currentTime:
            ser.write(azimuth[count])
            time.sleep(2)
            ser.write(zenith[count])
            time.sleep(2)
            #write text file
            f.write("Time: " + timeValues[count] + " Azimuth: " + azimuth[count] + " Zenith: " + zenith[count])
        #before
        if timeValues[count]-25 == currentTime:
            #take picture before
            filename = 'IMG_' + currentTime + '.jpg'
            os.system('raspistill -iso 800 -w 3280 -h 2464 -q 100 -r -ss 5500000 -o before' + filename)
            #take temperature reading before
            ser.write(b'1')
            time.sleep(3)
            #write text file
            f.write(" tempReadingBefore: " + ser.readline())
        #after
        if timeValues[count]+10 == currentTime:
            #take picture after
            filename = 'IMG_' + currentTime + '.jpg'
            os.system('raspistill -iso 800 -w 3280 -h 2464 -q 100 -r -ss 5500000 -o after' + filename)
            #take temperature reading after
            ser.write(b'1')
            time.sleep(3)
            #write text file
            f.write(" tempReadingAfter: " + ser.readline() + "\n \n")
            state = False
            break
    count += 1


#closing file
f.close()

                
                
