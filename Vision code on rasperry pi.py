import cv2
import numpy as np
from cv2 import aruco
import math
import serial
import time
import gui
#---------Predefined rooms list----------
rooms = [1,2,3,4,5,6,7,8,9,10,11,12]
dist = 700.0
#----------------------------------------
def recuart():
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
    return line
#-----------------------------------------
def caparuco ():
    ret, frame = capture.read()  # reads frame from the camera stream and saves it into the variable "Capturing"
    corners, ids= arucodet(frame)   #takes the corners co-ordinates & ids returned from read arucos
    global dist
    if corners == []:           #if no markers detected
        return
    else:
        dist = calcdist(corners[0][0][0], corners[0][0][1]) #calculating the distance between shepherd & the marker
        #return the aruco id when the distance is less than 120
        if ( dist <= 120.0 ):
            return ids[0][0]
#---------------------------------------
def arucodet (img):
    global dist
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
    cv2.putText(frame_markers,"distance: ",(0,50),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,255,0),1)
    cv2.putText(frame_markers,str(int(dist)),(200,50),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,255,0),1)
    cv2.imshow('Aruco', frame_markers)
    return corners, ids
#--------------------------------
def calcdist(p1, p2):
    #uses the triangluar similarity to calculate the distance between
    # shepherds & the marker from the marker's side length
    x = p1[0] - p2[0]
    y = p1[1] - p2[1]
    length = math.sqrt(x**2+y**2)
    dist = 20 * 220 / length
    return dist
#----------------------------------
if __name__ == '__main__':
#-------------Getting the rooms using GUI--------------------
    gui.GUI()
    dest1 = gui.inputValue
    dest2 = gui.inputValue2
    dest3 = gui.inputValue3
    home = 10                   #The home it returns to after finishing
    dest = [dest1,dest2,dest3,home]
#-------------Initialising the serial communication---------------
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()
    time.sleep(1)
    i=0             #counter of the rooms reached succesfully
    ser.flush()
    #Sending "s" to mega to start runnig the program
    ser.write("s\n".encode('utf-8'))
    capture = cv2.VideoCapture(0)
    while True:         # Our loop for video streaming
        ind=caparuco()      #scanning for arucos
        if ind != None :
            #Checking if the room reached is the one wanted by the operator
            if rooms[int(ind)-1] == dest[i]:
                ser.flush()
                # Sending "a" to mega to start the delivery procedure
                ser.write("a\n".encode('utf-8'))
                i=i+1
                cv2.destroyAllWindows()
                #waiting for the mega to return "r" meaning that the user received using RFID card
                while(str(recuart()) != 'r'):
                    print("waiting")
        #if all the rooms are served break the cycle
        if i == 4:
            break
    # Check if the user has pressed Esc key, this is to break from the while true loop
        c = cv2.waitKey(1)
        if c == 27:
            break
# Close the capturing device, releases the capture object as a resource to be used by other apps/scripts
    capture.release()
#Sending "t" to mega to terminate the running program
    ser.write("t\n".encode('utf-8'))
# Close all windows
    cv2.destroyAllWindows()