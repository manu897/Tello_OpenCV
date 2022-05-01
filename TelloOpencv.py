from utlis import *
import cv2
import os
import cv2.aruco as aruco
import numpy as np

w, h = 480, 360
pid = [0.5, 0.5, 0]  # kp, ki, kd
pError = 0
startCounter = 1 # for no Flight 1 and Flight 0


myDrone = initializeTello()

while True:
    ## Flight
    if startCounter == 0:
        myDrone.takeoff()
        startCounter = 1

    imgAug = cv2.imread("logo.jpg")
    img = telloGetFrame(myDrone, w, h)
    findMarker(img)
    cv2.imshow("Image", img)
    cv2.waitKey(1)
    arucoFound = findMarker(img)


    # looping for Aruco markers
    if len(arucoFound[0])!=0:
        for bbox, id in zip(arucoFound[0], arucoFound[1]):
            img2, info, r = augmentAruco(bbox, id, img, imgAug)
        # print(info, r)
            pError = trackAruco(myDrone, info, w, pid, pError)
            #cv2.cirlce(img2, info, 5, (0, 255, 0), cv2.FILLED)
         # cv2.imshow("Image3", img2)
         # cv2.waitKey(1)


