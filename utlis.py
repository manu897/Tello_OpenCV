from djitellopy import Tello
import cv2
import numpy as np
import cv2.aruco as aruco
import os


def initializeTello():
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0
    myDrone.left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0
    print(myDrone.get_battery())
    #myDrone.streamoff()
    myDrone.streamon()
    return myDrone

def telloGetFrame(myDrone, w = 480, h = 360):
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame, (w,h))
    return img

def findMarker(img, markerSize=6, totalMarkers= 250, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

    # print(ids) #to print aruco marker id

    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    return [bboxs, ids]


def augmentAruco(bbox, id, img, imgAug, drawID=True):

    tl = bbox[0][0][0], bbox[0][0][1]
    tr = bbox[0][1][0], bbox[0][1][1]
    br = bbox[0][2][0], bbox[0][2][1]
    bl = bbox[0][3][0], bbox[0][3][1]

    h, w, c = imgAug.shape

    pts1 = np.array([tl, tr, br, bl])
    pts2 = np.float32([[0,0],[w,0],[w,h],[0,h]])
    matrix, _ = cv2.findHomography(pts2, pts1)
    imgOut = cv2.warpPerspective(imgAug, matrix, (img.shape[1], img.shape[0]))
    cv2.fillConvexPoly(img, pts1.astype(int), (0, 0, 0))
    c, r = cv2.minEnclosingCircle(pts1.astype(int)) # getting center and radius of the four points

    imgOut = img + imgOut

    #cv2.imshow('Image2', imgOut)
    return imgOut, c, r

def trackAruco(myDrone, info, w, pid, pError):

    # PID controller

    error = int(info[0]) - w//2
    speed = pid[0]*error + pid[1]*(error - pError)
    speed = int(np.clip(speed, -100, 100))
    print(speed)

    if int(info[0])!=0:
        myDrone.yaw_velocity = speed
    else:
        myDrone.for_back_velocity = 0
        myDrone.left_right_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        error = 0
    if myDrone.send_rc_control:
        myDrone.send_rc_control(myDrone.left_right_velocity, myDrone.for_back_velocity, myDrone.up_down_velocity, myDrone.yaw_velocity)

    return error