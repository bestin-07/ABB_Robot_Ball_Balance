import json
import cv2
import imutils
from skimage.filters import threshold_local
from imutils.perspective import four_point_transform
import numpy
import numpy as np
import os
from os import listdir
from os.path import isfile, join
import time
import math
height = 800
width = 600
green = (0, 255, 0)
red =(255,0,0)
cap = cv2.VideoCapture('vedio/1.mp4')
prev_frame_time = 0
t=float(1)
new_frame_time = 25
i=0
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == False:
        break
    img_path=frame
    img=img_path.copy()
    org = img_path.copy()
    gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    blur_img = cv2.GaussianBlur(gray_img,(3,3),0)
    edged_img = cv2.Canny(blur_img,0,200)
    cnts,_ = cv2.findContours(edged_img.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts,key=cv2.contourArea,reverse=True)[:5]
    for c in cnts:
        peri = cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,0.02*peri,True)
        if len(approx)==4:
            doc = approx
            break
    p=[]
    for d in doc:
        tuple_point = tuple(d[0])
        cv2.circle(img,tuple_point,3,(0,0,255),4)
        p.append(tuple_point)
    warped = four_point_transform(org, doc.reshape(4, 2))
    img1=cv2.resize(warped, (600, 800))
    font = cv2.FONT_HERSHEY_SIMPLEX
    new_frame_time=time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    fps = int(fps)
    cv2.imshow('frame', img1 )
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    image=img1
    output=image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.blur(gray, (11, 11),10)
    circles = cv2.HoughCircles(gray_blurred,cv2.HOUGH_GRADIENT, 1, 50, param1 = 50,param2 = 30, minRadius = 50, maxRadius = 150)
    if circles is not None:
        circles =np.round(circles[0, :]).astype("int")
    circles[0][0]=circles[0][0]-300
    circles[0][1]=circles[0][1]-400
    print(circles)
    circles[0][0]=circles[0][0]+300
    circles[0][1]=circles[0][1]+400
    for (x, y, r) in circles:
        cv2.circle(output, (x, y), r, (0, 255, 0), 2)
    if i>0:
        t=float(1/fps)
        print(t)
        if(x<0):
            x=(x+300)
        elif(x>0):
            x=x-300

        if(y<0):
            y=y+400
        elif(y>0):
            y=y-400
        print(x,y) 
        p=math.atan(y/x)
        d= math.sqrt(x*x+y*y)
        dx= d*(math.cos(p))
        dy=d*(math.sin(p))
        s=d/(100*t)
        sx= s*(math.cos(p))
        sy=s*(math.sin(p))
        #a=float(d/800)
        #b=float(s/2)
        #q=float(a*b*45)
        qx=(dx/400)*(sx/2)*45
        qy=(dy/300)*(sy/2)*45

        print(d,s,dx,dy,sx,sy,qx,qy)
    i += 1
cap.release() 
cv2.waitKey(0)
cv2.destroyAllWindows() 