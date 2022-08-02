from ast import While
import errno
import json
import egm_pb2
import cv2
import socket
import select

from skimage.filters import threshold_local
from imutils.perspective import four_point_transform

import numpy as np

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


#ADDING EGM CLASS

##########################################################################################################################

class EGM(object):
    
    def __init__(self, port=6511):

        self.socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('',port))
        self.send_sequence_number=0
        self.egm_addr=None
        self.count=0

    def receive_from_robot(self, timeout=0):

        s=self.socket
        s_list=[s]
        try:
            res=select.select(s_list, [], s_list, timeout)
        except select.error as err:
            if err.args[0] == errno.EINTR:
                return False, None
            else:
                raise

        if len(res[0]) == 0 and len(res[2])==0:
            return False, None
        try:
            (buf, addr)=s.recvfrom(65536)
        except:
            self.egm_addr=None
            return False, None

        self.egm_addr=addr

        robot_message=egm_pb2.EgmRobot()
        robot_message.ParseFromString(buf)

        Robot_pos = None
        joint_angles=None
        rapid_running=False
        motors_on=False
        Robot_Position_Flag = False

        if robot_message.HasField('feedBack'):
            if Robot_Position_Flag == False:
                Robot_pos = robot_message.feedBack.cartesian.pos
                Robot_Position_Flag = True
            Joints = robot_message.feedBack.joints.joints
            print(Joints)
        if robot_message.HasField('rapidExecState'):
            rapid_running = robot_message.rapidExecState.state == robot_message.rapidExecState.RAPID_RUNNING
        if robot_message.HasField('motorState'):
            motors_on = robot_message.motorState.state == robot_message.motorState.MOTORS_ON

        return True,Joints

    def send_to_robot_joints(self, joint_angles):

        if not self.egm_addr:
            return False

        self.send_sequence_number+=1

        sensorMessage=egm_pb2.EgmSensor()

        header=sensorMessage.header
        header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno=self.send_sequence_number
        self.send_sequence_number+=1

        planned=sensorMessage.planned

        if joint_angles is not None:
            #joint_angles2 = list(np.rad2deg(joint_angles))
            #print(joint_angles2)
            planned.joints.joints[:] = [0.0,0.0,0.0,180,0.0,0.0]
            print(planned.joints.joints)
        buf2=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf2, self.egm_addr)
        except:
            return False

        return True

    def send_to_robot_cartesian(self, Position, Rob_pos):

        if not self.egm_addr:
            return False

        self.send_sequence_number+=1

        sensorMessage=egm_pb2.EgmSensor()

        header=sensorMessage.header
        header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno=self.send_sequence_number
        self.send_sequence_number+=1

        planned=sensorMessage.planned

        if Position is not None:
            planned.cartesian.pos.x = Rob_pos.x + Position[2]
            planned.cartesian.pos.y = Rob_pos.y + Position[0]
            planned.cartesian.pos.z = Rob_pos.z + Position[1]
            planned.cartesian.euler.x = -90
            planned.cartesian.euler.y = 180
            planned.cartesian.euler.z = 0
            print(planned.cartesian.pos)
        buf2=sensorMessage.SerializeToString()

        try:
            self.socket.sendto(buf2, self.egm_addr)
        except:
            return False

        return True
##############################################################################################


def Vedio_Process(rob_joints):
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



EGM1 = EGM()


while True:
    Current_Position = EGM1.receive_from_robot()[1]
    if Current_Position is not None:
        while True:
            #Current_Position = EGM1.receive_from_robot()[1]
            #print(Current_Position)
            EGM1.send_to_robot_joints(Current_Position)
            time.sleep(.1)
    else:
        print("Waiting for robot to connect")