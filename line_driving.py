#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2 as cv
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import rospy
from time import sleep
from sensor_msgs.msg import Joy
from jetbotmini_msgs.msg import *
from jetbotmini_msgs.srv import *
from actionlib_msgs.msg import GoalID
from jetbotmini import Robot
import pyzbar.pyzbar as pyzbar

global blue_lower
global blue_upper
global mot_start

blue_lower = np.array([100,43,46], dtype=np.uint8)
blue_upper = np.array([124, 255, 255], dtype=np.uint8)
mot_start = False

def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class JoyTeleop:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.speed_value = 0.5
        self.turn_value = 0.2
        self.Joy_active = False
        self.cancel_time = time.time()
        self.pub_goal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        self.pub_JoyState = rospy.Publisher("/JoyState", JoyState, queue_size=10)
        self.robot = Robot()

    def cancel(self):
        self.pub_goal.unregister()
        self.pub_JoyState.unregister()
        
    def colorDisplay(self, image, font_path):
        
        global blue_lower
        global blue_upper

        image=cv.GaussianBlur(image,(5,5),0)     
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        hsv.astype(np.uint8)
        mask=cv.inRange(hsv,blue_lower,blue_upper)  
        mask=cv.erode(mask,None,iterations=2)
        mask=cv.dilate(mask,None,iterations=2)
        mask=cv.GaussianBlur(mask,(3,3),0)     
        cnts=cv.findContours(mask.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[-2]

        if len(cnts)>0:
            cnt = max (cnts,key=cv.contourArea)
            (color_x,color_y),color_radius=cv.minEnclosingCircle(cnt)
            if color_radius > 45:
                cv.circle(image,(int(color_x),int(color_y)),int(color_radius),(255,0,255),2)   
                center_x = image.shape[1] // 2
                if mot_start :
                    if center_x - color_x > 45 :
                        self.robot.left_motor.value = 0.2
                        self.robot.left_motor.value = 0.5
                    elif center_x - color_x < -45 :
                        self.robot.left_motor.value = 0.5
                        self.robot.left_motor.value = 0.2
                    else :
                        self.robot.left_motor.value = 0.5
                        self.robot.left_motor.value = 0.5
                else :
                    self.robot.left_motor.value = 0.0
                    self.robot.left_motor.value = 0.0
            else :
                self.robot.left_motor.value = 0.0
                self.robot.left_motor.value = 0.0
        else :
            self.robot.left_motor.value = 0.0
            self.robot.left_motor.value = 0.0

        return image, mask

    def decodeDisplay(self, image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cv.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 5)
            encoding = 'UTF-8'
            barcodeData = barcode.data.decode(encoding)
            barcodeType = barcode.type
            pilimg = Image.fromarray(image)
            draw = ImageDraw.Draw(pilimg)
            draw.text((x, y - 25), str(barcode.data, encoding), fill=(255, 0, 0))
            image = cv.cvtColor(np.array(pilimg), cv.COLOR_RGB2BGR)
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return image

    def cancel_nav(self):
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            self.Joy_active = not self.Joy_active
            self.pub_JoyState.publish(JoyState(self.Joy_active))
            self.pub_goal.publish(GoalID())
            self.cancel_time=now_time

if __name__ == '__main__':
    rospy.init_node('color_line')
    joy = JoyTeleop()
    font_path = "./font/Block_Simplified.TTF"
    capture = cv.VideoCapture(gstreamer_pipeline(flip_method=0), cv.CAP_GSTREAMER)
    cv_edition = cv.__version__
    if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    while capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        action = cv.waitKey(10) & 0xFF
        frame, binary = joy.colorDisplay(frame, font_path)
        frame = joy.decodeDisplay(frame)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        cv.imshow('frame', frame)
        if action == ord('q') or action == 113:
            break
        elif action == ord('s'):
            mot_start = not mot_start
    
    capture.release()
    cv.destroyAllWindows()

