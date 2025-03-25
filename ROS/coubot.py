#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import math
import rospy
import smbus
import threading
import ros_numpy
import numpy as np
import RPi.GPIO as GPIO
import pyzbar.pyzbar as pyzbar
from time import sleep
from jetbotmini import Robot
from jetbotmini_msgs.msg import *
from jetbotmini_msgs.srv import *
from sensor_msgs.msg import Image
from std_msgs.msg import String

print("Starting now!")

STOP = 0
DRIVE = 1
LEFT = 2
RIGHT = 3
Led_B_pin = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(Led_B_pin, GPIO.OUT, initial=GPIO.HIGH)

bus = smbus.SMBus(1)
ADDRESS = 0X1B


class Coubot:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.robot = Robot()

        self.state = STOP
        self.frame = None
        self.qrData = None

        self.img_subscriber = rospy.Subscriber("/image", Image, self.line_driving)
        self.auto = rospy.Subscriber("/start_auto", String, self.auto_cb)
        self.srv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.buzzer) # catkin_ws/src/jetbotmini_msgs/srv/Buzzer.srv
        self.srv_LEDBLUE = rospy.Service("/LEDBLUE", LEDBLUE, self.LEDBLUEcallback)
        self.srv_speed = rospy.Service("/set_speed", SetSpeed, self.handle_set_speed)
        self.srv_dir = rospy.Service("/key_Controll", KeyControll, self.move_bindings)
        self.qr_publisher = rospy.Publisher("/data", String, queue_size=10)
        self.volPublisher = rospy.Publisher("/voltage", Battery, queue_size=10)

        t1 = threading.Thread(target=self.decode)
        t1.daemon = True
        t1.start()

    def cancel(self):
        self.img_subscriber.unregister()
        self.srv_LEDBLUE.shutdown()
        self.volPublisher.unregister()
        GPIO.cleanup()
        rospy.loginfo("Close the robot...")
        rospy.sleep(1)

    def auto_cb(self, msg):
        rospy.loginfo("Received start signal: " + msg.data)
        if msg.data == "on":
            rospy.loginfo("Starting line driving")
            self.state = DRIVE
        elif msg.data == "off":
            rospy.loginfo("Stopping line driving")
            self.state = STOP
            self.stop()
    
    def setState(self, newState):
        self.state = newState

    def battery(self):
        while not rospy.is_shutdown():
            sleep(30)
            AD_value = bus.read_i2c_block_data(ADDRESS,0x00,2)
            # AD_value = [high_byte, low_byte]
            # AD_value[0] << 8 : 상위 바이트를 왼쪽으로 8비트 시프트(곱하기 256)
            # ex) 0x03 << 8 = 0x0300 = 768
            voltage = ((AD_value[0] << 8) + AD_value[1]) * 13.3 / 1023.0
            voltage = math.floor((voltage * 100) / 13)
            battery = Battery()
            battery.Voltage = voltage
            if not rospy.is_shutdown():
                self.volPublisher.publish(battery)
    
    def LEDBLUEcallback(self, request):
        if not isinstance(request, LEDBLUERequest): return
        if request.ledblue == 1:
            for _ in range(10):
                GPIO.output(Led_B_pin, GPIO.HIGH) # LED Off
                sleep(0.5)
                GPIO.output(Led_B_pin, GPIO.LOW) # LED On
                sleep(0.5)
        elif request.ledblue == 0:
            GPIO.output(Led_B_pin, GPIO.HIGH) # LED Off
        response = LEDBLUEResponse()
        response.result = True
        return response

    def buzzer(self, request):
        # i2c 방식으로 버저 신호 전송
        bus.write_byte_data(ADDRESS,0x06,request.buzzer)
        response = BuzzerResponse()
        response.result = True
        return response

    def goForward(self):
        self.robot.left_motor.value = 0.55
        self.robot.right_motor.value = 0.55
    
    def stop(self):
        self.robot.left_motor.value = 0
        self.robot.right_motor.value = 0
    
    def goLeft(self):
        self.robot.left_motor.value = 0.4
        self.robot.right_motor.value = 0.55

    def goRight(self):
        self.robot.left_motor.value = 0.55
        self.robot.right_motor.value = 0.4

    def decode(self):
        while True:
            if self.frame is not None:
                gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                qrcodes = pyzbar.decode(gray)
                for qrcode in qrcodes:
                    encoding = "UTF-8"
                    self.qrData = qrcode.data.decode(encoding)
                    print("QRCODE DATA : {}".format(self.qrData))
                    if not rospy.is_shutdown():
                        self.qr_publisher.publish(self.qrData)
                    sleep(0.1)
                    if self.qrData == "right":
                        self.setState(STOP)
                        rospy.Timer(rospy.Duration(2), lambda _ : self.setState(DRIVE))
                    elif self.qrData == "left":
                        self.setState(STOP)
                        rospy.Timer(rospy.Duration(2), lambda _ : self.setState(DRIVE))

    def handle_set_speed(self, req):
        leftspeed = max(0, min(255, req.left_speed))
        rightspeed = max(0, min(255, req.right_speed))
        leftdir = 1 if leftspeed > 0 else 0
        rightdir = 1 if rightspeed > 0 else 0

        bus.write_i2c_block_data(ADDRESS, 0x01, [leftdir, leftspeed, rightdir, rightspeed])

        rospy.loginfo(f"Set Speed: Left={leftspeed}, Right={rightspeed}")
        return SetSpeedResponse(success=True, message="Speed updated successfully")
    
    def move_bindings(self, req):
        moveBindings = {
            "w" : [1, 100, 1, 100],
            "s" : [0, 100, 0, 100],
            "a" : [0, 100, 1, 100],
            "d" : [1, 100, 0, 100],
            "q" : [1, 0, 1, 0]
        }
        bus.write_i2c_block_data(ADDRESS, 0x01, [moveBindings[req.keyval]])
        return KeyControllResponse(success=True, message="manual driving good")

    def line_driving(self, msg):
            
        blue_lower = np.array([100,43,46], dtype=np.uint8)
        blue_upper = np.array([124,255,255], dtype=np.uint8)

        self.frame = ros_numpy.numpify(msg)
        frame=cv2.GaussianBlur(self.frame,(5,5),0)     
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv.astype(np.uint8)
        mask=cv2.inRange(hsv,blue_lower,blue_upper)  
        mask=cv2.erode(mask,None,iterations=2)
        mask=cv2.dilate(mask,None,iterations=2)
        mask=cv2.GaussianBlur(mask,(3,3),0)     
        cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(cnts)>0:
            cnt = max (cnts,key=cv2.contourArea)
            (color_x,color_y),color_radius=cv2.minEnclosingCircle(cnt)
            if color_radius > 45:
                cv2.circle(frame,(int(color_x),int(color_y)),int(color_radius),(255,0,255),2)   
                center_x = frame.shape[1] // 2
                if self.state != STOP:
                    if center_x - color_x > 30 :
                        self.setState(LEFT)
                    elif center_x - color_x < -30 :
                        self.setState(RIGHT)
                    else:
                        self.setState(DRIVE)
                
                if self.state == DRIVE:
                    self.goForward()
                elif self.state == LEFT:
                    self.goLeft()
                elif self.state == RIGHT:
                    self.goRight()
                elif self.state == STOP:
                    self.stop()
        else :
            self.stop()


        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == ord('z'):
            rospy.signal_shutdown("Exit")
        elif key == ord('k'):
            if self.state == DRIVE : self.setState(STOP)
            elif self.state == STOP : self.setState(DRIVE)
        
        return True

if __name__ == '__main__':
    rospy.init_node("video_check", anonymous=False)
    try:
        cb = Coubot()
        cb.battery()
        rospy.spin()
    except Exception as e:
        rospy.loginfo("Error: ", e)