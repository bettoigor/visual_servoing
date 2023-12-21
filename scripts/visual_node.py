#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script for image processing

Author: Adalberto Oliveira
Project: Solix AgBot
Version: 1.0
Date: 6-21-2022

"""

import rospy, time, sys, cv2, rostopic
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D, Twist
from utils.image_lib import *
from signal import signal, SIGINT

def callback_img(msg):

    global color_img

    # creating ros bridge
    bridge = CvBridge()

    # receive the ros image mesage and convert to bgr, ohta and hsv  
    color_img = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")   


def callback_depth(msg):

    global depth_img

    # creating ros bridge
    bridge = CvBridge()

    # receive the ros image mesage and convert to bgr, ohta and hsv  
    depth_img = bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")   
    

def exiting(signal_received, frame):

    # Publishers
    global cmd_vel


    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    cmd_vel.publish(Twist())
    
    exit(0)


def camera_main():

    # Global variables
    global color_img
    global depth_img
    global cmd_vel

    # Loading control parameters from rosparam
    RATE = rospy.get_param("image/rate")
    SHOW_IMG = rospy.get_param("image/show_img")
    ROI = rospy.get_param("image/roi")
    METHOD = rospy.get_param("image/method")
    

    # Initializing ros node
    rospy.init_node('visual_servoing_node', anonymous=True)    # node name

    # Subscribers
    h = rostopic.ROSTopicHz(-1)
    rospy.Subscriber('image_raw', Image, h.callback_hz, callback_args='image_raw')
    rospy.Subscriber('image_raw', Image, callback_img)
    
    # Publishers
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # send control signals
    processed_img = rospy.Publisher('processed_img',Image,queue_size=10)

    # Work objetcts
    img_process = ImageProcess(roi=ROI)
    pub_bridge = CvBridge()

 
    # control rate
    rate = rospy.Rate(RATE)
    time.sleep(1)

    # main loop
    quit = False

    while not rospy.is_shutdown() and not quit:

        pub_hz = h.get_hz('image_raw')

        if pub_hz is  not None:

            # Getting the control signal
            GAINS = rospy.get_param("control/gains")
            LIN_VEL = rospy.get_param("control/lin_vel")
            METHOD = rospy.get_param("image/method")

            control_signal,img_out = img_process.get_control_pd(color_img,
                                                       method=METHOD,
                                                       lin_vel=LIN_VEL,
                                                       gains=GAINS)
            image_message = pub_bridge.cv2_to_imgmsg(img_out, encoding="bgr8")


            # publishing messages
            cmd_vel.publish(control_signal)
            processed_img.publish(image_message)

            
            # showing images
            if SHOW_IMG:
                #cv2.imshow('Main - Color Image',color_img)
                cv2.imshow('Main - Processed Image',img_out)
                k = cv2.waitKey(5)
                if k == 27 or k == ord('q') or k == ord('Q'):
                    cmd_vel.publish(Twist())
                    quit = True

        else:
            print("No image received. Waiting image topic",end="\r") 
            cmd_vel.publish(Twist())
        
        signal(SIGINT, exiting)

        rate.sleep()
    
    cmd_vel.publish(Twist())

## Main

if __name__ == '__main__':
    camera_main()


