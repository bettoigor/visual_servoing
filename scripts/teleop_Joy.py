#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Joypad control node for SoyBot            #
#                                           #
# Author: Adalberto Oliveira                #
# Mastering in robotic - PUC-Rio            #   
# Version: 1.0                              #
# Date: 2-13-2019                           #
#                                           #
#############################################

import rospy, time, angles, math, tf2_geometry_msgs, tf, sys, cv2
import tf2_ros as tf2
import numpy as np
from operator import itemgetter, attrgetter, methodcaller
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy,Image, CameraInfo
from std_msgs.msg import Bool, Float64
from cv_bridge import CvBridge


def callback_joy(msg):

    global joyCommand
    global vel
    global steer
    global start
    global change_state
    global auto
    global cmd_vel 
    global max_vel
    global max_steer
    global pub

    joyCommand = msg
    cmdVel = Twist()


    '''
    vel -= msg.axes[4]
    if abs(vel) >= 25: vel = np.sign(vel)*25

    vel += (10* msg.axes[5])
    '''
    

    if msg.buttons[0]:
        start = True

    else:
        start = False


    vel = msg.axes[1] * max_vel  
    steer = msg.axes[2] * max_steer


    if msg.buttons[10]:
        pub = not pub
        if pub:
            print('Sending command.')
        else:
            print('Command do not sent.')



    if msg.buttons[1]:
        vel = 0
        steer = 0
        start = False 
        cmdVel.linear.x = 0  
        cmdVel.angular.z = 0
        cmd_vel.publish(cmdVel)
        print ('Stopping!')   


    if msg.buttons[5]:
        vel = 0
        steer = 0
        cmdVel.linear.x = 0  
        cmdVel.angular.z = 0
        cmd_vel.publish(cmdVel)
        print ('Stopping!')   

    if msg.buttons[8]:
        auto = True
        vel = 0
        steer = 0
        cmdVel.linear.x = 0  
        cmdVel.angular.z = 0
        cmd_vel.publish(cmdVel)   
        print ('Stopping!')   



#Callback da camera top
def image_callback_top(data):
        global imagem_top, bridge
        imagem_top = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")

#Callback da camera bottom0
def image_callback_bottom0(data):
        global imagem_bottom0, bridge
        imagem_bottom0 = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")

#Callback da camera bottom1
def image_callbackb_bottom1(data):
        global imagem_bottom1, bridge
        imagem_bottom1 = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")

#Callback da camera side0
def image_callback_side0(data):
        global imagem_side0, bridge
        imagem_side0 = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")

#Callback da camera side1
def image_callback_side1(data):
        global imagem_side1, bridge
        imagem_side1 = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")    

def state_callback(msg):
    global state

    state = msg.activeState

def save_image():
    global img_path

    time = rospy.Time.now()

    cv2.imwrite(img_path+'top/image_cam_{}_{}.png'.format('top', time), imagem_top)
    cv2.imwrite(img_path+'bottom_right/image_cam_{}_{}.png'.format('bottom_right',time), imagem_bottom0)
    cv2.imwrite(img_path+'bottom_left/image_cam_{}_{}.png'.format('bottom_left',time), imagem_bottom1)
    cv2.imwrite(img_path+'side_right/image_cam_{}_{}.png'.format('side_right',time), imagem_side0)
    cv2.imwrite(img_path+'side_left/image_cam_{}_{}.png'.format('side_left',time), imagem_side1)


def send_cmd():
    global vel
    global steer
    global start
    global cmd_vel 
    global state
    global pub

    cmdVel = Twist()

    if start:
        cmdVel.linear.x = vel
        cmdVel.angular.z = steer

    if pub:
        cmd_vel.publish(cmdVel)
    
 


def robot_joy():

    global cmd_vel 
    global tfBuffer  
    global cv_image
    global change_state

    rospy.init_node('soybot_joy', anonymous=True) #nome do nó
    
    # Publishers
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Subscribers
    rospy.Subscriber('joy', Joy, callback_joy)  

    rate = rospy.Rate(30) 
    time.sleep(2)

    
    print ('Running...')

    while not rospy.is_shutdown():
        send_cmd()
        rate.sleep()


# Main
# GLOBAL VARIABLES
img_path = 'local' 
max_vel = float(sys.argv[1])
max_steer = float(sys.argv[2])
joyCommand = Joy()
bridge = CvBridge()
imagem_top = np.array([[0],[0],[0]])
imagem_bottom0 = np.array([[0],[0],[0]])
imagem_bottom1 = np.array([[0],[0],[0]])
imagem_side0 = np.array([[0],[0],[0]])
imagem_side1 = np.array([[0],[0],[0]])
vel = 0
steer = 0
start = False
auto = False
state = False
pub = False
if __name__ == '__main__':
    robot_joy()
    '''
    try:
        robot_joy()
    except:
        cv2.destroyAllWindows()         
        print 'Node ended.'
    '''

# End Main