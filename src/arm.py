#!/usr/bin/env python3
import rospy
import sys, select, tty, termios
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import random
import cv2, cv_bridge
from sensor_msgs.msg import*
from gazebo_msgs.srv import*
from scara_arm.srv import fk
from scara_arm.srv import ik

a1 = 0.425 #length of link1
a2 = 0.345 #length of link2
th1 = th2 = 0 #angles initiallized
box_x = 0 #box initial x coordinate
box_y = 0.5 #box initial y coordinate
v = 1


#image callback function gets the image from camera and calculates the coordinates of the box
def image_callback(msg):
    global box_x,box_y
    bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([ 29, 86, 6])
    upper_yellow = np.array([64, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    M= cv2.moments(mask)

    if M['m00'] > 0:
        x,y,w,h= cv2.boundingRect(mask)
        xcoordinate1= x 
        xcoordinate2= x + w
        xcoordinate_center= int(M['m10']/M['m00'])
        ycoordinate1= y 
        ycoordinate2= y + h
        ycoordinate_center= int(M['m01']/M['m00'])
        if ycoordinate_center < 400:
            box_x = -0.003205128 * (ycoordinate_center - 400)
        if ycoordinate_center > 400:
            box_x = -0.003205128 * (400- ycoordinate_center)
        if xcoordinate_center < 400:
            box_y = 0.003205128 * (400- xcoordinate_center)
        if xcoordinate_center > 400:
            box_y = -0.003205128 * (xcoordinate_center - 400 )
        #print(box_x,box_y)
    cv2.imshow("window", image)
    cv2.waitKey(3)

def spawn_box(a,b):
    state_msg = ModelState()
    state_msg.model_name = 'box'
    state_msg.pose.position.x = a
    state_msg.pose.position.y = b
    state_msg.pose.position.z = 0.05
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 1
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)

p = 0
def random_spawn():
    global p
    rospy.sleep(4)
    print("spawning box at random position")
    if p==0:
        spawn_box(0.25,0.35)
        p = p+1
    elif p==1:
        spawn_box(-0.33,0.37)
        p = p+1
    elif p==2:
        spawn_box(-0.5,-0.2)
        p = p+1
    elif p==3:
        spawn_box(0.45,-0.25)
        p = 0
    rospy.sleep(1)

def carry_box():
    grip = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    arm = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    state = arm("link3","")
    state_msg = ModelState()
    state_msg.model_name = 'box'
    state_msg.pose.position.x = state.link_state.pose.position.x
    state_msg.pose.position.y = state.link_state.pose.position.y
    state_msg.pose.position.z = state.link_state.pose.position.z
    grip_state = grip(state_msg)

def check_reach(x,y):
    global v
    length = a1 + a2
    distance = np.sqrt(np.square(x) + np.square(y))
    if distance<length:
        v = 1
        #print("valid")
    else:
        v = 0
        #print("point is not reachable")
    return v

def forward_kinematics(a,b,c):
    joint1_rotate = JointTrajectory()
    joint1_point = JointTrajectoryPoint()
    joint1_rotate.header.stamp = rospy.Time.now()
    joint1_rotate.joint_names = ['joint1']
    joint1_point.positions = [a]
    joint1_point.time_from_start = rospy.Duration(1)
    joint1_rotate.points.append(joint1_point)
    #print("joint1 : %s"%joint1_point.positions)
    joint1_pub.publish(joint1_rotate)

    joint2_rotate = JointTrajectory()
    joint2_point = JointTrajectoryPoint()
    joint2_rotate.header.stamp = rospy.Time.now()
    joint2_rotate.joint_names = ['joint2']
    joint2_point.positions = [b]
    joint2_point.time_from_start = rospy.Duration(1)
    joint2_rotate.points.append(joint2_point)
    #print("joint2 : %s"%joint2_point.positions)
    joint2_pub.publish(joint2_rotate)

    joint3_rotate = JointTrajectory()
    joint3_point = JointTrajectoryPoint()
    joint3_rotate.header.stamp = rospy.Time.now()
    joint3_rotate.joint_names = ['joint3']
    joint3_point.positions = [c]
    joint3_point.time_from_start = rospy.Duration(1)
    joint3_rotate.points.append(joint3_point)
    #print("joint3 : %s"%joint3_point.positions)
    joint3_pub.publish(joint3_rotate)
    #print ("--------------")

def inverse_kinematics(x,y):
    global th1, th2
    th2 = np.arccos((np.square(x) + np.square(y) - np.square(a1) - np.square(a2)) / (2*a1*a2))
    th1 = np.arctan2(y,x) - np.arctan2( (a2*np.sin(th2)) , (a1 + a2*np.cos(th2)) )
    #print(th1,th2)
    forward_kinematics(th1,th2,0.2)

fk_mode = 0
def fk_callback(req):
    global j_1,j_2, fk_mode
    j_1 = req.j1
    j_2 = req.j2
    fk_mode = 1
    return True

ik_mode = 0
def ik_callback(req):
    global ik_x,ik_y, ik_mode
    ik_x = req.x
    ik_y = req.y
    ik_mode = 1
    return True

def get_the_box():
    print("going to pick the box")
    if check_reach(box_x, box_y) == 1:
        for x in range(50):
            inverse_kinematics(box_x,box_y)
            rate.sleep()
        for x in range(5):
            carry_box()
            rate.sleep()

        #print("done")
    else:
        print("box is not reachable")

def mode():
    global fk_mode,ik_mode
    print("waiting for commands")
    while 1:
        rate.sleep()
        if fk_mode == 1:
            print("forward_kinematics recieved")
            for x in range(50):
                forward_kinematics(j_1,j_2,0.3)
                carry_box()
                rate.sleep()
            fk_mode = 0
            break
        if ik_mode == 1:
            print("inverse_kinematics recieved")
            if check_reach(ik_x, ik_y) == 1:
                for x in range(50):
                    inverse_kinematics(ik_x,ik_y)
                    carry_box()
                    rate.sleep()
            else:
                print("not reachable point")
            ik_mode = 0
            break
def main():
    get_the_box()
    mode()
    print("-----------")
    random_spawn()


rospy.init_node('SCARA_Arm')
rospy.Service('Scara_FK', fk, fk_callback)
rospy.Service('Scara_IK', ik, ik_callback)
joint1_pub = rospy.Publisher('/joint1_controller/command', JointTrajectory, queue_size=10)
joint2_pub = rospy.Publisher('/joint2_controller/command', JointTrajectory, queue_size=10)
joint3_pub = rospy.Publisher('/joint3_controller/command', JointTrajectory, queue_size=10)
image_sub = rospy.Subscriber('/camera/image_raw', Image, image_callback)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    main()
    rate.sleep()

