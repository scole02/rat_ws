#!/usr/bin/python3
import rospy
import time
from roboclaw_python.roboclaw_3 import Roboclaw
from math import pi

from std_msgs.msg import String, Int32MultiArray

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    # fetch data from msg
    address = data.data[0] # default: 0x80 or 128
    channel = data.data[1]
    cnts_per_rev = data.data[2]
    rads = data.data[3]
    rads_precision = data.data[4]
    #Linux comport name
    rc = Roboclaw("/dev/ttyACM0",115200)

    rc.Open()
    buf = 1
    accel1 = 0
    speed1 = 0
    deccel1 = 0
    cnts1 = 0
    accel2 = 0
    speed2 = 0
    deccel2 = 0
    cnts2 = 0
    
    if channel == 1:
        cnts1 = int((rads*cnts_per_rev) / (2*pi*rads_precision))
    elif channel == 2:
        cnts2 = int((rads*cnts_per_rev) / (2*pi*rads_precision))
    else:
        rospy.loginfo(rospy.get_caller_id() + "Invalid motor channel: " + channel)
        return  
    rospy.loginfo(f'cnts1: {cnts1}\n cnts2: {cnts2}')
    rc.SpeedAccelDeccelPositionM1M2(address,accel1,speed1,deccel1,cnts1,
                                                 accel2,speed2,deccel2,cnts2,buf)
    #rc.ForwardM1(address,32)#1/4 power forward
    #time.sleep(3)
    #rc.ForwardM1(address,0)	#1/4 power forward

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('motors', Int32MultiArray, callback)
    rospy.loginfo('Listening...\n')
        

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
