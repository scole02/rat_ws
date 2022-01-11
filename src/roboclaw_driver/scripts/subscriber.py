#!/usr/bin/python3
import rospy
import time
from roboclaw_python.roboclaw_3 import Roboclaw

from std_msgs.msg import String, Int32MultiArray

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    #Linux comport name
    rc = Roboclaw("/dev/ttyACM0",115200)

    rc.Open()
    address = 0x80
    accel = 0
    speed = 0
    deccel = 0
    
    rc.SpeedAccelDeccelPositionM1(address,accel,speed,deccel,data.data[2],1)

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
    
        

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
