#!/usr/bin/python3
import rospy
import time
from roboclaw_python.roboclaw_3 import Roboclaw
from math import pi
from roboclaw_driver.msg import armCmd
from configparser import ConfigParser

# config file maps address/channel to index in armCmd.msg
# number of joints is specified in armCmd.msg AND in joint_config.ini
parser = ConfigParser()
parser.read("joint_config.ini")
num_joints = int(parser['JOINTS']['num_joints'])
joint_addresses = parser['JOINTS']['addresses'].split(", ") # address of roboclaw for that joint
joint_channels = parser['JOINTS']['channels'].split(", ")
joint_comports = parser['JOINTS']['comports'].split(", ")
joint_cnts_per_rev = parser['JOINTS']['cnts_per_rev'].split(", ")

BAUDRATE = 115200

def callback(data):

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

    for i in range(1):
        # fetch data from config file
        address = int(joint_addresses[i]) # default: 0x80 or 128
        channel = int(joint_channels[i])
        cnts_per_rev = int(joint_cnts_per_rev[i])

        # fetch data from armCmd.msg
        rads = data.position_rads[i]
        speed = data.speed[i]
        # accel = data.accel_deccel[i]
        
        # Linux comport name
        rc = Roboclaw(joint_comports[i], BAUDRATE)
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
            cnts1 = int((rads*cnts_per_rev) / (2*pi))
        elif channel == 2:
            cnts2 = int((rads*cnts_per_rev) / (2*pi))
        else:
            rospy.loginfo(rospy.get_caller_id() + "Invalid motor channel: " + channel)
            return  
        rospy.loginfo(f'addr: {address} comport: {joint_comports[i]}')
        
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
    rospy.Subscriber('motors', armCmd, callback)
    print(f'Joint Config:\n  Addresses: {joint_addresses}\n  Channels: {joint_channels}\n  Comports: {joint_comports}\n  Counts per Revolution: {joint_cnts_per_rev}')
    rospy.loginfo('Listening...\n')
        

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
