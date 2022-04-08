#!/usr/bin/python3
import rospy
import time
from roboclaw_python.roboclaw_3 import Roboclaw
from math import pi
from roboclaw_driver.msg import armCmd, ratTelemetry
from configparser import ConfigParser

class RoboclawNode:

    # config file maps address/channel to index in armCmd.msg
    # number of joints is specified in armCmd.msg AND in joint_config.ini
    parser = ConfigParser()
    parser.read("joint_config.ini")
    num_joints = int(parser['JOINTS']['num_joints'])
    joint_addresses= parser['JOINTS']['addresses'].split(", ") # address of roboclaw for that joint
    joint_channels = parser['JOINTS']['channels'].split(", ")
    joint_comports = parser['JOINTS']['comports'].split(", ")
    joint_cnts_per_rev = parser['JOINTS']['cnts_per_rev'].split(", ")
    
    BAUDRATE = 115200
    
    def __init__(self):
        
        self.rc = Roboclaw(self.joint_comports[0], self.BAUDRATE) # comports are not gonna change for us
        self.rc.Open()

    def callback(self, data):

        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        # init message to publish for hardware interface telem_callback
        telem_msg = ratTelemetry()

        for i in range(self.num_joints):
            # fetch data from config file
            address = int(self.joint_addresses[i]) # default: 0x80 or 128
            channel = int(self.joint_channels[i])
            cnts_per_rev = int(self.joint_cnts_per_rev[i])

            # fetch data from armCmd.msg
            rads = data.position_rads[i]
            speed = data.speed[i]
            # accel = data.accel_deccel[i]

            # Linux comport name
            #rc.Open()
            

            buf = 1
            accel1 = 0
            speed1 = 0
            deccel1 = 0
            cnts1 = 0
            accel2 = 0
            speed2 = 0
            deccel2 = 0
            cnts2 = 0

## Doing this seperate based on the channels seems kind of dumb, but roboclaw lib
### uses different funcs for m1 and m2, there is a func for both m1 and m2 at the same time
### so that could replace this later

            if channel == 1:
                # get current encoder val
                print(f'cur_address: {address}')
                cur_enc_val = self.rc.ReadEncM1(address)[1] # ReadEnc returns (success_code, enc_val, address)
                print(cur_enc_val)
                cur_angle = (cur_enc_val/cnts_per_rev) * 2 * pi
                print(cur_angle)
                # populate telemetry message
                telem_msg.angle[i] = cur_angle
                # calculate encoder counts to write to motor
                cnts1 = int((rads*cnts_per_rev) / (2*pi))
                rospy.loginfo(f'addr: {address}, cnts1: {cnts1}')
                self.rc.SpeedAccelDeccelPositionM1(address,accel1,speed1,deccel1,cnts1,buf)
            elif channel == 2:
                # get current encoder val
                cur_enc_val = self.rc.ReadEncM2(address)[1]
                cur_angle = (cur_enc_val/cnts_per_rev) * 2 * pi
                # populate telemetry message
                telem_msg.angle[i] = cur_angle
                # calculate encoder counts to write to motor
                cnts2 = int((rads*cnts_per_rev) / (2*pi))
                rospy.loginfo(f'cnts2: {cnts2}')
                self.rc.SpeedAccelDeccelPositionM2(address,accel2,speed2,deccel2,cnts2,buf)
            else:
                rospy.loginfo(rospy.get_caller_id() + "Invalid motor channel: " + channel)
                return  
            time.sleep(0.5)
            
        rospy.loginfo(f'telem_msg: {telem_msg}')

        self.telem_pub.publish(telem_msg)
        
        #self.rc.SpeedAccelDeccelPositionM1M2(address,accel1,speed1,deccel1,cnts1,
            #                                         accel2,speed2,deccel2,cnts2,buf)
        # read encoder values
        #for i in range(self.num_joints):
        #    address = int(self.joint_addresses[i]) # default: 0x80 or 128
        #    channel = int(self.joint_channels[i])
        #    if channel == 1:
        #        enc_val = rc.ReadEncM1(address)


        #rc.ForwardM1(address,32)#1/4 power forward
        #time.sleep(3)
        #rc.ForwardM1(address,0)	#1/4 power forward

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'telemetry' node so that multiple listeners can
        # run simultaneously.
        self.telem_pub = rospy.Publisher('roboclaw_telemetry', ratTelemetry, queue_size=5)
        rospy.init_node('roboclaw_node', anonymous=True)
        rospy.Subscriber('roboclaw_cmd', armCmd, self.callback)
        print(f'Num Joints: {self.num_joints}\n Joint Config:\n  Addresses: {self.joint_addresses}\n  Channels: {self.joint_channels}\n  Comports: {self.joint_comports}\n  Counts per Revolution: {self.joint_cnts_per_rev}')
        rospy.loginfo('Listening...\n')


        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    roboclaw_node = RoboclawNode()
    roboclaw_node.listener()
