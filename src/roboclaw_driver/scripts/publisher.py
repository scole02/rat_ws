#!/usr/bin/python3
# license removed for brevity
import rospy
from std_msgs.msg import String, Int32MultiArray

def talker():
    pub = rospy.Publisher('motors', Int32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    address = int(input("enter address in base 10 (default: 128): "))
    channel = int(input("enter motor channel: "))
    cnts_per_rev = int(input("enter encoder counts per revolution: "))
    rads_precision = 10 ** int(input("enter floating point position precision (0-9): "))  
    while not rospy.is_shutdown():
        rads = int(float(input("enter position in radians: ")) * rads_precision)
        if rads < 0 or rads/rads_precision > 6.29:
            rospy.loginfo("please enter a valid position in radians (0 to 6.283184...)") 
            continue
        arr_msg = Int32MultiArray()
        arr_msg.data = [address, channel, cnts_per_rev, rads, rads_precision]
        pub.publish(arr_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
