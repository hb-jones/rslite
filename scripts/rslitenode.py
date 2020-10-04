#!/usr/bin/env python
import rospy
import rslite
import rsconfig
import pyrealsense2 as rs
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image


def imagePub():
    #Initialise RS Camera
    config = rsconfig.depth_720_color_720_fps_30() #Must have colour stream
    cam = rslite.RSCam(config)
    cam.startStreaming()

    #Initialise ROS Publishers
    pub = rospy.Publisher('RSL/Image/Depth', String, queue_size=10)
    rospy.init_node('RSL/Camera', anonymous=True)
    rate = rospy.Rate(120) # 120Hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
    cam.stopStreaming()




def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        imagePub()
    except rospy.ROSInterruptException:
        pass
