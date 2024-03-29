#!/usr/bin/env python
import rospy
from rslite import rsconfig
from rslite import rslite
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import time
import math
import numpy as np
from cv_bridge import CvBridge
import pyrealsense2 as rs

import tf
import tf2_ros
import geometry_msgs.msg
import pcl_msgs
import pcl_ros


def imagePub(pc = False):
    #Initialise RS Camera
    config = rsconfig.depth_480_fps_90() #Must have colour stream
    cam = rslite.RSCam(config, True)
    cam.startStreaming()
    
    #Initialise ROS Publishers
    pubDepth = rospy.Publisher('RSL/Image/Depth', Image, queue_size=10)
    pubColor = rospy.Publisher('RSL/Image/Color', Image, queue_size=10)
    if pc:
        pubPC = rospy.Publisher('RSL/Pointcloud', PointCloud2, queue_size=10)
    rospy.init_node('RSLCamera', anonymous=True)
    publishBasicTransform('rs_frame')
    rate = rospy.Rate(120) # 120Hz
    while not rospy.is_shutdown():
        frameDepth = cam.pollDisparityFrame()
        if frameDepth:
            depthImage = generateImageFromFrame(frameDepth)
            pubDepth.publish(depthImage)
        frameColor = cam.pollColorFrame()
        frameColor = False #TODO REMOVE####################
        if frameColor:
            colorImage = generateImageFromFrame(frameColor)
            pubColor.publish(colorImage)
        framePC = cam.pollPointCloud()
        if pc and framePC:
            pointcloud = generatePointcloud(framePC)
            pubPC.publish(pointcloud)
        rate.sleep()
    cam.stopStreaming()

def generatePointcloud(frame):
    frame = rs.points(frame)
    pc = PointCloud2()
    pc.header.seq = frame.frame_number
    pc.header.stamp.secs = math.floor(frame.get_timestamp()/1000)
    pc.header.stamp.nsecs = int((frame.get_timestamp()%1000)*10**6)
    pc.header.frame_id = "rs_frame" #TODO Sort this shit out
    pc.height = 1 #As is unordered TODO check this
    pc.width = frame.size()
    pc.fields = genPointField()
    pc.is_bigendian = False #TODO Check
    pc.data = np.asanyarray(frame.get_vertices()).tobytes()
    pc.is_dense = True
    pc.point_step = 12
    pc.row_step = pc.point_step*pc.width
    return pc


def genPointField():
    #Does not support RGB
    x = PointField()
    x.name = "x"
    x.offset = 0
    x.datatype = 7
    x.count = 1
    y = PointField()
    y.name = "y"
    y.offset = 4
    y.datatype = 7
    y.count = 1
    z = PointField()
    z.name = "z"
    z.offset = 8
    z.datatype = 7
    z.count = 1
    # rgb = PointField()
    # rgb.name = "rgb"
    # rgb.offset = 12
    # rgb.
    return [x,y,z]
    
    
def publishBasicTransform(name):#TODO this should not be static and should have correct rotation
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = name

    static_transformStamped.transform.translation.x = float(0)
    static_transformStamped.transform.translation.y = float(0)
    static_transformStamped.transform.translation.z = float(0)

    quat = tf.transformations.quaternion_from_euler(
                float(math.pi/2),float(math.pi),float(-math.pi/2))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)

def generateImageFromFrame(frame):
    frame = rs.video_frame(frame)
    img = Image()
    img.header.seq = frame.frame_number
    img.header.stamp.secs = math.floor(frame.get_timestamp()/1000)
    img.header.stamp.nsecs = int((frame.get_timestamp()%1000)*10**6)
    img.header.frame_id = "rs_frame" #TODO Sort this shit out
    img.height = frame.get_height()
    img.width = frame.get_width()
    img.encoding = getROSImageFormatFromRS(frame.get_profile().format())
    img.is_bigendian = False
    img.step = frame.get_stride_in_bytes()
    bridge = CvBridge()
    data  = np.asanyarray(frame.get_data())
    img.data = bridge.cv2_to_imgmsg(data, encoding='passthrough').data
    return img

def getROSImageFormatFromRS(rsformat):
    if(rsformat == rs.format.bgr8):
        return "bgr8"
    elif(rsformat == rs.format.z16):
        return "mono16"
    else:
        print("Error: Unknown or unsupported image format")
        print(rsformat)
        raise RuntimeError


if __name__ == '__main__':
    try:
        imagePub(True)
    except rospy.ROSInterruptException:
        pass
