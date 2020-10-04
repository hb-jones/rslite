#!/usr/bin/env python
import pyrealsense2 as rs


def depth_720_color_720_fps_30():
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    return config 

def depth_720_color_540_fps_30_60():
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)
    return config 

def depth_720_color_1080_fps_30():
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    return config

def depth_480_color_480_fps_60():
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    return config

def depth_480_fps_90():
    config = rs.config()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 90)
    return config

def depth_144_fps_300():
    config = rs.config()
    config.enable_stream(rs.stream.depth, 256, 144, rs.format.z16, 300)
    return config

def depth_100_fps_300():
    config = rs.config()
    config.enable_stream(rs.stream.depth, 848, 100, rs.format.z16, 300)
    return config