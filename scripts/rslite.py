#!/usr/bin/env python
import pyrealsense2 as rs
import cv2
import threading

class RSCam():
    paused = True
    depth_flag = False
    color_flag = False
    depth_frame = None
    color_frame = None

    def __init__(self, config = None, pointcloud = False):
        """Simple wrapper that may be replaced with advanced mode"""
        #Initialise general settings and variables
        self.pipeline = rs.pipeline()
        if config is not None:
            self.config = config
        else:
            self.config = self.defaultConfig()
        self.profile = self.pipeline.start(self.config)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        

    def __del__(self):
        """Gracefully exit"""
        if self.streamThread is not None:
            self.paused = True
            self.streamThread.join()
        self.pipeline.stop()

    def defaultConfig(self):
        """Sets the default config"""
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        return config
    
    def startStreaming(self):
        """Start streams"""
        self.paused = False
        self.streamThread = threading.Thread(target = self.__stream)
        self.streamThread.start()

    def stopStreaming(self):
        self.paused = True
        if self.streamThread is not None:
            self.streamThread.join()
        
    def __stream(self):
        while not self.paused:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if depth_frame:
                self.depth_frame = depth_frame
                self.depth_flag = True
            if color_frame:
                self.color_frame = color_frame
                self.color_flag = True
    
    def pollDepthFrame(self):
        if self.depth_flag:
            self.depth_flag = False
            return self.depth_frame
        else:
            return False
    
    def pollColorFrame(self):
        if self.color_flag:
            self.color_flag = False
            return self.color_frame
        else:
            return False

    def getInfo(self):
        pass


