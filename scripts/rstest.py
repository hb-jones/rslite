import time
import numpy as np
import cv2
from rslite import rsconfig
from rslite import rslite
import pyrealsense2 as rs

def pcInfo():
    config = rsconfig.depth_480_fps_90()
    cam = rslite.RSCam(config, pointcloud = True)
    cam.startStreaming()
    x = 0
    while x<100:
        pc = cam.pollPointCloud()
        if pc:    
            x+=1
            pc = rs.points(pc)
            verticies = np.asanyarray(pc.get_vertices())
            print("Type Verticies:       {}".format(type(verticies)))
            print("Value Verticies:      {}".format(verticies))
            print("Type Verticies[0]:    {}".format(type(verticies[0])))
            print("Type Verticies[0]:    {}".format(verticies[0]))
            print("Type Verticies[0][0]: {}".format(type(verticies[0][0])))
            print("Type Verticies[0][0]: {}".format(verticies[0][0]))
            print()
    cam.stopStreaming()



def framerateTest():
    config = rsconfig.depth_480_color_480_fps_60()
    cam = rslite.RSCam(config)
    cam.startStreaming()
    
    time.sleep(2)
    tstart = time.time()
    i = 0
    while i<300:
        frame = cam.pollDepthFrame()
        if frame is not False:
            avdist = 0
            for x in range(848):
                for y in range(100):
                    avdist += frame.get_distance(x,y)
                    
            avdist  = avdist / (848*100)
            print("Average Distance"+str(avdist))
        else:
            i-= 1
        i += 1
    total = time.time()-tstart
    print("Total " + str(total))

def differenceView():
    config = rsconfig.depth_480_color_480_fps_60()
    cam = rslite.RSCam(config)
    cam.startStreaming()
    time.sleep(0.5)
    sceneData = 0
    errorMaxm = 0.11 #in meters
    errorMax = errorMaxm / cam.depth_scale
    i=0
    while True:
        frame = cam.pollDepthFrame()
        if frame and i<100:
            i+=1
            sceneData = (sceneData + np.asanyarray(frame.get_data()))
        if i>99:
            sceneData = sceneData/100
            break
    try:
        while True:
            frame = cam.pollDepthFrame()
            if frame:
                depth_image = np.asanyarray(frame.get_data())
                difference = abs(depth_image-sceneData)
                print("Difference "+str(np.mean(difference)))
                print("MAx: "+str(difference.max()))
                print("MAx: "+str(difference.min()))

                diff = np.where((difference < errorMax), 5/cam.depth_scale, depth_image)

                # Render images
                
                scene_colormap = cv2.applyColorMap(cv2.convertScaleAbs(sceneData, alpha=0.03), cv2.COLORMAP_JET)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(diff, alpha=0.03), cv2.COLORMAP_JET)
                difference_colormap = cv2.applyColorMap(cv2.convertScaleAbs(difference, alpha=0.03), cv2.COLORMAP_JET)
                images = np.hstack((scene_colormap, depth_colormap, difference_colormap))
                cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Align Example', images)
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
    finally:
        print("About to end")
        cam.stopStreaming()


if __name__ == '__main__':
    pcInfo()