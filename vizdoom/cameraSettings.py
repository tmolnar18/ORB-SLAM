#############################################################################################
#Author: Thomas Molnar
#Date: 18 Feb 2019
#File: cameraSettings.py
#Description: Uses agent and camera position data from main.py to generate a .yaml file 
#containing camera settings to be used with ORB-SLAM2.
##############################################################################################

import numpy as np
import os
import math
 
def setProjectionMatrix(angleOfView, near, far, M, xPos, yPos, zPos):  
    #Set the basic projection matrix
    # M[row][col]
    scale = 1 / math.tan(angleOfView * 0.5 * math.pi / 180) 
    M[0][0] = xPos * scale # scale the x coordinates of the projected point 
    M[1][1] = yPos * scale # scale the y coordinates of the projected point 
    M[2][2] = zPos * -far / (far - near) # used to remap z to [0,1] 
    M[3][2] = zPos * -far * near / (far - near) # used to remap z [0,1] 
    M[2][3] = -1 # set w = -z 
    M[3][3] = 0 
    return M

def setCameraMatrix(imageWidth, imageHeight, M):
    # Set the elements of the camera matrix
    M[0][0] = 1 # fx, Assuming focal length of 1 to image
    M[1][1] = 1 # fy
    M[0][2] = imageWidth/2  # cx
    M[1][2] = imageHeight/2 # cy 
    M[2][2] = 1
    return M

def writeYamlFile(imageWidth, imageHeight, M, CM, fps, far_threshold):
    # Save camera projection matrix to file for use by ORB-SLAM2
    cam_file = open('settings.yaml', 'w')

    # Set Camera Parameters
    # fx, fy, cx, and cy are default ROS values, see link below:
    # https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
    # Focal lengths set to screen size
    fx = 320.0 #M[0][0]
    fy = 240.0 #M[1][1] 
    # Half the screen width/height
    cx = 160.0 #M[0][2]
    cy = 120.0 #M[1][2]
    k1 = 0
    k2 = 0
    p1 = 0
    p2 = 0
    k3 = 0
    rgb = 1
    IR = 40.0
    depthmap_factor = 5000.0

    cam_file.write("%YAML:1.0\n\n")
    # Write Camera Parameters to .yaml file
    cam_file.write("#-------------------------------------------\n# Camera Parameters. Adjust as necessary.\n#-------------------------------------------\n\n")
    cam_file.write("# Camera calibration and distortion parameters (OpenCV)\n")
    cam_file.write("Camera.fx: %.6f\n"% fx)
    cam_file.write("Camera.fy: %.6f\n"% fy)
    cam_file.write("Camera.cx: %.6f\n"% cx)
    cam_file.write("Camera.cy: %.6f\n\n"% cy)
    cam_file.write("Camera.k1: %.6f\n"% k1)
    cam_file.write("Camera.k2: %.6f\n"% k2)
    cam_file.write("Camera.p1: %.6f\n"% p1)
    cam_file.write("Camera.p2: %.6f\n"% p2)
    cam_file.write("Camera.k3: %.6f\n\n"% k3)
    cam_file.write("Camera.width: %d\n"% imageWidth)
    cam_file.write("Camera.height: %d\n\n"%  imageHeight)
    cam_file.write("# Camera frames per second\nCamera.fps: %.1f\n\n"% fps)
    cam_file.write("# IR projector baseline times fx (aprox.)\nCamera.bf: %.1f\n\n"% IR)
    cam_file.write("# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)\nCamera.RGB: %d\n\n"% rgb)
    cam_file.write("# Close/Far Threshold. Baseline times.\nThDepth: %.1f\n\n"% far_threshold)
    cam_file.write("# Depthmap values factor.\nDepthMapFactor: %.1f\n\n"% depthmap_factor)
    
    # Set ORB Parameters
    features_per_image = 9000
    scale_factor = 1.2
    scale_levels = 8
    iniThFAST = 10
    minThFAST = 1

    # Write ORB Parameters to .yaml file
    cam_file.write("#-------------------------------------------\n# ORB Parameters\n#-------------------------------------------\n\n")
    cam_file.write("# ORB Extractor: Number of features per image.\nORBextractor.nFeatures: %d\n\n"% features_per_image)
    cam_file.write("# ORB Extractor: Scale factor between levels in the scale pyramid.\nORBextractor.scaleFactor: %.1f\n\n"% scale_factor)
    cam_file.write("# ORB Extractor: Number of levels in the scale pyramid.\nORBextractor.nLevels: %d\n\n"% scale_levels)
    cam_file.write("# ORB Extractor: Fast Threshold\n# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.\n# Firstly we impoese iniThFAST. If no corners are detected we impose a lower value minThFAST\n# You can lower these values if your images have low contrast.\n")
    cam_file.write("ORBextractor.iniThFAST: %d\n"% iniThFAST)
    cam_file.write("ORBextractor.minThFAST: %d\n\n"% minThFAST)

    # Set Viewer Parameters
    keyframe_size = 0.05
    keyframe_linewidth = 1
    graph_linewidth = 0.9
    point_size = 1
    camera_size = 0.08
    camera_linewidth = 3
    viewpointX = 0
    viewpointY = -0.7
    viewpointZ = -1.8
    viewpointF = 500

    # Write Viewer Parameters to .yaml file
    cam_file.write("#-------------------------------------------\n# Viewer Parameters\n#----------------------------------------\n")
    cam_file.write("Viewer.KeyFrameSize: %.2f\n"% keyframe_size)
    cam_file.write("Viewer.KeyFrameLineWidth: %d\n"% keyframe_linewidth)
    cam_file.write("Viewer.GraphLineWidth: %.1f\n"% graph_linewidth)
    cam_file.write("Viewer.PointSize: %d\n"% point_size)
    cam_file.write("Viewer.CameraSize: %.2f\n"% camera_size)
    cam_file.write("Viewer.CameraLineWidth: %d\n"% camera_linewidth)
    cam_file.write("Viewer.ViewpointX: %d\n"% viewpointX)
    cam_file.write("Viewer.ViewpointY: %.1f\n"% viewpointY)
    cam_file.write("Viewer.ViewpointZ: %.1f\n"% viewpointZ)
    cam_file.write("Viewer.ViewpointF: %d\n"% viewpointF)
    cam_file.close()
    return

def writeOpenCVCalibFile(imageWidth, imageHeight, M):
    cam_file = open('settings.txt', 'w')  
    # Format: fx fy cx cy k1 k2 p1 p2
    cam_file.write("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n"%(M[0][0], M[1][1], M[0][2], M[1][2], 0, 0, 0, 0))
    cam_file.write("%d %d\n"% (imageWidth, imageHeight))
    cam_file.write("full\n")
    cam_file.write("%d %d"% (imageWidth, imageHeight))
    cam_file.close()
    return

def main(imageWidth, imageHeight, xPos, yPos, zPos, fps):
    projMatrix = np.zeros((4,4))
    cameraMatrix = np.zeros((3,3))
    angleOfView = 60
    near = 1
    far = 1800
    projMatrix = setProjectionMatrix(angleOfView, near, far, projMatrix, xPos, yPos, zPos)
    cameraMatrix = setCameraMatrix(imageWidth, imageHeight, cameraMatrix)
    writeYamlFile(imageWidth, imageHeight, projMatrix, cameraMatrix, fps, far)
    #writeOpenCVCalibFile(imageWidth, imageHeight, cameraMatrix)
    return

if __name__ == '__main__':  
    main()
