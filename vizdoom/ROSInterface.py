#!/usr/bin/python

###########################################################################################
# Creator: Thomas Molnar
# Date: 19 Feb 2019
# File: convertROS.py    
# Description: Converts openCV images to ROS image messages to be used with ORB-SLAM2
########################################################################################## 
import cv2
import os
import sys
import rospy 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def publishImage(screen, depth, publisher_rgb, publisher_depth):
   # Create node to use for publishing
   rospy.init_node('VizDoom', anonymous=True)
   rate = rospy.Rate(10) # 10 Hz
   # Create cv2 bridge 
   bridge = CvBridge()
   # Publish screen buffer from VizDoom with publisher
   try:
      # Verify screen output 
      #cv2.imshow('Vizdoom screen', screen)
      #cv2.waitKey(1)
      # Publish rgb screen output 
      publisher_rgb.publish(bridge.cv2_to_imgmsg(screen, "bgr8"))
      # Publish depth screen output
      publisher_depth.publish(bridge.cv2_to_imgmsg(depth, "mono8"))
      rate.sleep()
      # To verify that image is being published accurately, use the following in the command line:
      # rosrun image_view image_view image:=/vizdoom_topic/image_raw

      # For visualizing the ORB_SLAM-generated point cloud, use the following in the command line, and be sure to be in the directory /opt/ORB_SLAM2/:
      # rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt /vizdoom/settings.yaml
      # RGB-D node will read images from topic /camera/rgb/image_raw
   except CvBridgeError as e:
      print(e)
   return   

def main(screen, depth, publisher_rgb, publisher_depth):
   try:
      publishImage(screen, depth, publisher_rgb, publisher_depth)
   except rospy.ROSInterruptException:
      pass
   return

if __name__ == '__main__':
   main() 
