#!/usr/bin/env python

##################################################################################
# Creator: Thomas Molnar
# Date: 18 Feb 2019
# File: main.py
# Description: Generates an instance of a VizDoom scenario, obtains in game agent 
# position, aim angles and field of view to generate camera projection matrix. Then
# utilizes ORB-SLAM2 to generate point cloud of the environment.
###################################################################################
from vizdoom import *
import vizdoom as vzd
from PIL import Image
import rospy
from sensor_msgs.msg import Image as Im
import numpy as np
import os
from time import time
from time import sleep 

import cameraSettings
import ROSInterface

os.environ['CUDA_VISIBLE_DEVICES'] = '1'
config_file_path = "scenarios/my_way_home.cfg"
resolution = (42, 42)
# Set these parameters to match the Screen Resolution of the game
# 640X480 recommended screen size for ORB-SLAM2
imageWidth = 320
imageHeight = 240
episodes = 1

# Approximate fps for vizdoom in Spectator mode
fps = 35

# Setup publisher
# Declare publisher, publishing to camera/rgb/image_raw topic using Image message type 
publisher_rgb = rospy.Publisher('camera/rgb/image_raw', Im, queue_size=8)
publisher_depth = rospy.Publisher('camera/depth_registered/image_raw', Im, queue_size=8)

# Converts and down-samples the input image
def preprocess(state):
    img = state.screen_buffer
    img = np.moveaxis(img, [0,1,2], [2,0,1])
    img = cv2.resize(img, resolution)
    img = ToTensor() (img)
    img = img.unsqueeze(0).cuda()
    return img

# Creates and initializes ViZDoom environment.
def initialize_vizdoom(config_file_path):
    print("Initializing doom...")
    game = DoomGame()
    game.load_config(config_file_path)
    game.set_window_visible(True)
    # Enable user to play the agent 
    game.add_game_args("+freelook1 ")
    game.set_mode(Mode.SPECTATOR)    
    game.add_game_args("+vid_forcesurface 1")
    #game.set_mode(Mode.PLAYER)
    # OpenCV uses a BGR colorspace by default
    game.set_screen_format(ScreenFormat.BGR24)
    # Obtain rgb and depth of image
    #game.set_screen_format(ScreenFormat.CBCGCRDB)
    #game.set_screen_format(ScreenFormat.CRCGCBDB)
    game.set_screen_resolution(ScreenResolution.RES_640X480)

    # Enables labeling of the in game objects 
    game.set_labels_buffer_enabled(True)
    # Enable rendering of depth buffer
    game.set_depth_buffer_enabled(True)
    game.clear_available_game_variables()

    # Obtain players X, Y, Z coordinates and camera Pitch and Angle
    game.add_available_game_variable(vzd.GameVariable.POSITION_X)
    game.add_available_game_variable(vzd.GameVariable.POSITION_Y)
    game.add_available_game_variable(vzd.GameVariable.POSITION_Z)

    game.add_available_game_variable(vzd.GameVariable.ANGLE)
    game.add_available_game_variable(vzd.GameVariable.PITCH)
    
    game.init()
    print("Doom initialized.")
    return game

if __name__ == '__main__':
    game = initialize_vizdoom(config_file_path)
    
    for i in range(episodes):
       print("Episode #" + str(i + 1))
       # Start new game episode
       game.new_episode()       

       while not game.is_episode_finished():
          # Get the game state
          state = game.get_state()
          # Get the screen buffer from the game state          
          screen_buf = state.screen_buffer
          # Get the depth buffer from the game state
          depth_buf = state.depth_buffer
          # Retrive agent's last action
          last_action = game.get_last_action()
          # Advance actor's action
          # Skiprate "prolongs" the action and skip frames
          skiprate = 1
          game.advance_action(skiprate)
          
          # If the player has moved, update position coords, generate new projection matrix and save the new screen
          if last_action != [0, 0, 0, 0, 0]:
             timestamp = hash(time())
             # Store x/y/z coordinates to variables          
             xPos = state.game_variables[0]
             yPos = state.game_variables[1]
             zPos = state.game_variables[2]
          
             # Build the camera.yaml file, passing in the game frame width and height with the player's x, y, and z pos
             cameraSettings.main(imageWidth, imageHeight, xPos, yPos, zPos, fps)
             
             # Pass screen buffer and depth buffer to ROSInterface to publish the image for ORB SLAM
             #screen_buf = np.transpose(screen_buf, axes=(1,2,0)) 
             #im = Image.fromarray(screen_buf)
             ROSInterface.main(screen_buf, depth_buf, publisher_rgb, publisher_depth)

             # Save screen buffer to images folder
             #im.save('images/screen_{}.jpg'.format(timestamp))
          
    print("Episode Done")
    sleep(2.0)
    

