#!/usr/bin/env python
# -*- coding: utf-8 -*-
# file is for debugging.

import sys
from os import getcwd
import time
import rospy
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped

import tf2_ros as tf
import tf2_geometry_msgs
import tf.transformations as tr
import pickle
import math
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
import cv2
import PIL
import ros_numpy
# This script uses listens to the contact_graspnet topic over the ROS Brigde, 
# unpacks the serialized Float32MultiArray msg
world='world1'

def callback_1(data):
  #plt.imshow(data)
  image=ros_numpy.numpify(data)
  image=image[:,:,::-1]
  np.save('data/'+world+'_1080pcamera.npy',image)
  image=PIL.Image.fromarray(image)
  #image.show()
  image_rgb=image.convert('RGB')
  image_rgb.save('data/'+world+'_1080pcamera.jpg')
# simple listener
def callback_2(data):
  #plt.imshow(data)
  image=ros_numpy.numpify(data)
  image=image[:,:,::-1]
  np.save('data/'+world+'_1080pcamera_0.npy',image)
  image=PIL.Image.fromarray(image)
  #image.show()
  image_rgb=image.convert('RGB')
  image_rgb.save('data/'+world+'_1080pcamera_0.jpg')
# simple listener
def callback_3(data):
  #plt.imshow(data)
  image=ros_numpy.numpify(data)
  image=image[:,:,::-1]
  np.save('data/'+world+'_1080pcamera_1.npy',image)
  image=PIL.Image.fromarray(image)
  #image.show()
  image_rgb=image.convert('RGB')
  image_rgb.save('data/'+world+'_1080pcamera_1.jpg')
# simple listener
def listener():
  rospy.init_node('reader', anonymous=True)
  rospy.Subscriber('/camera/color/image_raw_1080p_1',Image, callback_1)
  rospy.Subscriber('/camera/color/image_raw_1080p_2',Image, callback_2)
  rospy.Subscriber('/camera/color/image_raw_1080p_3',Image, callback_3)
  rospy.spin()

if __name__ == '__main__':
  worldnum=sys.argv[1]
  world='world'+worldnum
  listener()
