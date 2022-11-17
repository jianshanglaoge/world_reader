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
import numpy as np
import pickle
import math
from scipy.spatial.transform import Rotation as R

# This script uses listens to the contact_graspnet topic over the ROS Brigde, 
# unpacks the serialized Float32MultiArray msg

def callback(data):
  temp = Float32MultiArray()
  temp.data = data.data
  temp.layout.data_offset = 0

  # populate dimensions layout from data
  for i in range(len(data.layout.dim)):
    temp.layout.dim.append(MultiArrayDimension(data.layout.dim[i].label, data.layout.dim[i].size, data.layout.dim[i].stride))

  # convert to numpy array
  temp_np = np.array(temp.data).reshape(temp.layout.dim[0].size, temp.layout.dim[1].size, temp.layout.dim[2].size)

  # unpack into usable variables
  new_pred_grasps_cam = temp_np[:, :4, :]
  new_contact_pts = temp_np[:, 4, :3]
  new_scores = temp_np[:, 4, 3]

  # temp_np_2 = temp_np[:, 5, :]

  print(temp)
  print(temp_np)
  print('callback done')

# simple listener
def listener():
  rospy.init_node('pick_and_place', anonymous=True)
  rospy.Subscriber('turtle1/cmd_vel', Float32MultiArray, callback)
  rospy.spin()


if __name__ == '__main__':
  listener()