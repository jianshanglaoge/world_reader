#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from os import getcwd
import time
import rospy
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from moveit_python_interface_test import MoveGroupPythonInterfaceTutorial
import tf2_ros as tf
import tf2_geometry_msgs
import tf.transformations as tr
import numpy as np
import pickle
import math
from scipy.spatial.transform import Rotation as R
# This script uses a color/depth camera to get the arm to find objects and pick them up.
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR
# tag should be clearly visible to the camera. A small basket should also be placed in front of the arm.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python pick_place.py'


# def arm_status_callback(data):
#   print(rospy.get_caller_id() + data.data)
# def callback1(data):
#     temp = data.data[0, 0, 0]
#     print(temp)
# # simple listener
# def main():
#     rospy.init_node('pick_and_place', anonymous=True)
#     rospy.Subscriber('turtle1/cmd_vel', Float32MultiArray, callback1)
#     while not rospy.is_shutdown():
#         rospy.spin()


class grasper:

  def __init__(self, topic):
    self.topic = topic
    self.scores = 0
    self.pred_grasps_cam = 0
    self.contact_pts = 0
    self.Class = 0


  def callback_init_nparray(data):
    temp = Float32MultiArray()
    temp.data = data.data
    temp.layout.data_offset = 0

    # populate dimensions layout from data
    for i in range(len(data.layout.dim)):
      temp.layout.dim.append(MultiArrayDimension(data.layout.dim[i].label, data.layout.dim[i].size, data.layout.dim[i].stride))

    # convert to numpy array
    temp_np = np.array(temp.data).reshape(temp.layout.dim[0].size, temp.layout.dim[1].size, temp.layout.dim[2].size)
    
    print(temp)
    print(temp_np)
    print('callback done')

  # simple listener
  def listener():
    topic = '/turtle1/cmd_vel'
    rospy.init_node('pick_and_place', anonymous=True)
    rospy.Subscriber('turtle1/cmd_vel', Float32MultiArray, callback_init_nparray)
    rospy.spin()


def main():
  # rospy.init_node('pick_and_place', anonymous=True)
  # rospy.init_node('pick_and_place')
  # print(rospy.get_node_uri())

  # rospy.init_node('pick_place_node', anonymous=True)

  # rospy.Subscriber('/interbotix_xs_toolbox/xs_arm_status', String, arm_status_callback)
  # # Initialize the arm module along with the pointcloud and armtag modules
  saved_gripper_poses = np.load('/home/danial/Documents/contact_graspnet/results/pred_grasps_cam_july19_5pm.npy', allow_pickle=True)
  tutorial = MoveGroupPythonInterfaceTutorial()
  tutorial.group.set_pose_reference_frame('wx250s/arm_base_link')
  tutorial.group.set_end_effector_link(link_name='wx250s/gripper_link')
  tf_buffer = tf.Buffer()
  tf_listener = tf.TransformListener(tf_buffer)

  # rospy.Subscriber(name, data_class) wtf is this
  tutorial.pose_goal.position.x = 0.5
  tutorial.pose_goal.position.y = 0.0
  tutorial.pose_goal.position.z = 0.2
  tutorial.pose_goal.orientation.x = 0.2147255
  tutorial.pose_goal.orientation.y = -0.0062482
  tutorial.pose_goal.orientation.z = -0.2432511
  tutorial.pose_goal.orientation.w = 0.9458768


  # temp rotation matrix. model output pose frame different than wx250s frame
  # see wx250s.urdf and contact_graspnet paper figure 3. (axis: rgb=xyz)
  # rotataion matrix rotate about y axis by 90 degrees
  roty45 = R.from_euler('y', math.pi/2, degrees=False)
  
  # rotataion matrix rotate about z axis by -90 degrees
  rotz45 = R.from_euler('z', -math.pi/2, degrees=False)

  


  for i in range(saved_gripper_poses.shape[0]):
    temp = saved_gripper_poses[i]
    # create a temporary pose_msg to transform the goal pose to the 'wx250s/arm_base_link' frame
    pose_msg = PoseStamped()
    pose_msg.pose.position.x = temp[0,3]
    pose_msg.pose.position.y = temp[1,3]
    pose_msg.pose.position.z = temp[2,3]
    
    # apply rotation vector to goal pose
    temp[:3,:3] = roty45.apply(rotz45.apply(temp[:3,:3]))

    temp_quaternion = tr.quaternion_from_matrix(temp)
    pose_msg.pose.orientation.x = temp_quaternion[0]
    pose_msg.pose.orientation.y = temp_quaternion[1]
    pose_msg.pose.orientation.z = temp_quaternion[2]
    pose_msg.pose.orientation.w = temp_quaternion[3]



    trans = tf_buffer.lookup_transform('wx250s/arm_base_link', 'wx250s/camera/camera_color_optical_frame', rospy.Time(), rospy.Duration(2.0))

    # transform goal_pose from ~camera_color_optical_frame to ~arm_base_link
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, trans)

    # set the moveit_interface variables
    tutorial.pose_goal = pose_transformed.pose
    tutorial.go_to_pose_goal()
    
    print("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    
  rospy.spin()
  # armtag = InterbotixArmTagInterface()

  # # set initial arm and gripper pose
  # bot.arm.set_ee_pose_components(x=0.3, z=0.2)
  # bot.gripper.open()

  # # get the ArmTag pose
  # bot.arm.set_ee_pose_components(y=-0.3, z=0.2)
  # time.sleep(0.5)
  # armtag.find_ref_to_arm_base_transform()
  # bot.arm.set_ee_pose_components(x=0.3, z=0.2)

  # # get the cluster positions
  # # sort them from max to min 'x' position w.r.t. the 'wx200/base_link' frame
  # success, clusters = pcl.get_cluster_positions(ref_frame="wx200/base_link", sort_axis="x", reverse=True)

  # # pick up all the objects and drop them in a virtual basket in front of the robot
  # for cluster in clusters:
  #     x, y, z = cluster["position"]
  #     bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
  #     bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
  #     bot.gripper.close()
  #     bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
  #     bot.arm.set_ee_pose_components(x=0.3, z=0.2)
  #     bot.gripper.open()
  # bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass