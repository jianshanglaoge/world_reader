#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
from os import getcwd
import time
import rospy
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from std_srvs.srv import SetBool, Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker
from control_msgs.msg import FollowJointTrajectoryActionResult as ActionResult, FollowJointTrajectoryActionGoal as ActionGoal
import moveit_commander
import moveit_msgs
from moveit_commander.conversions import pose_to_list
# from interbotix_perception_modules.armtag import InterbotixArmTagInterface
# from interbotix_xs_modules.arm import InterbotixManipulatorXS
# from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from moveit_python_interface_test import MoveGroupPythonInterfaceTutorial
import tf2_ros as tf
import tf2_geometry_msgs
import tf.transformations as tr
import numpy as np
import pickle
import math
from scipy.spatial.transform import Rotation as R

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is PoseStamped:
    return all_close(goal.pose, actual, tolerance)

  elif type(goal) is Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class grasper:
  """
  MoveGroupCommander API accessible through self.arm and self.gripper
  
  Inputs:
    - topic: string, topic name of the graspnet output
    - pose_result_topic: string, topic name of the arm result
    - planning_time: float, maximum planning time in seconds

  Class functions:\n
  :func:`Get_cur_attempt`: Returns attempt #.
  :func:`Get_pose`:        Returns the current pose.
  :func:`Get_pose_raw`: Current pose before transform camera -> arm_base_link


  :func:`moveit_cmd_grasp`: Grasps the object.

  :func:`gen_Marker`: Generates a marker for the current attempt.
  """


  def __init__(self, topic_sub_poses, planning_time):
    self.topic_sub_poses = topic_sub_poses
    self.arm_result_topic = '/arm_controller/follow_joint_trajectory/result'
    self.gripper_result_topic = '/gripper_controller/follow_joint_trajectory/result'
    # self.arm_goal_topic = '/arm_controller/follow_joint_trajectory/goal'
    # self.gripper_goal_topic = '/gripper_controller/follow_joint_trajectory/goal'
    if planning_time is None:
      self.planning_time = 10.0
    else:
      self.planning_time = planning_time

    # filtered
    self.pred_grasps_cam = None
    self.pred_grasps_cam_raw = None
    self.contact_pts = None
    self.scores = None
    self.gripper_widths = None
    # unfiltered
    self.pred_grasps_cam_ALL = None
    self.contact_pts_ALL = None
    self.scores_ALL = None
    self.gripper_widths_ALL = None

    self.pose = None        # top4 transformed
    self.pose_raw = None    # top4 in camera_frame
    self.pose_marker = None

    self.tf_Buffer = tf.Buffer()
    self.tf_listener = tf.TransformListener(self.tf_Buffer)
    self.tf_lookup = self.tf_Buffer.lookup_transform('wx250s/arm_base_link', 'wx250s/camera/camera_color_optical_frame', rospy.Time(), rospy.Duration(3.0))

    self.sub_contact_graspnet = rospy.Subscriber(self.topic_sub_poses, Float32MultiArray, self.sub_contact_graspnet_callback, 1)
    # self.ServiceCaller = rospy.Service('/contact_graspnet/request_inference', SetBool, self.handler)
    self.ServiceCaller = rospy.ServiceProxy('/contact_graspnet/request_inference', Trigger)
    # not used
    # self.sub_arm_result = rospy.Subscriber(self.arm_result_topic, ActionResult, self.arm_result)
    # self.sub_gripper_result = rospy.Subscriber(self.gripper_result_topic, ActionResult, self.gripper_result)

    self.marker_pose_pub = rospy.Publisher('visualization_marker/current_attempt', Marker, queue_size = 100)
    self.Marker = Marker() # possibly not needed

    self.moveit_cmd_init()
    self.cur_attempt = 0
    self.CRJ = True


  def moveit_cmd_init(self):  
    moveit_commander.roscpp_initialize(sys.argv)
    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Interbotix
    ## arm so we set ``group_name = interbotix_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Interbotix Arm:
    group_arm = "interbotix_arm"
    self.group = moveit_commander.MoveGroupCommander(group_arm)
    # self.pub_arm_goal = rospy.Publisher('wx250s/move_group/goal', ActionGoal, queue_size=10)
    # self.pub_gripper_goal = rospy.Publisher('wx250s/gripper_controller/follow_joint_trajectory/goal', ActionGoal, queue_size=10)
    self.group.set_pose_reference_frame('wx250s/arm_base_link')
    self.group.set_end_effector_link(link_name='wx250s/gripper_link')
    self.group.set_planner_id('RRTConnectkConfigDefault')
    self.group.set_planning_time(self.planning_time) # seconds
    self.group.set_num_planning_attempts(10)
    self.group.set_max_velocity_scaling_factor(0.09)
    self.group.set_max_acceleration_scaling_factor(0.09)
    self.group.set_goal_tolerance(0.01)

    group_gripper = "interbotix_gripper"
    self.gripper = moveit_commander.MoveGroupCommander(group_gripper)
    self.gripper.set_planner_id('RRTConnectkConfigDefault')
    self.gripper.set_planning_time(self.planning_time) # seconds
    self.gripper.set_num_planning_attempts(10)
    self.gripper.set_max_velocity_scaling_factor(0.2)
    self.gripper.set_max_acceleration_scaling_factor(0.2)
    self.gripper.set_goal_tolerance(0.001)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    self.display_trajectory_publisher = rospy.Publisher("move_group/display_planned_path",
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

  def request_inference(self):
    print('service call available: ', self.ServiceCaller.wait_for_service(timeout=2.0))
    print(self.ServiceCaller.resolved_name)
    self.CRJ = False
    clear_listener(self)
    temp = self.ServiceCaller()
    rospy.loginfo(msg='Service Call returned: \nSuccess: {}\nMessage:{}'.format(temp.success, temp.message))
    if temp.success:
      return True
    else:
      print('Service call failed, bro, try again')
      return False


  def gen_Marker(self):
    """
    Publishes self.Marker Class object from self.pose .
    """
    self.Marker.header.frame_id = self.pose.header.frame_id
    self.Marker.header.stamp = rospy.Time.now()
    self.Marker.ns = "basic_shapes"
    self.Marker.id = 0
    self.Marker.type = Marker.ARROW
    self.Marker.action = Marker.ADD
    self.Marker.pose = self.pose.pose
    self.Marker.scale.x = 0.1
    self.Marker.scale.y = 0.1
    self.Marker.scale.z = 0.1
    self.Marker.color.r = 0.0
    self.Marker.color.g = 0.5
    self.Marker.color.b = 0.5
    self.Marker.color.a = 1.0
    self.Marker.lifetime = rospy.Duration().from_sec(60)
    # publish the marker
    self.marker_pose_pub.publish(self.Marker)

  def go_to_pose_goal(self):
    """
    Move Interbotix arm to a pose goal. (10s timeout)

    :return: True if success, False if not.
    """

    print('number of previous attempts: ', self.cur_attempt)
    print('pose_raw: ', self.pose_raw.pose)
    print("============ Pose Goal ============\n" + str(self.pose))

    # Planning to a Pose Goal
    # ^^^^^^^^^^^^^^^^^^^^^^^
    # We can plan a motion for this group to a desired pose for the
    # end-effector:
    self.group.set_pose_target(self.pose)

    # Now, we call the planner to compute the plan and execute it.
    plan = self.group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

    # TODO: Replace with a different verification of successful grasp.
    current_pose = self.group.get_current_pose().pose
    return all_close(self.pose, current_pose, 0.01)

  def go_to_myhome(self):
    """
    Move Interbotix arm to a custom home position. (10s timeout)

    :return: True if success, False if not.
    """
    print('number of previous attempts: ', self.cur_attempt)
    print('pose_raw: ', self.pose_raw.pose)
    print("============ Pose Goal ============\n" + str(self.pose))
    # create a new home pose
    myhome = 'myhome'
    r = [-0.0002771538426351583, 0.0037198056779725874, 0.00470465451352009, -0.007525991781174746, 1.236726321729079, 0.00011122886574810309]
    
    self.group.go(wait=True)

    # Planning to a Joint Goal to custom home position
    # ^^^^^^^^^^^^^^^^^^^^^^^
    self.group.set_joint_value_target(r)
    # Now, we call the planner to compute the plan and execute it.
    plan = self.group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

    # TODO: Replace with a different verification of successful grasp.
    current_pose = self.group.get_current_pose().pose
    return all_close(self.pose, current_pose, 0.01)

  def Get_cur_attempt(self):
    return self.cur_attempt
    
  def Get_pose(self):
    return self.pose
  
  def Get_pose_raw(self):
    return self.pose_raw

  def Gen_NextPose(self):
    # generate next pose
    self.pose_maker() 
    self.gen_Marker()

    return self.pose

  def sub_contact_graspnet_callback(self, Float32MultiArray, *args, **kwargs):
    """
    Subscriber Callback function for /contact_graspnet_pose topic.
    uppacks the Float32MultiArray message,  self.pose (PoseStamped)
    Prompts the User to attempt grasp.
    """
    rospy.loginfo('service: /contact_graspnet/request_inference - generated {} grasp poses.'.format(Float32MultiArray.layout.dim[0].size))
    if Float32MultiArray.layout.dim[0].size < 1: 
      # rospy.loginfo('inference model generated no poses. call service again')
      self.CRJ = True
      check_max(self)
      return
    # elif: Float32MultiArray.layout.dim[0].size == 1:
    #   self.pose



    # print(Float32MultiArray)
    # convert to numpy array
    temp_np = np.array(Float32MultiArray.data).reshape(Float32MultiArray.layout.dim[0].size, Float32MultiArray.layout.dim[1].size, Float32MultiArray.layout.dim[2].size)
    # if Float32MultiArray.layout.dim[0].size == 1:
    #   temp_np = temp_np.reshape(1, Float32MultiArray.layout.dim[1].size, Float32MultiArray.layout.dim[2].size)
    # temp_np_raw = np.array(Float32MultiArray.data).reshape(Float32MultiArray.layout.dim[0].size, Float32MultiArray.layout.dim[1].size, Float32MultiArray.layout.dim[2].size)
    print('temp_np: ', temp_np.shape)
    # save unfiltered data to readable variables
    self.pred_grasps_cam_ALL = temp_np[:, :4, :]
    self.contact_pts_ALL = temp_np[:, 4, :3]
    self.scores_ALL = temp_np[:, 4, 3]
    self.gripper_widths_ALL = temp_np[:, 5, 3]

    # Remove poses with too large gripper of widths
    # TODO train model to better filter valid wx250s gripper poses.
    self.indices_valid = np.where(self.gripper_widths_ALL < 0.037)[0]
    self.pred_grasps_cam = self.pred_grasps_cam_ALL[self.indices_valid]
    self.contact_pts = self.contact_pts_ALL[self.indices_valid]
    self.scores = self.scores_ALL[self.indices_valid]
    self.gripper_widths = self.gripper_widths_ALL[self.indices_valid]

    # find top4 pose scores
    # temp_scores = temp_np[:, 4, 3]
    # self.scores = temp_scores.reshape(Float32MultiArray.layout.dim[0].size, 1)
    idx = self.scores.argsort()[::-1][:4]
    self.top4_index = idx # top4 index in FILTERED pred_grasps
    
    #  save camera frame versions
    self.pred_grasps_cam_raw = self.pred_grasps_cam[idx]

    # self.scores = self.scores[idx].reshape(Float32MultiArray.layout.dim[0].size, 1)
    self.contact_pts = self.contact_pts[idx]
    self.scores = self.scores[idx].reshape(len(idx), 1) # [N,]->[N,1] - Max N: 4
    self.gripper_widths = self.gripper_widths[idx].reshape(len(idx), 1) # [N,]->[N,1] - Max N: 4
    self.pred_grasps_cam = self.pred_grasps_cam[idx]

    rospy.loginfo('sevice: /contact_graspnet/request_inference -  Keeping top {} valid grasp poses.'.format(self.scores.shape[0]))

    # temp rotation matrix. model output pose frame different than wx250s frame
    # see wx250s.urdf and contact_graspnet paper figure 3. (axis: rgb=xyz)
    # rotataion matrix rotate about y axis by 90 degrees
    roty45 = R.from_euler('y', math.pi/2, degrees=False)
    
    # rotataion matrix rotate about z axis by -90 degrees
    rotz45 = R.from_euler('z', -math.pi/2, degrees=False)
    
    # apply rotation vector to goal pose
    # loop over 1st DIMENSION of self.pred_grasps_cam
    for i in range(self.pred_grasps_cam.shape[0]):
      self.pred_grasps_cam[i,:3,:3] = roty45.apply(rotz45.apply(self.pred_grasps_cam[i,:3,:3]))
    rate = rospy.Rate(1)

    # try the 4 best poses    
    while check_max(self): # check max attempts
      rate.sleep()
      self.pose_maker()
      # get user input
      try_pose = raw_input('try? (y/n) Press Enter to continue...')
      # if user input is 'y', move arm to pose
      if try_pose == 'y':
        # go to home pose first:
        self.go_to_myhome()
        # open gripper
        self.gripper.set_named_target('Open')
        self.gripper.go(wait=True)
        self.gripper.stop()
        self.gripper.clear_pose_targets()
        # attempt grasp pose
        self.go_to_pose_goal()
        self.close_gripper()
        # finally increment the cur_attempt variable
        self.cur_attempt += 1
        # else:
        #   print('pose not solvable')
      # if user input is 'n', move arm to sleep pose
      elif try_pose == 'n':
        pass
      # if user input is 'q', quit
      elif try_pose == 'q':
        break
    
    # after all attempts, set the call request flag
    self.CRJ = True 


  def pose_maker(self):
    """
    - Generates pose attempt. 
    - Compute transform: camera -> arm_base_link.
    - Store to self.pose (GeometryPoseStamped).
    """
    # TODO: Make this a custom message type
    # MSG FORMAT: [LENGTH x 5 x 4]
    #   |4x4|= |rotation matrix|position| = |    pred_grasps_cam   |
    #          |    0 0 0           1   |   | contact_pts | scores |
    #   |1x4|= |  contact_pts  | scores |

    # make quaternion
    temp_quaternion = tr.quaternion_from_matrix(self.pred_grasps_cam[self.cur_attempt,:,:])
    temp_quaternion_raw = tr.quaternion_from_matrix(self.pred_grasps_cam_raw[self.cur_attempt,:,:])
    
    # pose_raw only for debugging
    # pose_raw - position
    self.pose_raw = PoseStamped()
    self.pose_raw.pose.position.x = self.pred_grasps_cam_raw[self.cur_attempt, 0, 3]
    self.pose_raw.pose.position.y = self.pred_grasps_cam_raw[self.cur_attempt, 1, 3]
    self.pose_raw.pose.position.z = self.pred_grasps_cam_raw[self.cur_attempt, 2, 3]
    # pose_raw - orientation
    self.pose_raw.pose.orientation.x = temp_quaternion[0]
    self.pose_raw.pose.orientation.y = temp_quaternion[1]
    self.pose_raw.pose.orientation.z = temp_quaternion[2]
    self.pose_raw.pose.orientation.w = temp_quaternion[3]
    # self.pose_raw.pose.position.y = slice[1,3]
    # self.pose_raw.pose.position.z = slice[2,3]

    # pose - position
    self.pose = PoseStamped()
    self.pose.pose.position.x = self.pred_grasps_cam[self.cur_attempt, 0, 3]
    self.pose.pose.position.y = self.pred_grasps_cam[self.cur_attempt, 1, 3]
    self.pose.pose.position.z = self.pred_grasps_cam[self.cur_attempt, 2, 3]
    # pose - orientation
    self.pose.pose.orientation.x = temp_quaternion[0]
    self.pose.pose.orientation.y = temp_quaternion[1]
    self.pose.pose.orientation.z = temp_quaternion[2]
    self.pose.pose.orientation.w = temp_quaternion[3]
    print('Grasp attempt number: {}'.format(self.cur_attempt+1))
    self.cur_attempt += 1

    # get a pose object from the camera frame to the arm base link frame
    self.pose = tf2_geometry_msgs.do_transform_pose(self.pose, self.tf_lookup)
    self.pose_raw = tf2_geometry_msgs.do_transform_pose(self.pose_raw, self.tf_lookup)

    # visualize the grasp attempt
    self.gen_Marker()

  def open_gripper(self):
    # open gripper
    self.gripper.set_named_target('Open')
    self.gripper.go(wait=True)
    self.gripper.stop()
    self.gripper.clear_pose_targets()
    self.robot.get_current_state()
    print('testing')

  def close_gripper(self):
    # uses pred_grasps_cam.
    width = self.gripper_widths[self.cur_attempt-1][0]/2
    if width < 0.015: width = 0.015
    # plan = self.gripper.plan([width, -width])
    self.gripper.go([width, -width], wait=True)
    rospy.loginfo('attempting to close gripper to inference model output width {}'.format(2*width))
    # self.gripper.clear_joint_value_targets()
    self.gripper.clear_pose_targets()
    pass

def check_max(listener):
  """
  - Checks if max attempts have been reached.
  - Max attempts is defined by number of valid grasps genereated by the model.

  - Returns False if max attempts have been reached.
  - Returns True if max attempts have not been reached.
  """
  # if listener.pred_grasps_cam is None:
  #   # val1 = raw_input('y to start. Press Enter to continue...')
  #   # if val1 == 'y': listener.request_inference()
  #   return False # do nothing
  if listener.pred_grasps_cam is None:
    return False
  elif listener.cur_attempt == 4 or listener.cur_attempt == listener.pred_grasps_cam.shape[0]:
    # print max poses and current pose attempts number
    print('max pose attempts reached. \nWaiting for Service Call.')
    print('Max poses attempts available: ', listener.pred_grasps_cam.shape[0])
    # reset variables
    clear_listener(listener)
    return False
  else:
    return True

def clear_listener(listener):
  listener.cur_attempt = 0
  listener.pred_grasps_cam = None
  listener.pred_grasps_cam_raw = None
  listener.contact_pts = None
  listener.scores = None
  listener.pose = None
  listener.pose_raw = None
  listener.pose_marker = None

def main():
  topic_pub_pose = '/contact_graspnet/output_poses'
  planning_time = 10
  # self.service = roslibpy.Service(client, topic_service, 'std_srvs/SetBool')
  rospy.init_node('pick_and_place', anonymous=True)

  # make sure service is available before creating listener.
  rospy.wait_for_service('/contact_graspnet/request_inference')
  rate = rospy.Rate(1)
  listener = grasper(topic_pub_pose, planning_time)

  # listener.request_inference() # REMOVE. debugging


  while not rospy.is_shutdown():
    rate.sleep()
    if listener.CRJ: #Carly Rae Jepson - call me maybe
      # get user input
      val1 = raw_input('request inference? (y/n) Press Enter to continue...')
      # if user input is 'y', move arm to pose
      if val1 == 'y':
        listener.request_inference()
        # time.sleep(5)# hack to preempt service request loop
      elif val1 == 'n':
        pass
      elif val1 == 'q':
        break
    

  # end main loop here
  rospy.signal_shutdown('user selected to quit')


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass




# #############################################################################
# ################################# Random ####################################
# #############################################################################


# listener = grasper('/contact_graspnet_model/output_poses')
# # print('current pose attempt: ', (listener.Get_cur_attempt()+1) )
# rate = rospy.Rate(1.0)

# def arm_result(self, ActionResult):
#   ActionResult(ActionResult)
#   print("ASDF", ActionResult)
#   return ActionResult

# def gripper_result(self, ActionResult):
#   print("ASDF", ActionResult)
#   return ActionResult





# #############################################################################
# ####################### was in sub_img_callback #############################
# #############################################################################

# elif val1 == 'r':
#   listener.gen_Marker()

# REMOVED: CONTROL IN MAIN LOOP
  # # print(self.pred_grasps_cam)
  # # generate arm pose in wx250s/arm_base_link frame
  # self.pose_maker()

  # # send arm pose goal
  # if self.go_to_pose_goal() == False:
  #   print('pose not solvable')
  # result = self.open_gripper()

  # # publish pose marker to rviz
  # self.gen_Marker()

  # rospy.loginfo('begin grasp attempt (10s timeout) \n time:') # Careful: logging can break the code.
  # result = self.go_to_pose_goal()
  # print('test')
