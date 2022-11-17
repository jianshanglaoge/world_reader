#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from os import getcwd
import time
import rospy
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped, Pose


# init a node as usual
rospy.init_node('service_call_TEST')

# wait for this sevice to be running
rospy.wait_for_service('/contact_graspnet/request_inference')

# Create the connection to the service. Remember it's a Trigger service
request_inference = rospy.ServiceProxy('/contact_graspnet/request_inference', Trigger)
print(request_inference.wait_for_service())
# Create an object of the type TriggerRequest. We nned a TriggerRequest for a Trigger service
# request = SetBool()

while not rospy.is_shutdown():    
  # Now send the request through the connection
  result = request_inference()
  print(result)

# Done
print('result')