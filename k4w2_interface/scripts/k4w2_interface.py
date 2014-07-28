#!/usr/bin/env python

"""
    Copyright (c) 2014 Francisco Suarez-Ruiz.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
# UDP staff
import socket, time
# Orientation staff
import numpy as np
from math import sin, cos, acos, atan2
import PyKDL as KDL
from tf_conversions import posemath
# Messages
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped
from k4w2_msgs.msg import SkeletonRaw, SkeletonState, RPY

sr = SkeletonRaw()

class K4W2Interface():
  node_name = 'K4W2 Interface'
  def __init__(self):
    rospy.on_shutdown(self.shutdown)
    rospy.loginfo("Initializing %s Node" % self.node_name)
    # Read from the parameter server
    self.publish_rate = rospy.get_param('~publish_rate', 60)
    # UDP
    self.read_port = int(self.read_parameter('~read_port', 6565))
    # Set up read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind(('', self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))
    # Set up a publishers
    skeleton = rospy.Publisher('kinect/skeleton', SkeletonState)
    
    # TODO: Extra publisher for debugging
    pose_pub = rospy.Publisher('kinect/pose_test', PoseStamped)
    l_shoulder_pub = rospy.Publisher('/kinect/l_shoulder', RPY)
    l_elbow_pub = rospy.Publisher('/kinect/l_elbow', RPY)
    
    while not rospy.is_shutdown():
      raw_msg = SkeletonRaw()
      skeleton_msg = SkeletonState()
      skeleton_msg.header.frame_id = 'base'
      data = self.recv_timeout()
      if data:
        raw_msg.deserialize(data)
        skeleton_msg.user_number = raw_msg.user_number
        skeleton_msg.joint_type = raw_msg.joint_type
        skeleton_msg.tracking_state = raw_msg.tracking_state
        skeleton_msg.l_hand_state = raw_msg.l_hand_state
        skeleton_msg.r_hand_state = raw_msg.r_hand_state
        # Append poses
        for i in raw_msg.joint_type:
          pose = Pose()
          pose.position = Point(raw_msg.position_x[i], raw_msg.position_y[i], raw_msg.position_z[i])
          pose.orientation = Quaternion(raw_msg.orientation_x[i], raw_msg.orientation_y[i], raw_msg.orientation_z[i], raw_msg.orientation_w[i])
          skeleton_msg.pose.append(pose)
        skeleton_msg.header.stamp = rospy.Time.now()
        # Publish skeleton state
        skeleton.publish(skeleton_msg)
        
        # TODO: Extra publisher for debugging
        xr = skeleton_msg.pose[sr.ElbowLeft].position.x - skeleton_msg.pose[sr.ShoulderLeft].position.x
        yr = skeleton_msg.pose[sr.ElbowLeft].position.y - skeleton_msg.pose[sr.ShoulderLeft].position.y
        zr = skeleton_msg.pose[sr.ElbowLeft].position.z - skeleton_msg.pose[sr.ShoulderLeft].position.z
        xe = skeleton_msg.pose[sr.WristLeft].position.x - skeleton_msg.pose[sr.ElbowLeft].position.x
        ye = skeleton_msg.pose[sr.WristLeft].position.y - skeleton_msg.pose[sr.ElbowLeft].position.y
        ze = skeleton_msg.pose[sr.WristLeft].position.z - skeleton_msg.pose[sr.ElbowLeft].position.z
        a = KDL.Vector(xe, ye, ze)
        b = KDL.Vector(xr, yr, zr)
        #~ angle = acos(KDL.dot(a, b) / (a.Norm() * b.Norm()))
        angle = 2 * atan2((a * b.Norm() - a.Norm() * b).Norm(), (a * b.Norm() + a.Norm() * b).Norm())
        axis = (a * b) / (a.Norm() * b.Norm())
        w = cos(angle / 2)
        v = sin(angle / 2) * axis
        rotation = KDL.Rotation.Rot(axis, angle)
        pose_msg = PoseStamped()
        pose_msg.header = skeleton_msg.header
        pose_msg.pose.orientation = Quaternion(v.x(), v.y(), v.z(), w)
        pose_pub.publish(pose_msg)
        l_shoulder_pub.publish(RPY(*rotation.GetRPY()))
        

  def recv_timeout(self, timeout=0.01):
    self.read_socket.setblocking(0)
    total_data=[]
    data=''
    begin=time.time()
    while 1:
      #if you got some data, then timeout break 
      if total_data and time.time()-begin>timeout:
        break
      #if you got no data at all, wait a little longer
      elif time.time()-begin>timeout*2:
        break
      try:
        data=self.read_socket.recv(8192)
        if data:
          total_data.append(data)
          begin=time.time()
      except:
        pass
    return ''.join(total_data)

  def shutdown(self):
    rospy.loginfo("Shutting down %s Node" % self.node_name)
  
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

if __name__ == '__main__':
  rospy.init_node('k4w2_interface')
  try:
    K4W2Interface()
  except rospy.ROSInterruptException:
    pass
