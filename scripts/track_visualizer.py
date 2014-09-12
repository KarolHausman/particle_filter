#!/usr/bin/env python

"""
usage: %(progname)s [--obs] [--tracks]
This node visualizes track messages.
"""


import rospy
import sys
from articulation_model_msgs.msg import ModelMsg, ParticlesMsg, ActionsMsg
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import ChannelFloat32
import logging
import getopt
import colorsys
import tf
import numpy as np
import random

class trackVisualizer:

  def __init__(self, colorize_track, colorize_obs):
    self.pub = rospy.Publisher('visualization_marker', Marker)
    self.pub_array = rospy.Publisher('visualization_marker_array', MarkerArray)
    self.colorize_track = colorize_track
    self.colorize_obs = colorize_obs
    rospy.Subscriber("model_track", ModelMsg, self.callback)
    rospy.Subscriber("model_particles", ParticlesMsg, self.callbackParticles)
    rospy.Subscriber("action", ActionsMsg, self.callbackAction)
    rospy.Subscriber("generated_actions", ActionsMsg, self.callbackGeneratedActions)
    self.num_poses = {}
    self.num_markers = {} 
    self.old_num_markers = {}
    self.particle_counter = 0
    self.particle_prismatic_counter = 0
    self.particle_rotational_counter = 0
    self.current_pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

  def callbackAction(self, actions_msg): 
    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = "world"
    marker.ns = "model_visualizer_action"
    marker.id = 0
    marker.action = Marker.ADD

    marker.scale = Vector3(0.02,0.02,0.02)
    marker.color.a = 1
    marker.color.g = 1
    marker.color.r = 1
    
    #actions[0] - direction, actions[1] - pose
    marker.type = Marker.ARROW
    if self.current_pose.position.x != 0:
      marker.pose = self.current_pose
    else:
      marker.pose = actions_msg.actions[1]
    marker.points.append( Point(0,0,0) )
    marker.points.append( Point(actions_msg.actions[0].position.x/3, actions_msg.actions[0].position.y/3, actions_msg.actions[0].position.z/3) )

    self.pub.publish(marker)

    
    
  def callbackGeneratedActions(self,actions):
    zero_orientation = Quaternion(0, 0, 0, 1)  
    zero_position = Point(0, 0, 0)
    zero_pose = Pose(zero_position, zero_orientation)
    counter = 0
    marker_array = MarkerArray()
    for pose in actions.actions:
      marker = Marker()
      marker.header.stamp = rospy.get_rostime()
      marker.header.frame_id = "world"
      marker.ns = "model_visualizer_generated_actions"
      marker.id = counter
      counter = counter + 1
      marker.action = Marker.ADD

      marker.scale = Vector3(0.02,0.02,0.02)
      marker.color.a = 1
      marker.color.b = random.random()
      marker.color.r = random.random()
      marker.color.g = random.random()

      marker.type = Marker.ARROW
      marker.pose = zero_pose
      marker.points.append( Point(0,0,0) )
      marker.points.append( Point(pose.position.x, pose.position.y, pose.position.z) )
      
      marker_array.markers.append(marker)
  
      self.pub_array.publish(marker_array)     


  def callbackParticles(self,particles):
    marker_array = MarkerArray()
    self.particle_counter = 0
    self.particle_prismatic_counter = 0
    self.particle_rotational_counter = 0

    for model in particles.particles:
      self.particle_counter = self.particle_counter + 1
      self.render_particle(model, marker_array)

    self.pub_array.publish(marker_array)

  def callback(self,model):
    rospy.loginfo( "received track %d, containing %d poses",model.track.id,len(model.track.pose) )
    marker_array = MarkerArray()

    if model.track.id in self.num_markers:
      self.old_num_markers[model.track.id] = self.num_markers[model.track.id]
    else:
      self.old_num_markers[model.track.id] = 0
      self.num_markers[model.track.id] = 0
      self.num_poses[model.track.id] = len(model.track.pose)

    marker_array = MarkerArray()
    self.render_points(model.track,marker_array)
    
    self.render_model(model, marker_array)

    #self.delete_old_markers(model.track,marker_array)	

    rospy.loginfo( "publishing MarkerArray, containing %d markers",
    len(marker_array.markers) )
    self.pub_array.publish(marker_array)

  def render_particle(self, model, marker_array):
    if model.name == "free":
      return
    for param in model.params:
      if param.name == "rigid_position.x" or param.name == "rot_center.x":
        rigid_position_x = param.value
      if param.name == "rigid_position.y" or param.name == "rot_center.y":
        rigid_position_y = param.value 
      if param.name == "rigid_position.z" or param.name == "rot_center.z":
        rigid_position_z = param.value

      if param.name == "rigid_orientation.x" or param.name == "rot_axis.x":
        rigid_orientation_x = param.value
      if param.name == "rigid_orientation.y" or param.name == "rot_axis.y":
        rigid_orientation_y = param.value
      if param.name == "rigid_orientation.z" or param.name == "rot_axis.z":
        rigid_orientation_z = param.value
      if param.name == "rigid_orientation.w" or param.name == "rot_axis.w":
        rigid_orientation_w = param.value

      if param.name == "prismatic_dir.x":
        prismatic_dir_x = param.value
      if param.name == "prismatic_dir.y":
        prismatic_dir_y = param.value
      if param.name == "prismatic_dir.z":
        prismatic_dir_z = param.value

      if param.name == "rot_axis.x":
        rot_axis_x = param.value
      if param.name == "rot_axis.y":
        rot_axis_y = param.value
      if param.name == "rot_axis.z":
        rot_axis_z = param.value
      if param.name == "rot_axis.w":
        rot_axis_w = param.value
      
      if param.name == "rot_radius":
        rot_radius = param.value

      if param.name == "weight":
        weight = param.value
      

      if param.name == "current_pose_trans.x":
        current_pos_x = param.value
      if param.name == "current_pose_trans.y":
        current_pos_y = param.value
      if param.name == "current_pose_trans.z":
        current_pos_z = param.value

      if param.name == "current_pose_quat.x":
        current_orient_x = param.value
      if param.name == "current_pose_quat.y":
        current_orient_y = param.value
      if param.name == "current_pose_quat.z":
        current_orient_z = param.value
      if param.name == "current_pose_quat.w":
        current_orient_w = param.value


      if param.name == "current_proj_pose_trans.x":
        current_proj_pos_x = param.value
      if param.name == "current_proj_pose_trans.y":
        current_proj_pos_y = param.value
      if param.name == "current_proj_pose_trans.z":
        current_proj_pos_z = param.value

      if param.name == "current_proj_pose_quat.x":
        current_proj_orient_x = param.value
      if param.name == "current_proj_pose_quat.y":
        current_proj_orient_y = param.value
      if param.name == "current_proj_pose_quat.z":
        current_proj_orient_z = param.value
      if param.name == "current_proj_pose_quat.w":
        current_proj_orient_w = param.value

    self.current_proj_pose = Pose(Point(current_proj_pos_x, current_proj_pos_y, current_proj_pos_z), Quaternion(current_proj_orient_x, current_proj_orient_y, current_proj_orient_z, current_proj_orient_w))


    identity_pose_orientation = Quaternion(0, 0, 0, 1)                  
    rigid_pose_orientation = Quaternion(rigid_orientation_x, rigid_orientation_y, rigid_orientation_z, rigid_orientation_w)  
    rigid_pose_position = Point(rigid_position_x, rigid_position_y, rigid_position_z)
    rigid_pose = Pose(rigid_pose_position, identity_pose_orientation)
    
    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = model.track.header.frame_id
    marker.ns = "model_visualizer_particle"
    marker.id = self.particle_counter
    #marker.lifetime = rospy.Duration.from_sec(3)
    marker.action = Marker.ADD

    marker.scale = Vector3(0.007,0.007,0.007)
    for param in model.params:
      if param.name == "added":
        if param.value != 0 :
          marker.scale = Vector3(0.05,0.05,0.05)


    marker.pose = self.current_proj_pose #rigid_pose


    if model.name == "rotational":
      marker.type = Marker.SPHERE
      marker.color.a = 1
      marker.color.b = 1
      marker.pose = rigid_pose
    if model.name == "rigid":
      marker.type = Marker.CUBE
      marker.color.a = 1
      marker.color.g = 1
    if model.name == "prismatic":
      marker.type = Marker.CYLINDER
      marker.color.a = 1
      marker.color.r = 1


    marker.points.append( Point(0,0,0) )
    #marker_array.markers.append(marker)

    #marker.points.remove( Point(0,0,0) )
    #small prismatic axis
    if model.name == "prismatic":
      self.particle_prismatic_counter = self.particle_prismatic_counter + 1
      marker_prismatic = Marker()  
      marker_prismatic.header.stamp = rospy.get_rostime()
      marker_prismatic.header.frame_id = model.track.header.frame_id
      marker_prismatic.ns = "model_visualizer_particle_prismatic_axis"
      marker_prismatic.id = self.particle_prismatic_counter
      #marker_prismatic.lifetime = rospy.Duration.from_sec(3)
      marker_prismatic.action = Marker.ADD

      marker_prismatic.type = Marker.LINE_STRIP
      marker_prismatic.color.b = 1
      marker_prismatic.color.a = 1
      marker_prismatic.color.r = 1
      marker_prismatic.scale = Vector3(0.001,0.001,0.001)

      marker_prismatic.pose = self.current_proj_pose  #rigid_pose
      length_scale = 15 #30
      marker_prismatic.points.append( Point(0,0,0) )
      marker_prismatic.points.append( Point(prismatic_dir_x/length_scale, prismatic_dir_y/length_scale, prismatic_dir_z/length_scale) )
      marker_array.markers.append(marker_prismatic)

    #small rotational axis
    if model.name == "rotational":
      self.particle_rotational_counter = self.particle_rotational_counter + 1
      marker_rotational = Marker()  
      marker_rotational.header.stamp = rospy.get_rostime()
      marker_rotational.header.frame_id = model.track.header.frame_id
      marker_rotational.ns = "model_visualizer_particle_rotational_axis"
      marker_rotational.id = self.particle_rotational_counter
      #marker_rotational.lifetime = rospy.Duration.from_sec(3)
      marker_rotational.action = Marker.ADD

      marker_rotational.type = Marker.LINE_STRIP
      marker_rotational.color.b = 1
      marker_rotational.color.a = 1
      marker_rotational.color.g = 1
      marker_rotational.scale = Vector3(0.001,0.001,0.001)

      marker_rotational.pose = Pose(rigid_pose_position, Quaternion(rot_axis_x, rot_axis_y, rot_axis_z, rot_axis_w))
      marker_rotational.points.append( Point(0,0,0) )
      marker_rotational.points.append( Point(0,0,0.15) )
      marker_array.markers.append(marker_rotational)

     

  def render_model(self, model, marker_array):

    #origin
    zero_orientation = Quaternion(0, 0, 0, 1)  
    zero_position = Point(0, 0, 0)
    zero_pose = Pose(zero_position, zero_orientation)

    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = model.track.header.frame_id
    marker.ns = "model_visualizer_origin"
    marker.id = 0
    marker.action = Marker.ADD

    marker.scale = Vector3(0.01,0.01,0.01)
    marker.color.a = 1
    marker.color.b = 1

    marker.type = Marker.LINE_STRIP
    marker.pose = zero_pose
    for axis in range(3):
      marker.points.append( Point(0,0,0) )
      marker.colors.append( ColorRGBA(0,0,0,0) )
      if axis==0:
        marker.points.append( Point(0.3,0,0) )
        marker.colors.append( ColorRGBA(1,0,0,1) )
      elif axis==1:
        marker.points.append( Point(0,0.3,0) )
        marker.colors.append( ColorRGBA(0,1,0,1) )
      elif axis==2:
        marker.points.append( Point(0,0,0.3) )
        marker.colors.append( ColorRGBA(0,0,1,1) )

    #marker_array.markers.append(marker)

    


    for param in model.params:
      if param.name == "current_pose_trans.x":
        current_pos_x = param.value
      if param.name == "current_pose_trans.y":
        current_pos_y = param.value
      if param.name == "current_pose_trans.z":
        current_pos_z = param.value

      if param.name == "current_pose_quat.x":
        current_orient_x = param.value
      if param.name == "current_pose_quat.y":
        current_orient_y = param.value
      if param.name == "current_pose_quat.z":
        current_orient_z = param.value
      if param.name == "current_pose_quat.w":
        current_orient_w = param.value


      if param.name == "current_proj_pose_trans.x":
        current_proj_pos_x = param.value
      if param.name == "current_proj_pose_trans.y":
        current_proj_pos_y = param.value
      if param.name == "current_proj_pose_trans.z":
        current_proj_pos_z = param.value

      if param.name == "current_proj_pose_quat.x":
        current_proj_orient_x = param.value
      if param.name == "current_proj_pose_quat.y":
        current_proj_orient_y = param.value
      if param.name == "current_proj_pose_quat.z":
        current_proj_orient_z = param.value
      if param.name == "current_proj_pose_quat.w":
        current_proj_orient_w = param.value

    self.current_pose = Pose(Point(current_pos_x, current_pos_y, current_pos_z), Quaternion(current_orient_x, current_orient_y, current_orient_z, current_orient_w))
    self.current_proj_pose = Pose(Point(current_proj_pos_x, current_proj_pos_y, current_proj_pos_z), Quaternion(current_proj_orient_x, current_proj_orient_y, current_proj_orient_z, current_proj_orient_w))
    #self.current_proj_pose_in_current_pose = Pose(Point(current_proj_pos_x, current_proj_pos_y, current_proj_pos_z), Quaternion(current_orient_x, current_orient_y, current_orient_z, current_orient_w))
    #self.current_proj_pose = Pose(Point(current_proj_pos_x, current_proj_pos_y, current_proj_pos_z), Quaternion(0, 0, 0, 1))

    #current pose
    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = model.track.header.frame_id
    marker.ns = "model_visualizer_pose"
    marker.id = 0
    marker.action = Marker.ADD

    marker.scale = Vector3(0.01,0.01,0.01)
    marker.color.a = 1
    marker.color.b = 1
    marker.type = Marker.LINE_STRIP
    marker.pose = self.current_pose
    for axis in range(3):
      marker.points.append( Point(0,0,0) )
      marker.colors.append( ColorRGBA(0,0,0,0) )
      if axis==0:
        marker.points.append( Point(0.6,0,0) )
        marker.colors.append( ColorRGBA(1,0,0,1) )
      elif axis==1:
        marker.points.append( Point(0,0.6,0) )
        marker.colors.append( ColorRGBA(0,1,0,1) )
      elif axis==2:
        marker.points.append( Point(0,0,0.6) )
        marker.colors.append( ColorRGBA(0,0,1,1) )

    #marker_array.markers.append(marker)
   




    
    if model.name == "rotational":
        self.render_rotational_model(model, marker_array)
    if model.name == "prismatic":
        self.render_prismatic_model(model, marker_array)
    if model.name == "rigid":
        self.render_rigid_model(model, marker_array)


  def render_rigid_model(self, model, marker_array):
    rigid_position_x = 0
    rigid_position_y = 0
    rigid_position_z = 0
    rigid_orientation_x = 0
    rigid_orientation_y = 0
    rigid_orientation_z = 0
    rigid_orientation_w = 0
    
    for param in model.params:
      if param.name == "rigid_position.x":
        rigid_position_x = param.value
      if param.name == "rigid_position.y":
        rigid_position_y = param.value
      if param.name == "rigid_position.z":
        rigid_position_z = param.value

      if param.name == "rigid_orientation.x":
        rigid_orientation_x = param.value
      if param.name == "rigid_orientation.y":
        rigid_orientation_y = param.value
      if param.name == "rigid_orientation.z":
        rigid_orientation_z = param.value
      if param.name == "rigid_orientation.w":
        rigid_orientation_w = param.value

      if param.name == "weight":
        weight = param.value

    rigid_pose_orientation = Quaternion(rigid_orientation_x, rigid_orientation_y, rigid_orientation_z, rigid_orientation_w)  
    rigid_pose_position = Point(rigid_position_x, rigid_position_y, rigid_position_z)
    rigid_pose = Pose(rigid_pose_position, rigid_pose_orientation)

    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = model.track.header.frame_id
    marker.ns = "model_visualizer_rigid"
    marker.id = 0
    marker.lifetime = rospy.Duration.from_sec(20)
    marker.action = Marker.ADD

    marker.scale = Vector3(0.05,0.05,0.05)
    marker.color.a = 1
    marker.color.b = 1

    marker.type = Marker.LINE_STRIP
    marker.pose = rigid_pose
    for axis in range(3):
      marker.points.append( Point(0,0,0) )
      marker.colors.append( ColorRGBA(0,0,0,0) )
      if axis==0:
        marker.points.append( Point(0.3,0,0) )
        marker.colors.append( ColorRGBA(1,0,0,1) )
      elif axis==1:
        marker.points.append( Point(0,0.3,0) )
        marker.colors.append( ColorRGBA(0,1,0,1) )
      elif axis==2:
        marker.points.append( Point(0,0,0.3) )
        marker.colors.append( ColorRGBA(0,0,1,1) )

    marker_array.markers.append(marker)    

    marker_weight = Marker()
    marker_weight.header.stamp = rospy.get_rostime()
    marker_weight.header.frame_id = model.track.header.frame_id
    marker_weight.ns = "model_visualizer_rigid_weight"
    marker_weight.id = 0
    marker_weight.lifetime = rospy.Duration.from_sec(20)
    marker_weight.action = Marker.ADD

    if abs(weight) < 0.05:
      weight = 0.05

    marker_weight.scale = Vector3(weight,weight,weight)
    marker_weight.color.a = 0.3
    marker_weight.color.b = 1

    marker_weight.type = Marker.SPHERE
    marker_weight.pose = rigid_pose
   

    marker_array.markers.append(marker_weight) 

    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = model.track.header.frame_id
    marker.ns = "model_visualizer_rigid_proj_pose"
    marker.id = 0
    marker.action = Marker.ADD

    marker.scale = Vector3(0.04,0.04,0.04)
    marker.color.a = 1
    marker.color.g = 1
    marker.type = Marker.SPHERE
    marker.pose = self.current_proj_pose
    marker.points.append( Point(0,0,0) )

    #marker_array.markers.append(marker)

    #orientation of above(self.current_proj_pose)
    #marker = Marker()
    #marker.header.stamp = rospy.get_rostime()
    #marker.header.frame_id = model.track.header.frame_id
    #marker.ns = "model_visualizer_rigid_proj_pose_orientation"
    #marker.id = 0
    #marker.lifetime = rospy.Duration.from_sec(20)
    #marker.action = Marker.ADD

    #marker.scale = Vector3(0.05,0.05,0.05)
    #marker.color.a = 1
    #marker.color.b = 1

    #marker.type = Marker.LINE_STRIP
    #marker.pose = self.current_proj_pose
    #for axis in range(3):
    #  marker.points.append( Point(0,0,0) )
    #  marker.colors.append( ColorRGBA(0,0,0,0) )
    #  if axis==0:
    #    marker.points.append( Point(0.7,0,0) )
    #    marker.colors.append( ColorRGBA(1,0,0,1) )
    #  elif axis==1:
    #    marker.points.append( Point(0,0.7,0) )
    #    marker.colors.append( ColorRGBA(0,1,0,1) )
    #  elif axis==2:
    #    marker.points.append( Point(0,0,0.7) )
    #    marker.colors.append( ColorRGBA(0,0,1,1) )

    #marker_array.markers.append(marker)    
   


  def render_prismatic_model(self, model, marker_array):
    rigid_position_x = 0
    rigid_position_y = 0
    rigid_position_z = 0
    rigid_orientation_x = 0
    rigid_orientation_y = 0
    rigid_orientation_z = 0
    rigid_orientation_w = 0
    prismatic_dir_x = 0
    prismatic_dir_y = 0
    prismatic_dir_z = 0
    current_proj_pose_prismatic_dir_in_current_x = 0
    current_proj_pose_prismatic_dir_in_current_y = 0
    current_proj_pose_prismatic_dir_in_current_z = 0
    for param in model.params:
      if param.name == "rigid_position.x":
        rigid_position_x = param.value
      if param.name == "rigid_position.y":
        rigid_position_y = param.value
      if param.name == "rigid_position.z":
        rigid_position_z = param.value

      if param.name == "rigid_orientation.x":
        rigid_orientation_x = param.value
      if param.name == "rigid_orientation.y":
        rigid_orientation_y = param.value
      if param.name == "rigid_orientation.z":
        rigid_orientation_z = param.value
      if param.name == "rigid_orientation.w":
        rigid_orientation_w = param.value

      if param.name == "prismatic_dir.x":
        prismatic_dir_x = param.value
      if param.name == "prismatic_dir.y":
        prismatic_dir_y = param.value
      if param.name == "prismatic_dir.z":
        prismatic_dir_z = param.value

      if param.name == "current_proj_pose_prismatic_dir_in_current.x":
        current_proj_pose_prismatic_dir_in_current_x = param.value
      if param.name == "current_proj_pose_prismatic_dir_in_current.y":
        current_proj_pose_prismatic_dir_in_current_y = param.value
      if param.name == "current_proj_pose_prismatic_dir_in_current.z":
        current_proj_pose_prismatic_dir_in_current_z = param.value

      if param.name == "current_proj_pose_trans.x":
        current_proj_pos_x = param.value
      if param.name == "current_proj_pose_trans.y":
        current_proj_pos_y = param.value
      if param.name == "current_proj_pose_trans.z":
        current_proj_pos_z = param.value

      if param.name == "current_proj_pose_quat.x":
        current_proj_orient_x = param.value
      if param.name == "current_proj_pose_quat.y":
        current_proj_orient_y = param.value
      if param.name == "current_proj_pose_quat.z":
        current_proj_orient_z = param.value
      if param.name == "current_proj_pose_quat.w":
        current_proj_orient_w = param.value

      if param.name == "weight":
        weight = param.value

    self.current_proj_pose = Pose(Point(current_proj_pos_x, current_proj_pos_y, current_proj_pos_z), Quaternion(current_proj_orient_x, current_proj_orient_y, current_proj_orient_z, current_proj_orient_w))



    rigid_pose_orientation = Quaternion(rigid_orientation_x, rigid_orientation_y, rigid_orientation_z, rigid_orientation_w)  
    identity_pose_orientation = Quaternion(0, 0, 0, 1)                  
    rigid_pose_position = Point(rigid_position_x, rigid_position_y, rigid_position_z)
    rigid_pose = Pose(rigid_pose_position, identity_pose_orientation)
    scale = 3
    prismatic_dir = Point(prismatic_dir_x/scale, prismatic_dir_y/scale, prismatic_dir_z/scale)

    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = model.track.header.frame_id
    marker.ns = "model_visualizer_prismatic"
    marker.id = 0
    marker.lifetime = rospy.Duration.from_sec(20)
    marker.action = Marker.ADD

    marker.scale = Vector3(0.01,0.01,0.01)
    marker.color.a = 1
    marker.color.b = 1

    marker.type = Marker.LINE_STRIP
    marker.pose = rigid_pose
    for axis in range(3):
      marker.points.append( Point(0,0,0) )
      marker.colors.append( ColorRGBA(0,0,0,0) )
      if axis==0:
        marker.points.append( Point(0.3,0,0) )
        marker.colors.append( ColorRGBA(1,0,0,1) )
      elif axis==1:
        marker.points.append( Point(0,0.3,0) )
        marker.colors.append( ColorRGBA(0,1,0,1) )
      elif axis==2:
        marker.points.append( Point(0,0,0.3) )
        marker.colors.append( ColorRGBA(0,0,1,1) )

    #marker_array.markers.append(marker)

    marker_weight = Marker()
    marker_weight.header.stamp = rospy.get_rostime()
    marker_weight.header.frame_id = model.track.header.frame_id
    marker_weight.ns = "model_visualizer_prismatic_weight"
    marker_weight.id = 0
    #marker_weight.lifetime = rospy.Duration.from_sec(20)
    marker_weight.action = Marker.ADD

    if abs(weight) < 0.05:
      weight = 0.05

    marker_weight.scale = Vector3(weight,weight,weight)
    marker_weight.color.a = 0.3
    marker_weight.color.b = 1

    marker_weight.type = Marker.SPHERE
    marker_weight.pose = self.current_proj_pose #rigid_pose
   

    marker_array.markers.append(marker_weight)


    #direction
    marker_dir = Marker()
    marker_dir.header.stamp = rospy.get_rostime()
    marker_dir.header.frame_id = model.track.header.frame_id
    marker_dir.ns = "model_visualizer_prismatic_dir"
    marker_dir.id = 0
    #marker_dir.lifetime = rospy.Duration.from_sec(20)
    marker_dir.action = Marker.ADD

    marker_dir.scale = Vector3(0.01,0.01,0.01)
    marker_dir.color.a = 1
    marker_dir.color.r = 1
    marker_dir.color.b = 1

    marker_dir.type = Marker.LINE_STRIP
    marker_dir.pose = self.current_proj_pose#rigid_pose
    marker_dir.points.append( Point(0,0,0) )
    marker_dir.points.append( prismatic_dir )
    
    marker_array.markers.append(marker_dir)

    #marker orientation
    marker_orient = Marker()
    marker_orient.header.stamp = rospy.get_rostime()
    marker_orient.header.frame_id = model.track.header.frame_id
    marker_orient.ns = "model_visualizer_prismatic_orientation"
    marker_orient.id = 0
    marker_orient.lifetime = rospy.Duration.from_sec(20)
    marker_orient.action = Marker.ADD

    marker_orient.scale = Vector3(0.01,0.01,0.01)
    marker_orient.color.a = 1
    marker_orient.color.b = 1

    marker_orient.type = Marker.LINE_STRIP
    marker_pose = Pose( Point(rigid_pose_position.x + prismatic_dir.x, rigid_pose_position.y + prismatic_dir.y, rigid_pose_position.z + prismatic_dir.z), rigid_pose_orientation)

    marker_orient.pose = marker_pose
    for axis in range(3):
      marker_orient.points.append( Point(0,0,0) )
      marker_orient.colors.append( ColorRGBA(0,0,0,0) )
      if axis==0:
        marker_orient.points.append( Point(0.3,0,0) )
        marker_orient.colors.append( ColorRGBA(1,0,0,1) )
      elif axis==1:
        marker_orient.points.append( Point(0,0.3,0) )
        marker_orient.colors.append( ColorRGBA(0,1,0,1) )
      elif axis==2:
        marker_orient.points.append( Point(0,0,0.3) )
        marker_orient.colors.append( ColorRGBA(0,0,1,1) )
    
    #marker_array.markers.append(marker_orient)

    #marker = Marker()
    #marker.header.stamp = rospy.get_rostime()
    #marker.header.frame_id = model.track.header.frame_id
    #marker.ns = "model_visualizer_prismatic_proj_pose"
    #marker.id = 0
    #marker.action = Marker.ADD

    #marker.scale = Vector3(0.02,0.02,0.02)
    #marker.color.a = 1
    #marker.color.b = 1
    #marker.color.r = 1
    #marker.type = Marker.ARROW
    #current_prismatic_direction_pose = Pose(Point(self.current_proj_pose.position.x, self.current_proj_pose.position.y, self.current_proj_pose.position.z),identity_pose_orientation)
    #marker.pose = current_prismatic_direction_pose#self.current_proj_pose
    #print "self.current_proj_pose", self.current_proj_pose
    #marker.points.append( Point(0,0,0) )
    #prismatic_dir_normalized = normalize((prismatic_dir.x,prismatic_dir.y,prismatic_dir.z))
    #print "prismatic dir normalized", prismatic_dir_normalized
    #marker.points.append( Point(prismatic_dir_normalized[0],prismatic_dir_normalized[1],prismatic_dir_normalized[2]) )

    #marker_array.markers.append(marker)  


    marker_additional = Marker()
    marker_additional.header.stamp = rospy.get_rostime()
    marker_additional.header.frame_id = model.track.header.frame_id
    marker_additional.ns = "model_visualizer_prismatic_proj_pose_in_current"
    marker_additional.id = 0
    marker_additional.action = Marker.ADD

    marker_additional.scale = Vector3(0.02,0.02,0.02)
    marker_additional.color.a = 1
    marker_additional.color.b = 1
    marker_additional.color.r = 1
    marker_additional.type = Marker.LINE_STRIP
    marker_additional.pose = self.current_pose
    marker_additional.points.append( Point(0,0,0) )
    prismatic_dir_normalized = normalize((current_proj_pose_prismatic_dir_in_current_x,current_proj_pose_prismatic_dir_in_current_y,current_proj_pose_prismatic_dir_in_current_z))
    #print "prismatic dir normalized", prismatic_dir_normalized
    print "tangent prismatic: ", (current_proj_pose_prismatic_dir_in_current_x,current_proj_pose_prismatic_dir_in_current_y,current_proj_pose_prismatic_dir_in_current_z)
    marker_additional.points.append( Point(prismatic_dir_normalized[0],prismatic_dir_normalized[1],prismatic_dir_normalized[2]) )

    #marker_array.markers.append(marker_additional) 


  def render_rotational_model(self, model, marker_array):
    rot_radius = 0
    rot_center_x = 0
    rot_center_y = 0
    rot_center_z = 0
    rot_orientation_x = 0
    rot_orientation_y = 0
    rot_orientation_z = 0
    rot_orientation_w = 0
    rot_axis_x = 0
    rot_axis_y = 0
    rot_axis_z = 0
    rot_axis_w = 0
    current_proj_pose_rot_dir_x = 0
    current_proj_pose_rot_dir_y = 0
    current_proj_pose_rot_dir_z = 0

    for param in model.params:
      if param.name == "rot_radius":
        rot_radius = param.value
      if param.name == "rot_center.x":
        rot_center_x = param.value
      if param.name == "rot_center.y":
        rot_center_y = param.value
      if param.name == "rot_center.z":
        rot_center_z = param.value

      if param.name == "rot_orientation.x":
        rot_orientation_x = param.value
      if param.name == "rot_orientation.y":
        rot_orientation_y = param.value
      if param.name == "rot_orientation.z":
        rot_orientation_z = param.value
      if param.name == "rot_orientation.w":
        rot_orientation_w = param.value

      if param.name == "rot_axis.x":
        rot_axis_x = param.value
      if param.name == "rot_axis.y":
        rot_axis_y = param.value
      if param.name == "rot_axis.z":
        rot_axis_z = param.value
      if param.name == "rot_axis.w":
        rot_axis_w = param.value

      if param.name == "weight":
        weight = param.value
      if param.name == "current_proj_pose_rot_dir.x":
        current_proj_pose_rot_dir_x = param.value
      if param.name == "current_proj_pose_rot_dir.y":
        current_proj_pose_rot_dir_y = param.value
      if param.name == "current_proj_pose_rot_dir.z":
        current_proj_pose_rot_dir_z = param.value

    current_proj_pose_normalized = normalize((current_proj_pose_rot_dir_x, current_proj_pose_rot_dir_y, current_proj_pose_rot_dir_z))    
    current_proj_pose_rot_dir = Point(current_proj_pose_normalized[0], current_proj_pose_normalized[1], current_proj_pose_normalized[2])
    #print "current_proj_pose_rot_dir: ", current_proj_pose_rot_dir

    #rotational axis
    rot_axis = Quaternion(rot_axis_x, rot_axis_y, rot_axis_z, rot_axis_w) 
    rot_center_position = Point(rot_center_x, rot_center_y, rot_center_z)
    rot_center_orientation = Quaternion(rot_orientation_x, rot_orientation_y, rot_orientation_z, rot_orientation_w)
    identity_pose_orientation = Quaternion(0, 0, 0, 1)     

    #setting yaw of rot_axis to 0 because it's a configuration space param
    quaternion = (rot_axis.x, rot_axis.y, rot_axis.z, rot_axis.w)

    #making quaternions canonical ---> doesnt work
    #quaternion = makeQuaternionCanonical(quaternion)
    #rot_center_orientation_array = makeQuaternionCanonical([rot_orientation_x, rot_orientation_y, rot_orientation_z, rot_orientation_w])
    #rot_center_orientation = Quaternion(rot_center_orientation_array[0], rot_center_orientation_array[1], rot_center_orientation_array[2], rot_center_orientation_array[3])

    matrix = tf.transformations.quaternion_matrix(quaternion)

    z = normalize(matrix[0:3,2])
    z = abs(z)    #HACK: for better visualization - otherwise z keeps changing its sign

    random_vector = normalize([0,-1,0])
    if abs(abs(np.vdot(z,random_vector))-1) < 0.1:
      random_vector = normalize([-1,0,0])
    x = np.cross(z,random_vector)    
#    another way of doing this, but sign change problem
#    x = ([1,0, (-z[0]*1 -z[1]*0)/z[2] ])
    x = normalize(x)
    y = normalize(-np.cross(x,z))

    matrix_no_yaw = [[x[0],y[0],z[0],0], [x[1],y[1],z[1],0], [x[2],y[2],z[2],0], [0,0,0,1]]
    quaternion_no_yaw = normalize(tf.transformations.quaternion_from_matrix(matrix_no_yaw))

#uncomment if it's rotating all the time
    #rot_axis.x = quaternion_no_yaw[0]
    #rot_axis.y = quaternion_no_yaw[1]
    #rot_axis.z = quaternion_no_yaw[2]
    #rot_axis.w = quaternion_no_yaw[3]
                       
    rot_center = Pose(rot_center_position, rot_axis) 
        
    marker_rot = Marker()
    marker_rot.header.stamp = rospy.get_rostime()
    marker_rot.header.frame_id = model.track.header.frame_id
    marker_rot.ns = "model_visualizer_rotational"
    marker_rot.id = 0
    #marker_rot.lifetime = rospy.Duration.from_sec(20)
    marker_rot.action = Marker.ADD

    marker_rot.scale = Vector3(0.01,0.01,0.01)
    marker_rot.color.a = 1
    marker_rot.color.b = 1

    marker_rot.type = Marker.LINE_STRIP
    marker_rot.pose = rot_center

        
    for axis in range(3):
      marker_rot.points.append( Point(0,0,0) )
      marker_rot.colors.append( ColorRGBA(0,0,0,0) )
      if axis==0:
        marker_rot.points.append( Point(0.0,0,0) ) # 1,0,0
        marker_rot.colors.append( ColorRGBA(1,0,0,0) )
      elif axis==1:
        marker_rot.points.append( Point(0,0,0) ) # 0,1,0
        marker_rot.colors.append( ColorRGBA(0,1,0,0) )
      elif axis==2:
        marker_rot.points.append( Point(0,0,0.6) ) 
        marker_rot.colors.append( ColorRGBA(0,0,1,0) )

    marker_array.markers.append(marker_rot)


    #weight
    marker_weight = Marker()
    marker_weight.header.stamp = rospy.get_rostime()
    marker_weight.header.frame_id = model.track.header.frame_id
    marker_weight.ns = "model_visualizer_rotational_weight"
    marker_weight.id = 0
    marker_weight.lifetime = rospy.Duration.from_sec(20)
    marker_weight.action = Marker.ADD
    if abs(weight) < 0.05:
      weight = 0.05


    marker_weight.scale = Vector3(weight,weight,weight)
    marker_weight.color.a = 0.3
    marker_weight.color.b = 1

    marker_weight.type = Marker.SPHERE
    marker_weight.pose = rot_center
   

    marker_array.markers.append(marker_weight)



    #adding radius
    marker_radius = Marker()
    marker_radius.header.stamp = rospy.get_rostime()
    marker_radius.header.frame_id = model.track.header.frame_id
    marker_radius.ns = "model_visualizer_rotational_radius"
    marker_radius.id = 0
    #marker_radius.lifetime = rospy.Duration.from_sec(20)
    marker_radius.action = Marker.ADD

    marker_radius.scale = Vector3(0.01,0.01,0.01)
    marker_radius.color.a = 1
    marker_radius.color.r = 1

    marker_radius.type = Marker.LINE_STRIP
    marker_radius.pose = Pose(rot_center_position, Quaternion(0,0,0,1))  #rot_center 
    marker_radius.points.append( Point(0,0,0) )
    marker_radius.points.append( Point(0,rot_radius,0) )#Point(rot_radius,0,0) )
    
    marker_array.markers.append(marker_radius)

    #marker orientation
    marker_rot_orient = Marker()
    marker_rot_orient.header.stamp = rospy.get_rostime()
    marker_rot_orient.header.frame_id = model.track.header.frame_id
    marker_rot_orient.ns = "model_visualizer__rotational_orientation"
    marker_rot_orient.id = 0
    marker_rot_orient.lifetime = rospy.Duration.from_sec(20)
    marker_rot_orient.action = Marker.ADD

    marker_rot_orient.scale = Vector3(0.01,0.01,0.01)
    marker_rot_orient.color.a = 1
    marker_rot_orient.color.r = 1
    marker_rot_orient.type = Marker.LINE_STRIP
    
    #orientation is with respect to rot_axis
#uncomment if its jumping
    #transform_no_yaw = [[matrix_no_yaw[0][0],matrix_no_yaw[0][1],matrix_no_yaw[0][2],rot_center.position.x],[matrix_no_yaw[1][0], matrix_no_yaw[1][1], matrix_no_yaw[1][2],rot_center.position.y], [matrix_no_yaw[2][0], matrix_no_yaw[2][1], matrix_no_yaw[2][2],rot_center.position.z],[0,0,0,1] ]

    transform_no_yaw = [[matrix[0][0],matrix[0][1],matrix[0][2],rot_center.position.x],[matrix[1][0], matrix[1][1], matrix[1][2],rot_center.position.y], [matrix[2][0], matrix[2][1], matrix[2][2],rot_center.position.z],[0,0,0,1] ]

    transform_radius = [[1,0,0,rot_radius],[0,1,0,0],[0,0,1,0],[0,0,0,1]]

    pose = np.dot(transform_no_yaw,transform_radius)

    quaternion_orient = (rot_orientation_x, rot_orientation_y, rot_orientation_z, rot_orientation_w)
    rot_orientation_matrix = tf.transformations.quaternion_matrix(quaternion_orient)
    
#uncomment if its jumping
    #rot_matrix = np.dot(matrix_no_yaw,rot_orientation_matrix)

    rot_matrix = np.dot(matrix,rot_orientation_matrix)

    rot_center_quat = tf.transformations.quaternion_from_matrix(rot_matrix)
    rot_center_orientation.x = rot_center_quat[0]
    rot_center_orientation.y = rot_center_quat[1]
    rot_center_orientation.z = rot_center_quat[2]
    rot_center_orientation.w = rot_center_quat[3]

    marker_rot_orient.pose = Pose( Point(pose[0][3], pose[1][3], pose[2][3]), rot_center_orientation )
    #marker_rot_orient.pose = Pose( Point(rot_center.position.x + rot_radius, rot_center.position.y, rot_center.position.z), rot_center_orientation )

    for axis in range(3):
      marker_rot_orient.points.append( Point(0,0,0) )
      marker_rot_orient.colors.append( ColorRGBA(0,0,0,0) )
      if axis==0:
        marker_rot_orient.points.append( Point(0.3,0,0) )
        marker_rot_orient.colors.append( ColorRGBA(1,0,0,0) )
      elif axis==1:
        marker_rot_orient.points.append( Point(0,0.3,0) )
        marker_rot_orient.colors.append( ColorRGBA(0,1,0,0) )
      elif axis==2:
        marker_rot_orient.points.append( Point(0,0,0.3) )
        marker_rot_orient.colors.append( ColorRGBA(0,0,1,0) )

    #marker_array.markers.append(marker_rot_orient)

    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = model.track.header.frame_id
    marker.ns = "model_visualizer_rotational_proj_pose"
    marker.id = 0
    marker.action = Marker.ADD

    marker.scale = Vector3(0.02,0.02,0.02)
    marker.color.a = 1
    marker.color.b = 1
    marker.type = Marker.ARROW
    marker.pose = self.current_pose
    marker.points.append( Point(0,0,0) )
    marker.points.append( current_proj_pose_rot_dir )
    #print "current rotational tangent vector: ", current_proj_pose_rot_dir

    #marker_array.markers.append(marker)   
      

  def render_points(self, track, marker_array, orientation = True):
    for i in range( len( track.pose ) ):
      marker = Marker()
      marker.header.stamp = rospy.get_rostime()
      marker.header.frame_id = track.header.frame_id
      marker.ns = "track_visualizer-%d"%(track.id)
      marker.id = self.num_markers[track.id]
      marker.action = marker.ADD
      marker.lifetime = rospy.Duration.from_sec(14)

      marker.scale = Vector3(0.006,0.006,0.006)
      marker.color.g = 1
      marker.color.a = 1

      marker.pose = track.pose[i]

      if orientation:
        marker.type = Marker.LINE_STRIP
        for axis in range(3):
          marker.points.append( Point(0,0,0) )
          marker.colors.append( ColorRGBA(0,0,0,0) )
          if axis==0:
            marker.points.append( Point(0.08,0,0) )
            marker.colors.append( ColorRGBA(1,0,0,0) )
          elif axis==1:
            marker.points.append( Point(0,0.08,0) )
            marker.colors.append( ColorRGBA(0,1,0,0) )
          elif axis==2:
            marker.points.append( Point(0,0,0.08) )
            marker.colors.append( ColorRGBA(0,0,1,0) )
      else:    
        marker.type = Marker.SPHERE_LIST
        marker.points.append( Point(0,0,0) )
        marker.colors.append( ColorRGBA(0,1,0,1) )

      marker_array.markers.append(marker)
      self.num_markers[track.id] += 1

  def delete_old_markers(self,track,marker_array):
    i = self.num_markers[track.id]
    while i < self.old_num_markers[track.id]:
      marker = Marker()
      marker.header.stamp = rospy.get_rostime()
      marker.header.frame_id = track.header.frame_id
      marker.ns = "track_visualizer-%d"%(track.id)
      marker.id = i
      marker.action = Marker.DELETE
      marker_array.markers.append(marker)
      i += 1



def normalize(array):
  """ 
  Normalize a 4 element array/list/numpy.array for use as a quaternion

  :param quat_array: 4 element list/array
  :returns: normalized array
  :rtype: numpy array

  """
  quat = np.array(array)
  return quat / np.sqrt(np.dot(quat, quat))

def makeQuaternionCanonical(quat):
    if quat[0] < 0:
        quat = [quat[i]*-1 for i in range(4)]
    elif quat[0] == 0:
        if quat[1] < 0:
            quat = [quat[i]*-1 for i in range(4)]
            
    return quat

def main():
  colorize_track = False 
  colorize_obs = False
  try:
    rospy.init_node('track_visualizer')
    trackVisualizer(colorize_track,colorize_obs)
    rospy.spin()
  except rospy.ROSInterruptException: pass
      
if __name__ == '__main__':
  main()


