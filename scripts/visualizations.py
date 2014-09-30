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
import math
import scipy

class modelVisualizer:

  def __init__(self):
    self.pub_array = rospy.Publisher('visualization_marker_array', MarkerArray)
    rospy.Subscriber("model_track", ModelMsg, self.callbackBestModels)
    rospy.Subscriber("action", ActionsMsg, self.callbackAction)
    #rospy.Subscriber("generated_actions", ActionsMsg, self.callbackGeneratedActions)
    #rospy.Subscriber("model_particles", ParticlesMsg, self.callbackParticles)


  def callbackParticles(self,particles):
    self.particle_free_counter = 0
    self.particle_prismatic_counter = 0
    self.particle_rotational_counter = 0
    self.particle_rigid_counter = 0

    (self.current_pose,current_prism_pose) = self.getCurrentPose(particles.particles[1])

    for model in particles.particles:
      if model.name == "prismatic":
        self.particle_prismatic_counter = self.particle_prismatic_counter + 1
        (prism_pose, prism_dir) = self.read_prismatic_params(model)
        self.visualizePrismatic(current_prism_pose,prism_dir,box_dims = [0.03,0.015,0.015], duration = 3, id= self.particle_prismatic_counter, arrow_dims = [0.01,0.02,0.02], arrow_offset= 0.012, arrow_length= 0.1, color_box=[1,0,0])
      if model.name == "rotational":
        self.particle_rotational_counter = self.particle_rotational_counter + 1
        (rot_pose,radius) = self.read_revolute_params(model)
        self.visualizeRotational(rot_pose,radius,ns="particle_revolute",duration = 3, id=self.particle_rotational_counter, axis_length=0.2, size=0.005)
      if model.name == "rigid":
        self.particle_rigid_counter = self.particle_rigid_counter + 1
        rigid_pose = self.read_rigid_params(model)
        self.visualizeRigid(rigid_pose,0.1, id= self.particle_rigid_counter)
      if model.name == "free":
        self.particle_free_counter = self.particle_free_counter + 1
        self.visualizeFree(self.current_pose,size = 0.02,arrow_offset=0.02, id=self.particle_free_counter, arrow_dims= [0.01,0.02,0.02], arrow_length=0.06)


  def callbackGeneratedActions(self,actions):
    zero_orientation = Quaternion(0, 0, 0, 1)
    zero_position = Point(0, 0, 0)
    zero_pose = Pose(zero_position, zero_orientation)
    (position,orientation) = poseToArray(zero_pose)
    counter = 0
    marker_array = MarkerArray()

    for pose in actions.actions:
      marker_arrow = self.create_marker(Marker.ARROW, [0.01,0.02,0.02], "ar_marker_12", "generated_actions", counter, 60, [random.random(),random.random(),random.random()], 1, position, orientation)
      counter = counter + 1
      marker_arrow.points.append( Point(0,0,0) )
      scale = 0.3
      marker_arrow.points.append( Point(pose.position.x*scale, pose.position.y*scale, pose.position.z*scale) )

      marker_array.markers.append(marker_arrow)

    self.pub_array.publish(marker_array)


  def callbackAction(self, actions_msg):

    (position,orientation) = poseToArray(actions_msg.actions[1])
    marker_arrow = self.create_marker(Marker.ARROW, [0.03,0.06,0.06], "world", "action_arrow", 0, 60, [1,1,0], 1, position, orientation)
    marker_arrow.points.append( Point(0,0,0) )
    scale = 0.3
    marker_arrow.points.append( Point(actions_msg.actions[0].position.x*scale, actions_msg.actions[0].position.y*scale, actions_msg.actions[0].position.z*scale) )

    marker_array = MarkerArray()
    marker_array.markers.append(marker_arrow)

    self.pub_array.publish(marker_array)


  def getCurrentPose(self,model):
    current_pos_x = 0
    current_pos_y = 0
    current_pos_z = 0
    current_orient_x = 0
    current_orient_y = 0
    current_orient_z = 0
    current_orient_w = 0

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

    self.current_pose = Pose(Point(current_pos_x, current_pos_y, current_pos_z), Quaternion(current_orient_x, current_orient_y, current_orient_z, current_orient_w))
    current_prism_pose = Pose(Point(current_pos_x, current_pos_y, current_pos_z), Quaternion(0,0,0,1) )
    return (self.current_pose,current_prism_pose)


  def callbackBestModels(self,model):
    (self.current_pose,current_prism_pose) = self.getCurrentPose(model)

    if model.name == "rotational":
        (rot_pose,radius) = self.read_revolute_params(model)
        #HACK: for better visualizations
        #rot_pose.orientation.x = 0.5
        #rot_pose.orientation.y = 0.5
        #rot_pose.orientation.z = 0.5
        #rot_pose.orientation.w = 0.5
        # juergen params
        #rot_pose = Pose(Point(-0.0114209492134,0.187020427431,-0.0291489658073), Quaternion(-0.127809662014,-0.0529209653851,0.397977486363,0.9069057184))
        #radius =  0.108107574867       

        self.visualizeRotational(rot_pose,radius)
    if model.name == "prismatic":
        (prism_pose, prism_dir) = self.read_prismatic_params(model)
        self.visualizePrismatic(prism_pose,prism_dir,box_dims = [0.1,0.05,0.05], arrow_dims = [0.03,0.06,0.06], arrow_offset= 0.04, arrow_length= 0.3, color_box=[1,0,0])
        #self.visualizePrismatic(current_prism_pose,prism_dir,box_dims = [0.1,0.05,0.05], arrow_dims = [0.03,0.06,0.06], arrow_offset= 0.04, arrow_length= 0.3, color_box=[1,0,1])
    if model.name == "rigid":
        rigid_pose = self.read_rigid_params(model)
        self.visualizeRigid(rigid_pose,0.1)
    if model.name == "free":
        self.visualizeFree(self.current_pose,0.1,arrow_offset=0.07, arrow_dims= [0.03,0.06,0.06], arrow_length=0.2)


    #(position,orientation) = poseToArray(self.current_pose)
    #marker_arrow = self.create_marker(Marker.ARROW, [0.03,0.06,0.06], "world", "tangent_arrow", 0, 60, [0,0,1], 1, position, orientation)
    #marker_arrow.points.append( Point(0,0,0) )
    #scale = 0.3
    #marker_arrow.points.append( Point(0,0,0.3) )
    #marker_array = MarkerArray()
    #marker_array.markers.append(marker_arrow)
    #self.pub_array.publish(marker_array)


  def read_rigid_params(self,model):
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

    rigid_pose = Pose(Point(rigid_position_x,rigid_position_y, rigid_position_z), Quaternion(rigid_orientation_x,rigid_orientation_y, rigid_orientation_z, rigid_orientation_w))
    return rigid_pose

  def read_prismatic_params(self,model):
    rigid_position_x = 0
    rigid_position_y = 0
    rigid_position_z = 0
    prismatic_dir_x = 0
    prismatic_dir_y = 0
    prismatic_dir_z = 0

    for param in model.params:
      if param.name == "rigid_position.x":
        rigid_position_x = param.value
      if param.name == "rigid_position.y":
        rigid_position_y = param.value
      if param.name == "rigid_position.z":
        rigid_position_z = param.value


      if param.name == "prismatic_dir.x":
        prismatic_dir_x = param.value
      if param.name == "prismatic_dir.y":
        prismatic_dir_y = param.value
      if param.name == "prismatic_dir.z":
        prismatic_dir_z = param.value

    prismatic_pose = Pose(Point(rigid_position_x,rigid_position_y,rigid_position_z),Quaternion(0,0,0,1))
    prismatic_dir = [prismatic_dir_x,prismatic_dir_y,prismatic_dir_z]
    return (prismatic_pose,prismatic_dir)

  def read_revolute_params(self,model):
    rot_radius = 0
    rot_center_x = 0
    rot_center_y = 0
    rot_center_z = 0

    rot_axis_x = 0
    rot_axis_y = 0
    rot_axis_z = 0
    rot_axis_w = 0

    for param in model.params:
      if param.name == "rot_radius":
        rot_radius = param.value
      if param.name == "rot_center.x":
        rot_center_x = param.value
      if param.name == "rot_center.y":
        rot_center_y = param.value
      if param.name == "rot_center.z":
        rot_center_z = param.value

      if param.name == "rot_axis.x":
        rot_axis_x = param.value
      if param.name == "rot_axis.y":
        rot_axis_y = param.value
      if param.name == "rot_axis.z":
        rot_axis_z = param.value
      if param.name == "rot_axis.w":
        rot_axis_w = param.value

    rot_center_pose = Pose(Point(rot_center_x,rot_center_y,rot_center_z),Quaternion(rot_axis_x,rot_axis_y,rot_axis_z,rot_axis_w))
    return (rot_center_pose,rot_radius)


  def create_marker(self, type, dims, frame, ns, id, duration = 60., color = [1,0,0], opaque = 0.5, pos = [0.,0.,0.], quat = [0.,0.,0.,1.], frame_locked = False):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.type = type
    marker.action = Marker.ADD
    marker.scale.x = dims[0]
    marker.scale.y = dims[1]
    marker.scale.z = dims[2]
    marker.color.a = opaque
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.lifetime = rospy.Duration(duration)
    marker.id = id
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    marker.frame_locked = frame_locked
    return marker

  def draw_rviz_line_strip(self, pose, points, frame = "world", size = .005, ns = 'line strip', id = 0, duration = 60., color = [0,1,1], opaque = 1.0, frame_locked = False):
    (position,orientation) = poseToArray(pose)
    marker = self.create_marker(Marker.LINE_STRIP, [size, 0, 0], frame, ns, id, duration, color, opaque, position, orientation, frame_locked = frame_locked)
    for point_ind in range(scipy.shape(points)[1]):
      new_point = Point()
      new_point.x = points[0, point_ind]
      new_point.y = points[1, point_ind]
      new_point.z = points[2, point_ind]
      marker.points.append(new_point)

    return marker


##draw a circle in rviz (pose z-axis is normal to the circle plane) with radius r, thickness size, and number of facets num_facets
  def draw_rviz_circle(self, pose, r, frame = "world", num_facets = 50, size = .05, ns = 'circle', id = 0, duration = 60., color = [0,0,1], opaque = 1.0, frame_locked = False):
    angle_step = math.pi*2./num_facets
    points = scipy.matrix(scipy.zeros([3, num_facets+1]))
    for (ind, angle) in enumerate(scipy.arange(0, math.pi*2.+angle_step, angle_step)):
      points[0,ind] = math.cos(angle)*r
      points[1,ind] = math.sin(angle)*r

    return self.draw_rviz_line_strip(pose, points, frame, size, ns, id, duration, color, opaque, frame_locked)



  def visualizeRotational(self, pose, radius, axis_length = 1.0, size = 0.05, frame = "world", duration = 60, color_axis = [0,0,1], color_radius = [0,0,1], opaque = 1, id = 0, ns = "revolute_model"):

    (position,orientation) = poseToArray(pose)
    #marker responsible for the axis
    dims = [size,0,0]
    marker_axis = self.create_marker(Marker.LINE_STRIP, dims, frame, ns, id, duration, color_axis, opaque, position, orientation)
    marker_axis.points.append( Point(0,0,-axis_length/2) )
    marker_axis.points.append( Point(0,0,axis_length/2) )


    #marker responsible for the circle
    ns_circle = ns + "_circle"
    marker_circle = self.draw_rviz_circle(pose,radius,size = size,id = id, color = color_radius, ns = ns_circle,duration=duration)


    marker_array = MarkerArray()
    marker_array.markers.append(marker_axis)
    marker_array.markers.append(marker_circle)



    print "publishing revolute model"
    self.pub_array.publish(marker_array)

  def visualizePrismatic(self, pose, direction, arrow_offset = 0.5, arrow_dims = [0.1,0.2,0.2], box_dims = [0.6,0.3,0.3], arrow_length = 1, frame = "world", duration = 60, color_box = [1,1,0], color_arrow = [1,0,1], opaque = 1, id = 0, ns = "prismatic_model"):

    quaternion = directionToQuaternion(direction)
    pose.orientation = Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    (position,orientation) = poseToArray(pose)

    #marker responsible for the box
    marker_box = self.create_marker(Marker.CUBE, box_dims, frame, ns, id, duration, color_box, opaque, position, orientation)

    #marker for the first arrow
    ns_arrow1=ns+"_arrow1"
    marker_arrow1 = self.create_marker(Marker.ARROW, arrow_dims, frame, ns_arrow1, id, duration, color_arrow, opaque, position, orientation)
    marker_arrow1.points.append( Point(arrow_offset,0,0) )
    marker_arrow1.points.append( Point(arrow_offset+arrow_length,0,0) )

    #marker for the second arrow
    ns_arrow2=ns+"_arrow2"
    marker_arrow2 = self.create_marker(Marker.ARROW, arrow_dims, frame, ns_arrow2, id, duration, color_arrow, opaque, position, orientation)
    marker_arrow2.points.append( Point(-arrow_offset,0,0) )
    marker_arrow2.points.append( Point(-arrow_offset-arrow_length,0,0) )

    marker_array = MarkerArray()
    marker_array.markers.append(marker_box)
    marker_array.markers.append(marker_arrow1)
    marker_array.markers.append(marker_arrow2)


    print "publishing prismatic model"
    self.pub_array.publish(marker_array)

  def visualizeFree(self, pose, size = 0.02, arrow_offset = 0.2, arrow_dims = [0.1,0.2,0.2], color_arrow = [0.5,1,0.5], arrow_length = 0.5, frame = "world", duration = 60, color = [0,1,0], opaque = 0.5, id = 0, ns = "free_model"):

    (position,orientation) = poseToArray(pose)
    dims = [size, size, size]
    #marker responsible for the sphere
    marker_sphere = self.create_marker(Marker.SPHERE, dims, frame, ns, id, duration, color, opaque, position, orientation)


    marker_array = MarkerArray()

    #arrow markers
    for i in range(3):
      marker_arrow = self.create_marker(Marker.ARROW, arrow_dims, frame, ns + "_arrow_" + str(i), id, duration, color_arrow, opaque, position, orientation)
      offset = [0,0,0]
      offset[i] = arrow_offset
      marker_arrow.points.append( Point(offset[0],offset[1],offset[2]) )
      offset[i] = arrow_offset + arrow_length
      marker_arrow.points.append( Point(offset[0],offset[1],offset[2]) )
      marker_array.markers.append(marker_arrow)

      marker_arrow = self.create_marker(Marker.ARROW, arrow_dims, frame, ns + "_arrow_opposite_" + str(i), id, duration, color_arrow, opaque, position, orientation)
      offset[i] = -arrow_offset
      marker_arrow.points.append( Point(offset[0],offset[1],offset[2]) )
      offset[i] = -arrow_offset - arrow_length
      marker_arrow.points.append( Point(offset[0],offset[1],offset[2]) )
      marker_array.markers.append(marker_arrow)


    marker_array.markers.append(marker_sphere)

    print "publishing free model"
    self.pub_array.publish(marker_array)

  def visualizeRigid(self, pose, size = 0.002, frame = "world", duration = 60, color = [1,0,0], opaque = 1, id = 0, ns = "rigid_model"):

    (position,orientation) = poseToArray(pose)
    dims = [size, size, size]
    #marker responsible for the sphere
    marker_cube = self.create_marker(Marker.CUBE, dims, frame, ns, id, duration, color, opaque, position, orientation)

    marker_array = MarkerArray()
    marker_array.markers.append(marker_cube)

    print "publishing rigid model"
    self.pub_array.publish(marker_array)




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

def poseToArray(pose):
    return [pose.position.x,pose.position.y,pose.position.z],[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

def directionToQuaternion(dir):
    random_vector = [0,-1,0]
    dir = normalize(dir)
    if abs(abs(np.vdot(dir,random_vector))-1) < 0.1:
      random_vector = [-1,0,0]
    z = np.cross(dir,random_vector)
    z = normalize(z)
    y = normalize(np.cross(z,dir))
    #print "x , y, z = ", dir, y, z
    rotation_matrix = [[dir[0],y[0],z[0],0], [dir[1],y[1],z[1],0], [dir[2],y[2],z[2],0], [0,0,0,1]]
    return normalize(tf.transformations.quaternion_from_matrix(rotation_matrix))


def main():
  try:
    rospy.init_node('model_visualizer')
    mV = modelVisualizer()
    #prismaticPose = Pose(Point(0,0,0),Quaternion(0,0,0,1))
    #rigidPose = Pose(Point(1,0,0),Quaternion(0,0,0,1))
    #freePose = Pose(Point(0,0,1),Quaternion(0,0,0,1))
    #revolutePose = Pose(Point(1,1,0),Quaternion(0.5,0.5,0.5,0.5))
    #r = rospy.Rate(1)
    #i = 0
    #color_cube =[1,0,1]
    #while not rospy.is_shutdown():
    #  mV.visualizePrismatic(prismaticPose,[0,1,1],box_dims = [0.1,0.05,0.05], arrow_dims = [0.03,0.06,0.06], arrow_offset= 0.08, arrow_length= 0.3, color_arrow=color_cube, color_box=[1,0,0])
    #  mV.visualizeRotational(revolutePose,0.5)
    #  mV.visualizeFree(freePose,0.1,arrow_offset=0.07, arrow_dims= [0.03,0.06,0.06], arrow_length=0.2)
    #  mV.visualizeRigid(rigidPose,0.1)
    #  r.sleep()
    #  i = i+1
    #  if i%2 == 0:
    #    color_cube[1] = 1
    #  else:
    #    color_cube[1] = 0
    rospy.spin()
  except rospy.ROSInterruptException: pass
      
if __name__ == '__main__':
  main()


