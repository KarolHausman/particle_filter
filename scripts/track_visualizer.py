#!/usr/bin/env python

"""
usage: %(progname)s [--obs] [--tracks]
This node visualizes track messages.
"""


import rospy
import sys
from articulation_model_msgs.msg import ModelMsg
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Vector3,Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import ChannelFloat32
import logging
import getopt
import colorsys

class trackVisualizer:

  def __init__(self, colorize_track, colorize_obs):
	  self.pub = rospy.Publisher('visualization_marker', Marker)
	  self.pub_array = rospy.Publisher('visualization_marker_array', MarkerArray)
	  self.colorize_track = colorize_track
	  self.colorize_obs = colorize_obs
	  rospy.Subscriber("model_track", ModelMsg, self.callback)
	  self.num_poses = {}
	  self.num_markers = {}
	  self.old_num_markers = {}

  def callback(self,model):
    rospy.loginfo( "received track %d, containing %d poses",model.track.id,len(model.track.pose) )
    marker_array = MarkerArray()

    if model.track.id in self.num_markers:
      self.old_num_markers[model.track.id] = self.num_markers[model.track.id]
    else:
      self.old_num_markers[model.track.id] = 0
      self.num_markers[model.track.id] = 0
      self.num_poses[model.track.id] = len(model.track.pose)

    #channel_w = None
    #channel_h = None
    #for channel in track.channels:
    #if channel.name == 'width':
    #channel_w = channel
    #elif channel.name == 'height':
    #channel_h = channel


    marker_array = MarkerArray()
    self.render_points(model.track,marker_array)
    

    self.delete_old_markers(model.track,marker_array)	

    rospy.loginfo( "publishing MarkerArray, containing %d markers",
    len(marker_array.markers) )
    self.pub_array.publish(marker_array)

  def render_points(self,track,marker_array):
    for i in range( len( track.pose ) ):
      marker = Marker()
      marker.header.stamp = track.header.stamp
      marker.header.frame_id = track.header.frame_id
      marker.ns = "track_visualizer-%d"%(track.id)
      marker.id = self.num_markers[track.id]
      marker.action = Marker.ADD

      marker.scale = Vector3(0.003,0.003,0.003)
      #marker.color = self.generate_color_rectangle(track.id, i)

      marker.type = Marker.SPHERE_LIST
      marker.pose = track.pose[i]
      marker.points.append( Point(0,0,0) )

      marker_array.markers.append(marker)
      self.num_markers[track.id] += 1

  def delete_old_markers(self,track,marker_array):
    i = self.num_markers[track.id]
    while i < self.old_num_markers[track.id]:
      marker = Marker()
      marker.header.stamp = track.header.stamp
      marker.header.frame_id = track.header.frame_id
      marker.ns = "track_visualizer-%d"%(track.id)
      marker.id = i
      marker.action = Marker.DELETE
      marker_array.markers.append(marker)
      i += 1



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


