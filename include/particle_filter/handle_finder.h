#ifndef HANDLE_FINDER_H
#define HANDLE_FINDER_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "pr2_lfd_utils/WMData.h"
#include "simple_robot_control/gripper_control.h"
#include "Eigen/Core"

class HandleFinder
{
public:
  HandleFinder();
  ~HandleFinder();

  bool findHandle(const std::string& marker_tf);
  void markerCB(const pr2_lfd_utils::WMDataConstPtr msg);

  // positive offset for the pregrasp with respect to the marker(x,y,z,r,p,y) and for the grasp (x,y,z)
  bool executeHandleGrasp(const Eigen::VectorXd &offset_pregrasp, const Eigen::VectorXd &offset_grasp);

  ros::NodeHandle nh;
  ros::Subscriber marker_sub;
  tf::TransformListener tf_listener;
  tf::StampedTransform odom2marker;
  simple_robot_control::Gripper gripper;
  ros::AsyncSpinner spinner;



};


#endif //HANDLE_FINDER_H
