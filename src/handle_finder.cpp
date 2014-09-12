#include "particle_filter/handle_finder.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

HandleFinder::HandleFinder():
  nh(),gripper('r'),spinner(1)
{
  marker_sub = nh.subscribe("ar_world_model", 100, &HandleFinder::markerCB, this);
  sleep(5.0);
}

HandleFinder::~HandleFinder()
{

}

bool HandleFinder::findHandle(const std::string& marker_tf)
{
  while (nh.ok() && odom2marker.stamp_.isZero())
  {
    try
    {
      tf_listener.lookupTransform("/odom_combined", marker_tf, ros::Time(0), odom2marker);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }
  }
  return true;
}

void HandleFinder::markerCB(const pr2_lfd_utils::WMDataConstPtr msg)
{

}

bool HandleFinder::executeHandleGrasp(const Eigen::VectorXd& offset_pregrasp, const Eigen::VectorXd& offset_grasp)
{
  spinner.start();
  moveit::planning_interface::MoveGroup group("right_arm");
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  geometry_msgs::Pose handle_pose;
  tf::poseTFToMsg(odom2marker, handle_pose);

  handle_pose.position.x += offset_pregrasp(0);
  handle_pose.position.y += offset_pregrasp(1);
  handle_pose.position.z += offset_pregrasp(2);
  ROS_INFO_STREAM ("handle pose: \n" << "x = " << handle_pose.position.x << ", y = " << handle_pose.position.y << " ,z = " << handle_pose.position.z);


  tf::Quaternion quat(0,0,0,1);
  quat.setRPY(offset_pregrasp(3), offset_pregrasp(4), offset_pregrasp(5));
  handle_pose.orientation.x = quat.getX();
  handle_pose.orientation.y = quat.getY();
  handle_pose.orientation.z = quat.getZ();
  handle_pose.orientation.w = quat.getW();

  group.setPoseTarget(handle_pose);

//  moveit::planning_interface::MoveGroup::Plan my_plan;
//  bool success = group.plan(my_plan);
  bool open_success = gripper.open();
  bool pre_grasp_success = group.move();

  handle_pose.position.x += offset_grasp(0);
  handle_pose.position.y += offset_grasp(1);
  handle_pose.position.z += offset_grasp(2);
  group.setPoseTarget(handle_pose);
  bool grasp_success = group.move();

  bool close_success = gripper.close();
  spinner.stop();


  return close_success && open_success && pre_grasp_success && grasp_success;
}
