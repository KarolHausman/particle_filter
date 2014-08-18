#include "particle_filter/action.h"


Action::Action():
  distance(0.02),nh(),group("right_arm"),spinner(1)
{
  vector_pub = nh.advertise <geometry_msgs::Pose>("action", 5);
}

Action::~Action()
{
}

bool Action::execute(tf::Vector3& direction, const std::string& marker_tf, const bool& both_ways)
{
  action_direction = direction;
  spinner.start();
  plan(direction, both_ways);

  while (nh.ok() && marker2r__wrist_roll.stamp_.isZero())
  {
    try
    {
      tf_listener.lookupTransform(marker_tf, "/r_wrist_roll_link", ros::Time(0), marker2r__wrist_roll);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  direction = (direction * marker2r__wrist_roll.getBasis());
//  double yaw,pitch,roll;
//  marker2r__wrist_roll.getBasis().getEulerYPR(yaw,pitch,roll);
//  std::cerr << "ANGLES: roll = " << roll << ", pitch = " << pitch << ", yaw = " << yaw << std::endl;
//  std::cerr << "AFTER: direction x = " << direction.getX() << ", y = " << direction.getY() << ", z = " << direction.getZ() << std::endl;
  direction.normalize();
//  std::cerr << "AFTER NORMALIZING: direction x = " << direction.getX() << ", y = " << direction.getY() << ", z = " << direction.getZ() << std::endl;

//  moveit::planning_interface::MoveGroup group("right_arm");
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
  geometry_msgs::PoseStamped pull_pose = current_pose;
  pull_pose.pose.position.x += direction.getX()*distance;
  pull_pose.pose.position.y += direction.getY()*distance;
  pull_pose.pose.position.z += direction.getZ()*distance;


  group.setPoseTarget(pull_pose);
//  moveit::planning_interface::MoveGroup::Plan my_plan;
//  bool success = group.plan(my_plan);
  bool success = group.move();
  sleep(3.0);

  group.setPoseTarget(current_pose);
  bool back_success = group.move();

  if (both_ways)
  {
    geometry_msgs::PoseStamped push_pose = current_pose;
    push_pose.pose.position.x -= direction.getX()*distance;
    push_pose.pose.position.y -= direction.getY()*distance;
    push_pose.pose.position.z -= direction.getZ()*distance;

    group.setPoseTarget(push_pose);
    group.move();

    group.setPoseTarget(current_pose);
    group.move();
  }
  spinner.stop();
  return success && back_success;
}

void Action::plan(tf::Vector3& direction, const bool& both_ways)
{
  action_direction = direction;
  geometry_msgs::Pose pose_direction;
  tf::Transform transform(tf::Quaternion(0, 0, 0, 1), direction);
  tf::poseTFToMsg(transform, pose_direction);
  vector_pub.publish(pose_direction);
}
