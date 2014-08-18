#include "particle_filter/action.h"


Action::Action():
  distance(0.02),nh(),group("right_arm"),spinner(1),stopped(false)
{
  vector_pub = nh.advertise <geometry_msgs::Pose>("action", 5);
  effort_exceeded_sub = nh.subscribe ("/r_arm_controller/effort_exceeded", 1, &Action::effortCB, this);
}

Action::~Action()
{
}

void Action::effortCB(const std_msgs::BoolConstPtr& msg)
{
  stopped = msg->data;
  ROS_ERROR("effortCB called with %d", stopped);
}

int Action::getActionResult()
{
  return static_cast<int>(!stopped);
}

bool Action::execute(tf::Vector3& direction, const std::string& marker_tf, const bool& both_ways)
{
  stopped = false;
  action_direction = direction;
  spinner.start();
  plan(direction, both_ways);

  while (nh.ok() && marker2odom.stamp_.isZero())
  {
    try
    {
      tf_listener.lookupTransform(marker_tf, "/odom_combined", ros::Time(0), marker2odom);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  direction = (direction * marker2odom.getBasis());

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
