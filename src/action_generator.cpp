#include "particle_filter/action_generator.h"

ActionGenerator::ActionGenerator():
  nh()
{
  actions_pub = nh.advertise <articulation_model_msgs::ActionsMsg>("generated_actions", 5);
}

ActionGenerator::~ActionGenerator()
{
}

std::vector<tf::Vector3> ActionGenerator::generateActionDirections (const int grid_size_horizontal, const int grid_size_vertical)
{
  ROS_INFO("vertical number = %d   ,    horizontal number = %d", grid_size_vertical, grid_size_horizontal);

  uint vertical_increment = static_cast<int> (180.0 / (double)(grid_size_vertical - 1));
  uint horizontal_increment = static_cast<int> (180.0 / (double)(grid_size_horizontal - 1));
  ROS_INFO("vertical_increment = %d   ,    horizontal_increment = %d", vertical_increment, horizontal_increment);
  std::vector<tf::Vector3> action_dirs;

//  for (int horizontal = 0 + horizontal_increment; horizontal < 180; horizontal += horizontal_increment)
//  {
//    for (int vertical = -90 + vertical_increment; vertical < 90; vertical += vertical_increment)
//    {
//      ROS_INFO("vertical = %d      ,       horizontal = %d", vertical, horizontal);
//      double x = cos(horizontal*M_PI/180)*sin(vertical*M_PI/180);
//      double y = sin(horizontal*M_PI/180)*sin(vertical*M_PI/180);
//      double z = cos(vertical*M_PI/180);
//      action_dirs.push_back(tf::Vector3(x, y, z));
//    }
//  }



//  tf::Vector3 v(1,0,1);
  tf::Vector3 u(0,1,0);
  for (int vertical = -90; vertical <= 90; vertical += vertical_increment)
  {
    double x = tan(vertical*M_PI/180);
    int z = 1;
    if (isinf(x))
    {
      x = 1;
      z = 0;
    }
    tf::Vector3 v(x,0,z);
    v.normalize();
    for (int horizontal = 0; horizontal <= 180; horizontal += horizontal_increment)
    {
      tf::Vector3 action = u*cos(horizontal*M_PI/180) + v*sin(horizontal*M_PI/180);
      action_dirs.push_back(action);
    }
  }




  actions_msg.header.stamp = ros::Time::now();
  actions_msg.header.seq = 0;
  for (std::vector<tf::Vector3>::iterator it = action_dirs.begin(); it != action_dirs.end(); it++)
  {
    ROS_INFO("generated vector: x = %f, y = %f, z = %f", it->getX(), it->getY(), it->getZ());
    geometry_msgs::Pose action_pose;
    action_pose.position.x = it->getX();
    action_pose.position.y = it->getY();
    action_pose.position.z = it->getZ();
    action_pose.orientation.w = 1;
    actions_msg.actions.push_back(action_pose);
  }
  return action_dirs;
}

void ActionGenerator::publishGenActions()
{
  actions_pub.publish(actions_msg);
}
