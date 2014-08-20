#ifndef ACTION_GENERATOR_H
#define ACTION_GENERATOR_H
#include "tf/tf.h"
#include "articulation_model_msgs/ActionsMsg.h"


class ActionGenerator
{
public:
  ActionGenerator();
  virtual ~ActionGenerator();

  std::vector<tf::Vector3> generateActionDirections (const int grid_size_horizontal, const int grid_size_vertical);
  void publishGenActions();

  ros::NodeHandle nh;
  ros::Publisher actions_pub;
  articulation_model_msgs::ActionsMsg actions_msg;

};

#endif // ACTION_GENERATOR_H
