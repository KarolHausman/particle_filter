#ifndef ACTION_H
#define ACTION_H
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class Action;
typedef boost::shared_ptr<Action> ActionPtr;


class Action
{
public:
  Action();
  virtual ~Action();


  bool execute(tf::Vector3 &direction, const std::string &marker_tf, const bool &both_ways = true);
  void plan(tf::Vector3 &direction, const bool &both_ways = true);

  tf::StampedTransform marker2r__wrist_roll;
  tf::TransformListener tf_listener;
  double distance;
  ros::Publisher vector_pub;
  ros::NodeHandle nh;
  moveit::planning_interface::MoveGroup group;
  ros::AsyncSpinner spinner;


};


#endif //ACTION_H
