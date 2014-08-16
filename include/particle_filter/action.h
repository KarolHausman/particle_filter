#ifndef ACTION_H
#define ACTION_H
#include <tf/tf.h>


class Action;
typedef boost::shared_ptr<Action> ActionPtr;


class Action
{
public:
  Action();
  virtual ~Action();


  bool execute(const tf::Vector3 &direction, const bool &both_ways = true);
  void plan(const tf::Vector3 &direction, const bool &both_ways = true);

  tf::Transform grasp_point;
  double distance;
  ros::Publisher vector_pub;
  ros::NodeHandle nh;


};


#endif //ACTION_H
