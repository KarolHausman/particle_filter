#ifndef ACTION_H
#define ACTION_H
#include <tf/tf.h>


enum ActionType {RIGID_ACTION, PRISMATIC_ACTION, ROTATIONAL_ACTION};

class Action;
typedef boost::shared_ptr<Action> ActionPtr;


class Action
{
public:
  Action(){};
  virtual ~Action(){};

  tf::Transform grasp_point;
  ActionType action_type;

};


#endif //ACTION_H
