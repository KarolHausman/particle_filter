#ifndef ACTION_ROTATIONAL_H
#define ACTION_ROTATIONAL_H
#include "particle_filter/action.h"


class ActionRotational : public Action
{
public:
  ActionRotational()
  {
    action_type = ROTATIONAL_ACTION;
  }
  virtual ~ActionRotational(){}


  double rot_radius;
  double rot_center_x, rot_center_y, rot_center_z, roll, pitch, yaw, radius;
  tf::Quaternion rot_axis;
};


#endif //ACTION_PRISMATIC_H
