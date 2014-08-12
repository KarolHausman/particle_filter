#ifndef ACTION_PRISMATIC_H
#define ACTION_PRISMATIC_H
#include "particle_filter/action.h"


class ActionPrismatic : public Action
{
public:
  ActionPrismatic()
  {
    action_type = PRISMATIC_ACTION;
  }
  virtual ~ActionPrismatic(){}

  //prismatic params
  double axis_x, axis_y, axis_z;

  //rigid params
  double pos_x, pos_y, pos_z, roll, pitch, yaw;
};


#endif //ACTION_PRISMATIC_H
