#ifndef ACTION_PRISMATIC_H
#define ACTION_PRISMATIC_H
#include "particle_filter/action.h"
#include "particle_filter/prismatic_model.h"

class ActionPrismatic : public Action
{
public:
  ActionPrismatic()
  {
    action_type = PRISMATIC_ACTION;
  }

  ActionPrismatic(PrismaticModel &model)
  {
    axis_x = model.axis_x;
    axis_y = model.axis_y;
    axis_z = model.axis_z;
    pos_x = model.pos_x;
    pos_y = model.pos_y;
    pos_z = model.pos_z;
    roll = model.roll;
    pitch = model.pitch;
    yaw = model.yaw;
    action_type = PRISMATIC_ACTION;
  }

  virtual ~ActionPrismatic(){}

  //prismatic params
  double axis_x, axis_y, axis_z;

  //rigid params
  double pos_x, pos_y, pos_z, roll, pitch, yaw;
};


#endif //ACTION_PRISMATIC_H
