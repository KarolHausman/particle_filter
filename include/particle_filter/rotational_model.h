#ifndef ROTATIONAL_MODEL_H
#define ROTATIONAL_MODEL_H

#include "particle_filter/articulation_model.h"

class RotationalModel : public ArticulationModel
{
public:
  RotationalModel();
  virtual ~RotationalModel();

protected:
  double rot_center_x, rot_center_y, rot_center_z, roll, pitch, yaw, radius, axis_x, axis_y;


};

#endif //ROTATIONAL_MODEL_H
