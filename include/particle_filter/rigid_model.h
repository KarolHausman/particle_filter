#ifndef RIGID_MODEL_H
#define RIGID_MODEL_H

#include "particle_filter/articulation_model.h"

class RigidModel : public ArticulationModel
{
public:
  RigidModel();
  virtual ~RigidModel();

protected:
  double pos_x, pos_y, pos_z, roll, pitch, yaw;


};

#endif //RIGID_MODEL_H
