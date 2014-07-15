#ifndef PRISMATIC_MODEL_H
#define PRISMATIC_MODEL_H

#include "particle_filter/articulation_model.h"

class PrismaticModel : public ArticulationModel
{
public:
  PrismaticModel();
  virtual ~PrismaticModel();

protected:
  double pos_x, pos_y, pos_z, roll, pitch;


};

#endif //PRISMATIC_MODEL_H
