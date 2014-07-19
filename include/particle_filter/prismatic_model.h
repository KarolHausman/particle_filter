#ifndef PRISMATIC_MODEL_H
#define PRISMATIC_MODEL_H

#include "particle_filter/articulation_model.h"

class PrismaticModel : public ArticulationModel
{
public:
  PrismaticModel();
  virtual ~PrismaticModel();
  friend std::ostream& operator << (std::ostream& stream, const PrismaticModel& pm)
  {
    stream << "MODEL = " << pm.model << "\n";
    stream << "pos_x = " << pm.pos_x << "\n";
    stream << "pos_y = " << pm.pos_y << "\n";
    stream << "pos_z = " << pm.pos_z << "\n";
    stream << "roll = " << pm.roll << "\n";
    stream << "pitch = " << pm.pitch << "\n \n";
    return stream;
  }

  double pos_x, pos_y, pos_z, roll, pitch;
protected:


};

#endif //PRISMATIC_MODEL_H
