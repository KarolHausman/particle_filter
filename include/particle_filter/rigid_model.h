#ifndef RIGID_MODEL_H
#define RIGID_MODEL_H

#include "particle_filter/articulation_model.h"

class RigidModel : public ArticulationModel
{
public:
  RigidModel();
  virtual ~RigidModel();

  friend std::ostream& operator << (std::ostream& stream, const RigidModel& pm)
  {
    stream << "MODEL = " << pm.model << "\n";
    stream << "pos_x = " << pm.pos_x << "\n";
    stream << "pos_y = " << pm.pos_y << "\n";
    stream << "pos_z = " << pm.pos_z << "\n";
    stream << "roll = " << pm.roll << "\n";
    stream << "pitch = " << pm.pitch << "\n";
    stream << "yaw = " << pm.yaw << "\n \n";
    return stream;
  }

  double pos_x, pos_y, pos_z, roll, pitch, yaw;
protected:


};

#endif //RIGID_MODEL_H
