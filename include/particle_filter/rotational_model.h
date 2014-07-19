#ifndef ROTATIONAL_MODEL_H
#define ROTATIONAL_MODEL_H

#include "particle_filter/articulation_model.h"

class RotationalModel : public ArticulationModel
{
public:
  RotationalModel();
  virtual ~RotationalModel();

  friend std::ostream& operator << (std::ostream& stream, const RotationalModel& rm)
  {
    stream << "MODEL = " << rm.model << "\n";
    stream << "rot_center_x = " << rm.rot_center_x << "\n";
    stream << "rot_center_y = " << rm.rot_center_y << "\n";
    stream << "rot_center_z = " << rm.rot_center_z << "\n";
    stream << "roll = " << rm.roll << "\n";
    stream << "pitch = " << rm.pitch << "\n";
    stream << "yaw = " << rm.yaw << "\n";
    stream << "radius = " << rm.radius << "\n";
    stream << "axis_x = " << rm.axis_x << "\n";
    stream << "axis_y = " << rm.axis_y << "\n \n";
    return stream;
  }

  double rot_center_x, rot_center_y, rot_center_z, roll, pitch, yaw, radius, axis_x, axis_y;
protected:


};

#endif //ROTATIONAL_MODEL_H
