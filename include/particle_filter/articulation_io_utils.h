#ifndef ARTICULATION_IO_UTILS
#define ARTICULATION_IO_UTILS
#include "particle_filter/rigid_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rotational_model.h"
#include "particle_filter/articulation_model.h"


std::ostream& operator << (std::ostream& stream, const ArticulationModel& am)
{
  stream << "model= " << am.model << "\n";
  return stream;
}

std::ostream& operator << (std::ostream& stream, const PrismaticModel& pm)
{
  stream << "MODEL = " << pm.model << "\n";
  stream << "pos_x = " << pm.pos_x << "\n";
  stream << "pos_y = " << pm.pos_y << "\n";
  stream << "pos_z = " << pm.pos_z << "\n";
  stream << "roll = " << pm.roll << "\n";
  stream << "pitch = " << pm.pitch << "\n \n";
  return stream;
}

std::ostream& operator << (std::ostream& stream, const RigidModel& pm)
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

std::ostream& operator << (std::ostream& stream, const RotationalModel& rm)
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
std::ostream& operator << (std::ostream& stream, const ArticulationModelPtr& amptr)
{
  switch (amptr->model)
  {
    case (RIGID):
      {
        boost::shared_ptr<RigidModel> rigid = boost::dynamic_pointer_cast< RigidModel > (amptr);
        stream << "memory address: " << rigid << std::endl;
        stream << *rigid;
        break;
      }
    case (PRISMATIC):
      {
        boost::shared_ptr<PrismaticModel> prismatic = boost::dynamic_pointer_cast< PrismaticModel > (amptr);
        stream << "memory address: " << prismatic << std::endl;
        stream << *prismatic;
        break;
      }
    case (ROTATIONAL):
      {
        boost::shared_ptr<RotationalModel> rotational = boost::dynamic_pointer_cast< RotationalModel > (amptr);
        stream << "memory address: " << rotational << std::endl;
        stream << *rotational;
        break;
      }
  }
  return stream;
}

#endif
