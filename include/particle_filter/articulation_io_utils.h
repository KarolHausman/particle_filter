#ifndef ARTICULATION_IO_UTILS
#define ARTICULATION_IO_UTILS
#include "particle_filter/rigid_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rotational_model.h"
#include "particle_filter/free_model.h"
#include "particle_filter/articulation_model.h"


inline std::ostream& operator << (std::ostream& stream, const ArticulationModel& am)
{
  stream << "model= " << am.model << "\n";
  return stream;
}

inline std::ostream& operator << (std::ostream& stream, const PrismaticModel& pm)
{
  stream << "MODEL = " << pm.model << "\n";
  stream << "pos_x = " << pm.pos_x << "\n";
  stream << "pos_y = " << pm.pos_y << "\n";
  stream << "pos_z = " << pm.pos_z << "\n";
  stream << "roll = " << pm.roll << "\n";
  stream << "pitch = " << pm.pitch << "\n";
  stream << "yaw = " << pm.yaw << "\n";
  stream << "axis_x = " << pm.axis_x << "\n";
  stream << "axis_y = " << pm.axis_y << "\n";
  stream << "axis_z = " << pm.axis_z << "\n \n";
  return stream;
}

inline std::ostream& operator << (std::ostream& stream, const RigidModel& pm)
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

inline std::ostream& operator << (std::ostream& stream, const RotationalModel& rm)
{
  stream << "MODEL = " << rm.model << "\n";
  stream << "rot_center_x = " << rm.rot_center_x << "\n";
  stream << "rot_center_y = " << rm.rot_center_y << "\n";
  stream << "rot_center_z = " << rm.rot_center_z << "\n";
  stream << "roll = " << rm.roll << "\n";
  stream << "pitch = " << rm.pitch << "\n";
  stream << "yaw = " << rm.yaw << "\n";
  stream << "radius = " << rm.radius << "\n";
  stream << "axis_roll = " << rm.axis_roll << "\n";
  stream << "axis_pitch = " << rm.axis_pitch << "\n";
  stream << "axis_yaw = " << rm.axis_yaw << "\n \n";
  return stream;
}

inline std::ostream& operator << (std::ostream& stream, const FreeModel& fm)
{
  stream << "MODEL = " << fm.model << "\n";
  return stream;
}


inline std::ostream& operator << (std::ostream& stream, const ArticulationModelPtr& amptr)
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
    case (FREE):
      {
        boost::shared_ptr<FreeModel> free = boost::dynamic_pointer_cast< FreeModel > (amptr);
        stream << "memory address: " << free << std::endl;
        stream << *free;
        break;
      }
  }
  return stream;
}

#endif
