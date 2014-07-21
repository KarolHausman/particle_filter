#include "particle_filter/articulation_model.h"
#include "particle_filter/rigid_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rotational_model.h"

ArticulationModel::ArticulationModel()
{}
ArticulationModel::~ArticulationModel()
{}


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
