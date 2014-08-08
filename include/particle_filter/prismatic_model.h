#ifndef PRISMATIC_MODEL_H
#define PRISMATIC_MODEL_H

#include "particle_filter/rigid_model.h"

class PrismaticModel : public RigidModel
{
public:
  PrismaticModel();
  virtual ~PrismaticModel();

  virtual ArticulationModelPtr getCopy()
  {
   return static_cast<ArticulationModelPtr>(new PrismaticModel(*this));
  }


  void readParamsFromModel();
  void writeParamsToModel();

  size_t getDOFs() { return 1; }

  V_Configuration predictConfiguration(geometry_msgs::Pose pose);
  geometry_msgs::Pose predictPose(V_Configuration q);
  M_CartesianJacobian predictHessian(V_Configuration q,double delta=1e-6);

  bool guessParameters();
  void updateParameters(std::vector<double> delta);
  bool normalizeParameters();

  void updateStateParametersToModel();
  void updateModelToStateParameters();


  tf::Vector3 prismatic_dir;
  double axis_x, axis_y, axis_z;
protected:


};

#endif //PRISMATIC_MODEL_H
