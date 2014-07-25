#ifndef RIGID_MODEL_H
#define RIGID_MODEL_H

#include "particle_filter/articulation_model.h"

class RigidModel : public ArticulationModel
{
public:
  RigidModel();
  virtual ~RigidModel();

  virtual ArticulationModelPtr getCopy()
  {
   return static_cast<ArticulationModelPtr>(new RigidModel(*this));
  }

  // -- params
  void readParamsFromModel();
  void writeParamsToModel();

  size_t getDOFs() { return 0; }

  geometry_msgs::Pose predictPose(V_Configuration q);

  bool guessParameters();
  void updateParameters(std::vector<double> delta);
  virtual void updateStateParametersToModel();
  virtual void updateModelToStateParameters();

  tf::Vector3 rigid_position;
  tf::Quaternion rigid_orientation;
//  double rigid_width,rigid_height; TODO: Not needed now
  double pos_x, pos_y, pos_z, roll, pitch, yaw;
protected:


};

#endif //RIGID_MODEL_H
