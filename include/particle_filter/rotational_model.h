#ifndef ROTATIONAL_MODEL_H
#define ROTATIONAL_MODEL_H

#include "particle_filter/articulation_model.h"

class RotationalModel : public ArticulationModel
{
public:
  RotationalModel();
  virtual ~RotationalModel();

  virtual ArticulationModelPtr getCopy()
  {
   return static_cast<ArticulationModelPtr>(new RotationalModel(*this));
  }


  // -- params
  void readParamsFromModel();
  void writeParamsToModel();

  void updateStateParametersToModel();
  void updateModelToStateParameters();

  size_t getDOFs() { return 1; }

  V_Configuration predictConfiguration(geometry_msgs::Pose pose);
  geometry_msgs::Pose predictPose(V_Configuration q);

  bool guessParameters();
  void updateParameters(std::vector<double> delta);
  bool normalizeParameters();

  double rot_mode;
  tf::Vector3 rot_center;
  tf::Quaternion rot_axis;

  double rot_radius;
  tf::Quaternion rot_orientation;

  double rot_center_x, rot_center_y, rot_center_z, roll, pitch, yaw, radius, axis_roll, axis_pitch, axis_yaw;
protected:


};

#endif //ROTATIONAL_MODEL_H
