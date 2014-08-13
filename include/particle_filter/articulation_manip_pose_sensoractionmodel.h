#ifndef ARTICULATION_MANIPULATION_POSE_SENSORACTIONMODEL_H
#define ARTICULATION_MANIPULATION_POSE_SENSORACTIONMODEL_H

#include "particle_filter/sensor_action_model.h"

template <class StateType, class ZType, class AType>
class ArtManipPoseSensorActionModel : public SensorActionModel<StateType, ZType, AType>
{
public:
  ArtManipPoseSensorActionModel();
  virtual ~ArtManipPoseSensorActionModel();

  ZType sense(const StateType &state, const Eigen::VectorXd &noise) const;
  double senseLikelihood(const ZType &z, const AType& a, const StateType &state, const Eigen::MatrixXd &cov) const;
  double senseLogLikelihood (const ZType& z, const AType& a, const StateType& state, const Eigen::MatrixXd& cov) const;

  double distance_threshold;
  double angle_threshold;
};


#endif //ARTICULATION_MANIPULATION_POSE_SENSORACTIONMODEL_H


