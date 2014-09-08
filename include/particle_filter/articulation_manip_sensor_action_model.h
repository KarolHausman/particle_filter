#ifndef ARTICULATION_MANIPULATION_SENSORACTIONMODEL_H
#define ARTICULATION_MANIPULATION_SENSORACTIONMODEL_H

#include "particle_filter/sensor_action_model.h"

template <class StateType, class ZType, class AType>
class ArtManipSensorActionModel : public SensorActionModel<StateType, ZType, AType>
{
public:
  ArtManipSensorActionModel();
  virtual ~ArtManipSensorActionModel();

  ZType sense(const StateType &state, const Eigen::VectorXd &noise) const;
  double senseLikelihood(const ZType &z, const AType& a, const StateType &state, const Eigen::MatrixXd &cov) const;
  double senseLogLikelihood (const ZType& z, const AType& a, const StateType& state, const Eigen::MatrixXd& cov) const;

  ///the bigger the scale the more stiff the controller
  double scale;
  double log_multiplier;
  bool exponencial_likelihood, linear_likelihood, quadratic_likelihood, quadratic_likelihood_with_zero;
};


#endif //ARTICULATION_MANIPULATION_SENSORACTIONMODEL_H


