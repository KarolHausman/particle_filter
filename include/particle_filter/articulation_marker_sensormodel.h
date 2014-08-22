#ifndef ARTICULATION_MARKER_SENSORMODEL_H
#define ARTICULATION_MARKER_SENSORMODEL_H

#include "particle_filter/sensor_model.h"

template <class StateType, class ZType>
class ArtMarkerSensorModel : public SensorModel<StateType, ZType>
{
public:
  ArtMarkerSensorModel();
  virtual ~ArtMarkerSensorModel();

  ZType sense(const StateType &state, const Eigen::VectorXd &noise) const;
  double senseLikelihood(const ZType &z, const StateType &state, const Eigen::MatrixXd &cov) const;
  double senseLogLikelihood (const ZType& z, const StateType& state, const Eigen::MatrixXd& cov) const;

  double loglikelihood_free_model;
};


#endif //ARTICULATION_MARKER_SENSORMODEL_H


