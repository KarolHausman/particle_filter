#ifndef GAUSSIAN_SENSORMODEL_H
#define GAUSSIAN_SENSORMODEL_H

#include "particle_filter/sensor_model.h"

template <class StateType, class ZType>
class GaussianSensorModel : public SensorModel<StateType, ZType>
{
public:
  GaussianSensorModel();
  virtual ~GaussianSensorModel();

  ZType sense(const StateType &state, const Eigen::VectorXd &noise) const;
  double senseLikelihood(const ZType &z, const StateType &state, const Eigen::MatrixXd &cov) const;
};


#endif //GAUSSIAN_SENSORMODEL_H


