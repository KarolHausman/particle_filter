#ifndef GAUSSIAN_SENSORMODEL_H
#define GAUSSIAN_SENSORMODEL_H

#include "particle_filter/sensor_model.h"

template <class StateType>
class GaussianSensorModel : public SensorModel<StateType>
{
public:
  GaussianSensorModel();
  virtual ~GaussianSensorModel();

  Eigen::VectorXd sense(const StateType &state, const Eigen::VectorXd &noise) const;
  double senseLikelihood(const Eigen::VectorXd &z, const StateType &state, const Eigen::MatrixXd &cov) const;
};


#endif //GAUSSIAN_SENSORMODEL_H


