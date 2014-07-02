#ifndef GAUSSIAN_SENSORMODEL_H
#define GAUSSIAN_SENSORMODEL_H

#include "particle_filter/sensor_model.h"

class GaussianSensorModel : public SensorModel
{
public:
  GaussianSensorModel();
  virtual ~GaussianSensorModel();

  Eigen::VectorXd sense(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const;
  double senseLikelihood(const Eigen::VectorXd &z, const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const;
};


#endif //GAUSSIAN_SENSORMODEL_H


