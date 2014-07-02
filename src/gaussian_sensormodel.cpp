#include "particle_filter/gaussian_sensormodel.h"

GaussianSensorModel::GaussianSensorModel()
{
}

GaussianSensorModel::~GaussianSensorModel()
{
}

Eigen::VectorXd GaussianSensorModel::sense(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const
{

}

double GaussianSensorModel::senseLikelihood(const Eigen::VectorXd &z, const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const
{
  // construct a gaussian with state as mean and noiseCov
  // check where z lies in the gaussian
  // return this value

  return 1.0;
}
