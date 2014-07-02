#include "particle_filter/gaussian_sensormodel.h"
#include "particle_filter/random.h"

GaussianSensorModel::GaussianSensorModel()
{
}

GaussianSensorModel::~GaussianSensorModel()
{
}

Eigen::VectorXd GaussianSensorModel::sense(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const
{

}

double GaussianSensorModel::senseLikelihood(const Eigen::VectorXd &z, const Eigen::VectorXd &state, const Eigen::MatrixXd &cov) const
{
  // construct a gaussian with state as mean and noiseCov
  // check where z lies in the gaussian
  // return this value
  return Random::multivariateGaussianProbability(state, cov, z);
}
