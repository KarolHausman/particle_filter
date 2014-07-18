#include "particle_filter/gaussian_sensormodel.h"
#include "particle_filter/random.h"

template <class StateType> GaussianSensorModel<StateType>::GaussianSensorModel()
{
}

template <class StateType> GaussianSensorModel<StateType>::~GaussianSensorModel()
{
}

template <class StateType> Eigen::VectorXd GaussianSensorModel<StateType>::sense(const StateType &state, const Eigen::VectorXd &noise) const
{

}

template <class StateType> double GaussianSensorModel<StateType>::senseLikelihood(const Eigen::VectorXd &z, const StateType &state, const Eigen::MatrixXd &cov) const
{
  // construct a gaussian with state as mean and noiseCov
  // check where z lies in the gaussian
  // return this value
  return Random::multivariateGaussianProbability(state, cov, z);
}

template class GaussianSensorModel<Eigen::VectorXd>;
