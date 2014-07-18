#include "particle_filter/sensor_model.h"

template <class StateType> SensorModel<StateType>::SensorModel()
{
}

template <class StateType> SensorModel<StateType>::~SensorModel()
{
}

template <class StateType> double SensorModel<StateType>::senseLogLikelihood(const Eigen::VectorXd &z, const StateType &state,
                                       const Eigen::MatrixXd &cov) const
{
  return log(senseLikelihood(z, state, cov));
}

template class SensorModel<Eigen::VectorXd>;
