#include "particle_filter/sensor_model.h"

template <class StateType, class ZType> SensorModel<StateType, ZType>::SensorModel()
{
}

template <class StateType, class ZType> SensorModel<StateType, ZType>::~SensorModel()
{
}

template <class StateType, class ZType> double SensorModel<StateType, ZType>::senseLogLikelihood(const ZType &z, const StateType &state,
                                       const Eigen::MatrixXd &cov) const
{
  return log(senseLikelihood(z, state, cov));
}

template class SensorModel<Eigen::VectorXd, Eigen::VectorXd>;
template class SensorModel<ArticulationModelPtr, Eigen::VectorXd>;
template class SensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;

