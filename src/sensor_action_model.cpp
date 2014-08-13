#include "particle_filter/sensor_action_model.h"

template <class StateType, class ZType, class AType> SensorActionModel<StateType, ZType, AType>::SensorActionModel()
{
}

template <class StateType, class ZType, class AType> SensorActionModel<StateType, ZType, AType>::~SensorActionModel()
{
}

template <class StateType, class ZType, class AType> double SensorActionModel<StateType, ZType, AType>::senseLogLikelihood(const ZType &z, const AType& a, const StateType &state,
                                       const Eigen::MatrixXd &cov) const
{
  return log(senseLikelihood(z, a, state, cov));
}

template class SensorActionModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg, ActionPtr>;
template class SensorActionModel<ArticulationModelPtr, int, ActionPtr>;
template class SensorActionModel<ArticulationModelPtr, tf::Transform, ActionPtr>;
