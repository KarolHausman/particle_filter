#include "particle_filter/identity_motionmodel.h"

template <class StateType> IdentityMotionModel<StateType>::IdentityMotionModel()
{
}

template <class StateType> IdentityMotionModel<StateType>::~IdentityMotionModel()
{
}

template <class StateType> StateType IdentityMotionModel<StateType>::move(const StateType &state, const Eigen::VectorXd &controls,
                          const Eigen::VectorXd &noise) const
{
  return state + noise;
}

template class IdentityMotionModel<Eigen::VectorXd>;
