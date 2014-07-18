#include "particle_filter/motion_model.h"

template <class StateType> MotionModel<StateType>::MotionModel()
{
}

template <class StateType> MotionModel<StateType>::~MotionModel()
{
}

template class MotionModel<Eigen::VectorXd>;
