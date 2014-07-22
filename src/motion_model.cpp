#include "particle_filter/motion_model.h"
#include <particle_filter/articulation_model.h>

template <class StateType> MotionModel<StateType>::MotionModel()
{
}

template <class StateType> MotionModel<StateType>::~MotionModel()
{
}

template class MotionModel<Eigen::VectorXd>;
template class MotionModel<ArticulationModelPtr>;
