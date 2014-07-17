#include "particle_filter/particle.h"

template <class StateType> Particle <StateType>::Particle()
{
}

template <class StateType> Particle <StateType>::~Particle()
{
}

template class Particle <Eigen::VectorXd>;
