#include "particle_filter/particle.h"
#include "particle_filter/articulation_model.h"

template <class StateType> Particle <StateType>::Particle()
{
}

template <class StateType> Particle <StateType>::~Particle()
{
}

template class Particle <Eigen::VectorXd>;
template class Particle <double>;
template class Particle <ArticulationModelPtr>;

