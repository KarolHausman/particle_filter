#include "particle_filter/particle.h"

template <class StateType> Particle <StateType>::Particle()
{
}
//deep copy constructor for ArticulationModelPtr
template <> Particle <ArticulationModelPtr>::Particle(const Particle<ArticulationModelPtr>& p2)
{
  this->state  = p2.state->getCopy();
  this->weight = p2.weight;
}
template <> Particle <ArticulationModelPtr>&  Particle <ArticulationModelPtr>::operator = (
    const Particle<ArticulationModelPtr>& p2)
{
  this->state  = p2.state->getCopy();
  this->weight = p2.weight;
}
template <class StateType> Particle <StateType>::~Particle()
{
}

template class Particle <Eigen::VectorXd>;
template class Particle <ArticulationModelPtr>;


