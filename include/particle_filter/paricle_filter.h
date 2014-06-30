#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle_filter/particle.h"

class ParticleFilter
{
public:
  ParticleFilter(const std::vector <Particle>& particles);
  virtual ~ParticleFilter();

  bool normalizeWeights();
  void sortParticles();
  virtual void predict();
  virtual void correct();


  std::vector <Particle> particles;

protected:
  virtual bool resample();


};





#endif // PARTICLE_FILTER_H
