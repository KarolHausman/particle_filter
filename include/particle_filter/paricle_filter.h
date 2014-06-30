#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle_filter/particle.h"

class ParticleFilter
{
public:
  ParticleFilter();
  virtual ~ParticleFilter();

  bool normalizeWeights();
  void sortParticles();


  std::vector <Particle> particles;

protected:
  virtual bool resample();


};





#endif // PARTICLE_FILTER_H
