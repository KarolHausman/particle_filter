#include "particle_filter/paricle_filter.h"
#include <algorithm>


ParticleFilter::ParticleFilter()
{
}

ParticleFilter::~ParticleFilter()
{
}


bool ParticleFilter::normalizeWeights()
{
  double weights_sum = 0.0;
  for (std::vector <Particle>::iterator it = particles.begin();
    it != particles.end(); it++)
    {
      weights_sum += it->weight;
    }

  if (weights_sum  <= 0.0)
    {
      return false;
    }

  for (std::vector <Particle>::iterator it = particles.begin();
    it != particles.end(); it++)
    {
      it->weight = it->weight / weights_sum;
    }
  return true;
}

void ParticleFilter::sortParticles()
{
  std::sort(particles.begin(), particles.end());
}

bool ParticleFilter::resample()
{
}
