#include "particle_filter/paricle_filter.h"
#include <algorithm>
#include "particle_filter/random.h"


ParticleFilter::ParticleFilter(const std::vector <Particle>& particles)
{
  this->particles = particles;
}

ParticleFilter::ParticleFilter(const int& size, const Eigen::VectorXd& mean,
                               const Eigen::MatrixXd& cov)
{
  this->particles.resize(size);
  for (std::vector <Particle>::iterator it = particles.begin();
    it != particles.end(); it++)
    {
      it->state = mean + Random::multivariateGaussian(cov);
      it->weight = 1.0 / (double) size;
    }
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

void ParticleFilter::propagate(const Eigen::VectorXd& u, const Eigen::MatrixXd& noiseCov,
                       const MotionModel& model)
{
  for (std::vector <Particle>::iterator it = particles.begin(); it != particles.end();
                       it++)
  {
    Eigen::VectorXd noise = Random::multivariateGaussian(noiseCov);
    it->state = model.move(it->state, u, noise);
  }

}

void ParticleFilter::correct(const Eigen::VectorXd z, const Eigen::MatrixXd& noiseCov,
                     const SensorModel& model)
{
  for (std::vector <Particle>::iterator it = particles.begin(); it != particles.end();
                       it++)
  {
    Eigen::VectorXd noise = Random::multivariateGaussian(noiseCov);
    it->weight = model.senseLikelihood(z, it->state, noise);
  }
  normalizeWeights();
}


