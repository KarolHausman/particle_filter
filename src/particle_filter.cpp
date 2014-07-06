#include "particle_filter/paricle_filter.h"
#include <algorithm>
#include "particle_filter/random.h"
#include <iostream>


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

bool ParticleFilter::normalizeLogWeights()
{
  /*
  //first method

  //find max and subtract max form every weight
  Particle maxParticle = *(std::max(particles.begin(), particles.end()));
  double sumLikelihoodsTemp = 0;

  for (std::vector <Particle>::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    double tempWeight = it->weight - maxParticle.weight;
    sumLikelihoodsTemp += exp(tempWeight);
  }
  double sumLikelihoods = maxParticle.weight + log(sumLikelihoodsTemp);
  for (std::vector <Particle>::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    it->weight = it->weight / sumLikelihoods;
  }
  return true;
  */

  //second method
  Particle maxParticle = *(std::max(particles.begin(), particles.end()));
  double sumLikelihoods= 0;

  for (std::vector <Particle>::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    it->weight = it->weight - maxParticle.weight;
//    if (it->weight < -40)
//    {
//    }
    it->weight = exp(it->weight);
    sumLikelihoods += it->weight;
  }
  for (std::vector <Particle>::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    it->weight = it->weight / sumLikelihoods;
  }
  return true;


}

bool ParticleFilter::normalizeWeights()
{
  double weights_sum = 0.0;
  for (std::vector <Particle>::iterator it = particles.begin();
    it != particles.end(); it++)
    {
      weights_sum += it->weight;
      std::cout << "it->weight: " << it->weight << std::endl;
    }

  if (weights_sum  <= 0.0)
    {
      std::cerr << "weights_sum: " << weights_sum << std::endl;
      std::cerr << "WEIGHTS_SUM <= 0!!!" << std::endl;
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

double ParticleFilter::getWeightsSum() const
{
  double weights_sum = 0;
  for (std::vector <Particle>::const_iterator it = particles.begin(); it != particles.end();
       it++)
  {
    weights_sum += it->weight;
  }
  return weights_sum;
}


Eigen::VectorXd ParticleFilter::getWeightedAvg(const double& particles_fraction)
{
  sortParticles();
  int weighted_num = particles.size() * particles_fraction;
  int i = 0;
  double weights_sum = 0;


  //FIXME: Hard coded Vector3d for this state, can be readed from stateDim maybe
  Eigen::VectorXd state_sum = Eigen::Vector3d::Zero();

  for (std::vector <Particle>::reverse_iterator it = particles.rbegin(); it != particles.rend();
       it++, i++)
  {
    if(i >= weighted_num)
    {
      if (weights_sum <= 0)
      {
        std::cerr << "WEIGHTS_SUM <= 0";
        return Eigen::Vector3d::Zero();
      }
      return state_sum/ weights_sum;
    }
    weights_sum += it->weight;
    state_sum += it->state * it->weight;
  }
  return Eigen::Vector3d::Zero();
}

bool ParticleFilter::resample(const int& particles_number)
{
  normalizeLogWeights();
//  normalizeWeights();

  std::vector <Particle> new_particles;
  new_particles.reserve(particles_number);
  sortParticles();
  std::vector <double> cumultative_weights;
  double cumultative_weight = 0;
  for (std::vector <Particle>::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    cumultative_weight += it->weight;
    cumultative_weights.push_back(cumultative_weight);
  }
  for (int i = 0; i < particles_number; i++)
  {
    double random_double = Random::uniform(0.0, 1.0);
    int idx = 0;
    for (std::vector <double>::iterator it = cumultative_weights.begin(); it != cumultative_weights.end();
         ++it, ++idx)
    {
      if (random_double < *it)
      {
        new_particles.push_back(particles[idx]);
        new_particles.back().weight = 1.0 / particles_number;
        break;
      }
    }
  }
  // use swap maybe?
  particles = new_particles;

  //change to log
  for (std::vector <Particle>::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    it->weight = log(it->weight);
  }
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
//    Eigen::VectorXd noise = Random::multivariateGaussian(noiseCov);
//    it->weight *= model.senseLikelihood(z, it->state, noiseCov);
    it->weight += model.senseLogLikelihood(z, it->state, noiseCov);
  }
//  normalizeWeights();
}


