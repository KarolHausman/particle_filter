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
  logLikelihoods = true;
  this->particles.resize(size);
  for (std::vector <Particle>::iterator it = particles.begin();
    it != particles.end(); it++)
  {
    it->state = mean + Random::multivariateGaussian(cov);
    it->weight = 1.0 / (double) size;
  }
  if (logLikelihoods)
    weightsToLogWeights();
}

ParticleFilter::~ParticleFilter()
{
}

bool ParticleFilter::normalizeLogWeights()
{
  sortParticles();
  /*
  //first method

  //find max and subtract max form every weight
  Particle maxParticle = particles.back();
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
  Particle maxParticle = particles.back();

  double sumLikelihoods= 0;

  for (std::vector <Particle>::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    it->weight = it->weight - maxParticle.weight;
// to do with precision -> disgard very low weights; not sure if needed
//    if (it->weight < -40)
//    {
//    }
    it->weight = exp(it->weight);
    sumLikelihoods += it->weight;
  }
  if (sumLikelihoods  <= 0.0)
  {
    std::cerr << "sumLikelihoods: " << sumLikelihoods << std::endl;
    std::cerr << " , sumLikelihoods <= 0!!! " << std::endl;
    return false;
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
  }

  if (weights_sum  <= 0.0)
  {
    std::cerr << "weights_sum: " << weights_sum << std::endl;
    std::cerr << " , WEIGHTS_SUM <= 0!!! " << std::endl;
    return false;
  }

  for (std::vector <Particle>::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    it->weight = it->weight / weights_sum;
  }

  return true;
}

bool ParticleFilter::getLogLikelihoodsFlag() const
{
  return logLikelihoods;
}

void ParticleFilter::setLogLikelihoodsFlag(const bool &flag)
{
  if (logLikelihoods != flag)
  {
    if (flag)
    {
      weightsToLogWeights();
    }
    else
    {
      logWeightsToWeights();
    }
  }
  logLikelihoods = flag;
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
  if (logLikelihoods)
    logWeightsToWeights();

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
        if (logLikelihoods)
          weightsToLogWeights();
        return Eigen::Vector3d::Zero();
      }
      if (logLikelihoods)
        weightsToLogWeights();
      return state_sum/ weights_sum;
    }
    weights_sum += it->weight;
    state_sum += it->state * it->weight;
  }
  if (logLikelihoods)
    weightsToLogWeights();
  return Eigen::Vector3d::Zero();
}

bool ParticleFilter::resample(const int& particles_number)
{
  if (logLikelihoods)
    if (!normalizeLogWeights())
      return false;
  else
    if (!normalizeWeights())
      return false;


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
  // TODO: use swap maybe?
  particles = new_particles;
  if (logLikelihoods)
    weightsToLogWeights();
  return true;
}

void ParticleFilter::logWeightsToWeights()
{
  std::cout << "switching to normal likelihoods" << std::endl;
  for (std::vector <Particle>::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    it->weight = exp(it->weight);
  }
}

void ParticleFilter::weightsToLogWeights()
{
  std::cout << "switching to log likelihoods" << std::endl;
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
    if (!logLikelihoods)
      it->weight *= model.senseLikelihood(z, it->state, noiseCov);
    else
      it->weight += model.senseLogLikelihood(z, it->state, noiseCov);
  }
}


