#include "particle_filter/paricle_filter.h"
#include <algorithm>
#include "particle_filter/random.h"
#include <iostream>
#include <ros/console.h>


template <class ParticleType> ParticleFilter<ParticleType>::ParticleFilter(const std::vector <Particle <ParticleType> > &particles)
{
  this->particles = particles;
}

template <class ParticleType> ParticleFilter<ParticleType>::ParticleFilter(const int& size, const Eigen::VectorXd& mean,
                               const Eigen::MatrixXd& cov)
{
  logLikelihoods = true;
  this->particles.resize(size);
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin();
    it != particles.end(); it++)
  {
    it->state = mean + Random::multivariateGaussian(cov);
    it->weight = 1.0 / (double) size;
  }
  if (logLikelihoods)
    weightsToLogWeights();
}

template <class ParticleType> ParticleFilter<ParticleType>::~ParticleFilter()
{
}

template <class ParticleType> void ParticleFilter<ParticleType>::printParticles()
{
  std::cout << "printing particles: " << std::endl;
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    std::cout << *it;
  }
}

template <class ParticleType> bool ParticleFilter<ParticleType>::normalizeLogWeights()
{
  sortParticles();
  Particle <ParticleType> maxParticle = particles.back();
  double sumLikelihoods= 0;

  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin();
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
    ROS_ERROR_STREAM ("sumLikelihoods: " << sumLikelihoods <<
                 " , sumLikelihoods <= 0!!! returning false in normalizeLogweights()");
    return false;
  }
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    it->weight = it->weight / sumLikelihoods;
  }
  return true;
}

template <class ParticleType> bool ParticleFilter<ParticleType>::normalizeWeights()
{
  double weights_sum = 0.0;
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin();
    it != particles.end(); it++)
  {
    weights_sum += it->weight;
  }

  if (weights_sum  <= 0.0)
  {
    ROS_ERROR_STREAM ("weights_sum: " << weights_sum <<
                 " , WEIGHTS_SUM <= 0!!! returning false in normalizeWeights()");
    return false;
  }

  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    it->weight = it->weight / weights_sum;
  }

  return true;
}

template <class ParticleType> bool ParticleFilter<ParticleType>::getLogLikelihoodsFlag() const
{
  return logLikelihoods;
}

template <class ParticleType> void ParticleFilter<ParticleType>::setLogLikelihoodsFlag(const bool &flag)
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

template <class ParticleType> void ParticleFilter<ParticleType>::sortParticles()
{
  std::sort(particles.begin(), particles.end());
}

template <class ParticleType> double ParticleFilter<ParticleType>::getWeightsSum() const
{
  double weights_sum = 0;
  for (typename std::vector <Particle <ParticleType> >::const_iterator it = particles.begin(); it != particles.end();
       it++)
  {
    weights_sum += it->weight;
  }
  return weights_sum;
}

//FIXME: This is not generic - works for Eigen::VectorXd
template <class ParticleType> Eigen::VectorXd ParticleFilter<ParticleType>::getWeightedAvg(const double& particles_fraction)
{
  if (logLikelihoods)
    logWeightsToWeights();

  sortParticles();
  int weighted_num = particles.size() * particles_fraction;
  int i = 0;
  double weights_sum = 0;

  //FIXME: Hard coded Vector3d for this state, can be readed from stateDim maybe
  Eigen::VectorXd state_sum = Eigen::Vector3d::Zero();

  for (typename std::vector <Particle <ParticleType> >::reverse_iterator it = particles.rbegin(); it != particles.rend();
       it++, i++)
  {
    if(i >= weighted_num)
    {
      if (weights_sum <= 0)
      {
        ROS_ERROR ("WEIGHTS_SUM <= 0 returning 0 in getWeightedAvg");
        if (logLikelihoods)
        {
          weightsToLogWeights();
        }
        return Eigen::Vector3d::Zero();
      }
      if (logLikelihoods)
      {
        weightsToLogWeights();
      }
      return state_sum/ weights_sum;
    }
    weights_sum += it->weight;
    state_sum += it->state * it->weight;
  }
  if (logLikelihoods)
  {
    weightsToLogWeights();
  }
  ROS_ERROR ("returning 0 in getWeightedAvg because i was never >= weighted_num");
  return Eigen::Vector3d::Zero();
}

template <class ParticleType> bool ParticleFilter<ParticleType>::resample(const int& particles_number)
{

  if (logLikelihoods)
    {
    if (!normalizeLogWeights())
      {
        return false;
      }
    }
  else
    {
    if (!normalizeWeights())
      {
        return false;
      }
    }

  //at this point these are normal likelihoods, not log
  typename std::vector <Particle <ParticleType> > new_particles;
  new_particles.reserve(particles_number);
  sortParticles();
  std::vector <double> cumultative_weights;
  double cumultative_weight = 0;
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    cumultative_weight += it->weight;
    cumultative_weights.push_back(cumultative_weight);
  }
  for (int i = 0; i < particles_number; i++)
  {
    double random_double = Random::uniform(0.0, 1.0);
    int idx = 0;
    for (typename std::vector <double>::iterator it = cumultative_weights.begin(); it != cumultative_weights.end();
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

template <class ParticleType> void ParticleFilter<ParticleType>::logWeightsToWeights()
{
  ROS_INFO ("switching to normal likelihoods");
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    it->weight = exp(it->weight);
  }
}

template <class ParticleType> void ParticleFilter<ParticleType>::weightsToLogWeights()
{
  ROS_INFO ("switching to log likelihoods");
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    it->weight = log(it->weight);
  }
}

template <class ParticleType> void ParticleFilter<ParticleType>::propagate(const Eigen::VectorXd& u, const Eigen::MatrixXd& noiseCov,
                       const MotionModel& model)
{
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
                       it++)
  {
    Eigen::VectorXd noise = Random::multivariateGaussian(noiseCov);
    it->state = model.move(it->state, u, noise);
  }
}

template <class ParticleType> void ParticleFilter<ParticleType>::correct(const Eigen::VectorXd z, const Eigen::MatrixXd& noiseCov,
                     const SensorModel& model)
{
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
                       it++)
  {
    if (!logLikelihoods)
      it->weight *= model.senseLikelihood(z, it->state, noiseCov);
    else
      it->weight += model.senseLogLikelihood(z, it->state, noiseCov);
  }
}

template class ParticleFilter <Eigen::VectorXd>;
