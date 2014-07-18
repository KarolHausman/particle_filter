#include "particle_filter/paricle_filter.h"
#include <algorithm>
#include "particle_filter/random.h"
#include <iostream>
#include <ros/console.h>
#include "particle_filter/articulation_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rigid_model.h"
#include "particle_filter/rotational_model.h"
#include "particle_filter/free_model.h"


template <class ParticleType> ParticleFilter<ParticleType>::ParticleFilter(const std::vector <Particle <ParticleType> > &particles)
{
  this->particles = particles;
}

//initialization for localization
template <> ParticleFilter<Eigen::VectorXd>::ParticleFilter(const int& size, const Eigen::VectorXd& mean,
                               const Eigen::MatrixXd& cov)
{
  logLikelihoods = true;
  this->particles.resize(size);

  for (typename std::vector <Particle <Eigen::VectorXd> >::iterator it = particles.begin();
    it != particles.end(); it++)
  {
    it->state = mean + Random::multivariateGaussian(cov);
    it->weight = 1.0 / (double) size;
  }
  if (logLikelihoods)
    weightsToLogWeights();
}

//initialization of state for articulation models
template <> ParticleFilter<ArticulationModelPtr>::ParticleFilter(const int& size,
                                                                 const Eigen::VectorXd& rigid_mean, const Eigen::MatrixXd& rigid_cov,
                                                                 const Eigen::VectorXd& rotational_mean, const Eigen::MatrixXd& rotational_cov,
                                                                 const Eigen::VectorXd& prismatic_mean, const Eigen::MatrixXd& prismatic_cov)
{
  logLikelihoods = true;
  this->particles.resize(size);

  const uint freemodel_samples = 10;
  const double remaining_models_temp = static_cast<double> ((size - freemodel_samples) / (MODELS_NUMBER - 1));
  const int remaining_models = static_cast<uint> (remaining_models_temp);
  uint i = 0;
  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin();
    it != particles.end(); it++, i++)
  {
    if (i < freemodel_samples)
    {
      it->state.reset(new FreeModel);
    }
    else if (i >= freemodel_samples && i < freemodel_samples + remaining_models)
    {
      boost::shared_ptr<RigidModel> rigid_model(new RigidModel);
      Eigen::VectorXd state_vector = rigid_mean + Random::multivariateGaussian(rigid_cov);
      rigid_model->pos_x = state_vector(0);
      rigid_model->pos_y = state_vector(1);
      rigid_model->pos_z = state_vector(2);
      rigid_model->roll = state_vector(3);
      rigid_model->pitch = state_vector(4);
      rigid_model->yaw = state_vector(5);

      it->state = static_cast<ArticulationModelPtr> (rigid_model);
    }
    else if (i >= freemodel_samples + remaining_models && i < freemodel_samples + remaining_models*2)
    {
      boost::shared_ptr<RotationalModel> rotational_model(new RotationalModel);
      Eigen::VectorXd state_vector = rotational_mean + Random::multivariateGaussian(rotational_cov);
      rotational_model->rot_center_x = state_vector(0);
      rotational_model->rot_center_y = state_vector(1);
      rotational_model->rot_center_z = state_vector(2);
      rotational_model->roll = state_vector(3);
      rotational_model->pitch = state_vector(4);
      rotational_model->yaw = state_vector(5);
      rotational_model->radius = state_vector(6);
      rotational_model->axis_x = state_vector(7);
      rotational_model->axis_y = state_vector(8);

      it->state = static_cast<ArticulationModelPtr> (rotational_model);
    }
    else
    {
      boost::shared_ptr<PrismaticModel> prismatic_model(new PrismaticModel);
      Eigen::VectorXd state_vector = prismatic_mean + Random::multivariateGaussian(prismatic_cov);
      prismatic_model->pos_x = state_vector(0);
      prismatic_model->pos_y = state_vector(1);
      prismatic_model->pos_z = state_vector(2);
      prismatic_model->roll = state_vector(3);
      prismatic_model->pitch = state_vector(4);

      it->state = static_cast<ArticulationModelPtr> (prismatic_model);
    }
    it->weight = 1.0 / (double) size;
  }
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
//      continue;
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

//FIXME: This is not generic - works for Eigen::VectorXd or even Eigen::Vector3d
template <> Eigen::VectorXd ParticleFilter<Eigen::VectorXd>::getWeightedAvg(const double& particles_fraction)
{
  if (logLikelihoods)
    logWeightsToWeights();

  sortParticles();
  int weighted_num = particles.size() * particles_fraction;
  int i = 0;
  double weights_sum = 0;

  //FIXME: Hard coded Vector3d for this state, can be readed from stateDim maybe
  Eigen::VectorXd state_sum = Eigen::Vector3d::Zero();

  for (std::vector <Particle <Eigen::VectorXd> >::reverse_iterator it = particles.rbegin(); it != particles.rend();
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

  std::cerr << "before normalizing" << std::endl;

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

  std::cerr << "after normalizing" << std::endl;


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
  std::cerr << "cumultative weights calculated" << std::endl;
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
  std::cerr << "new particles calculated" << std::endl;
  // TODO: use swap maybe?
  particles = new_particles;
  std::cerr << "after swapping" << std::endl;

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
                       const MotionModel<ParticleType>& model)
{
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
                       it++)
  {
    Eigen::VectorXd noise = Random::multivariateGaussian(noiseCov);
    it->state = model.move(it->state, u, noise);
  }
}

template <class ParticleType> void ParticleFilter<ParticleType>::correct(const Eigen::VectorXd z, const Eigen::MatrixXd& noiseCov,
                     const SensorModel<ParticleType> &model)
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
template class ParticleFilter <double>;
template class ParticleFilter <ArticulationModelPtr>;


