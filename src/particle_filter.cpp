#include "particle_filter/paricle_filter.h"
#include "particle_filter/random.h"
#include <algorithm>
#include <iostream>
#include <ros/console.h>
#include "particle_filter/articulation_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rigid_model.h"
#include "particle_filter/rotational_model.h"
#include "particle_filter/free_model.h"

#include "particle_filter/articulation_io_utils.h"
#include <math.h>


template <class ParticleType> ParticleFilter<ParticleType>::ParticleFilter(const std::vector <Particle <ParticleType> > &particles)
{
  this->particles = particles;
}

//initialization for localization
template <> ParticleFilter<Eigen::VectorXd>::ParticleFilter(const int& size, const Eigen::VectorXd& mean,
                               const Eigen::MatrixXd& cov):
  logLikelihoods_(true)
{
  this->particles.resize(size);

  for (typename std::vector <Particle <Eigen::VectorXd> >::iterator it = particles.begin();
    it != particles.end(); it++)
  {
    it->state = mean + Random::multivariateGaussian(cov);
    it->weight = 1.0 / (double) size;
  }
  if (logLikelihoods_)
    weightsToLogWeights(particles);
}

//TODO: take care of free model
template <> ParticleFilter<ArticulationModelPtr>::ParticleFilter(const int& size, articulation_model_msgs::ModelMsg& model):
  logLikelihoods_(true),freemodel_samples_(0)
{
  this->particles.resize(size);
  const double remaining_models_temp = static_cast<double> ((size - freemodel_samples_) / (MODELS_NUMBER - 1));
  const int remaining_models = static_cast<uint> (remaining_models_temp);
  uint i = 0;
  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin();
    it != particles.end(); it++, i++)
  {
    if (i < freemodel_samples_)
    {
      it->state.reset(new FreeModel);
    }
    else if (i >= freemodel_samples_ && i < freemodel_samples_ + remaining_models)
    {
      ArticulationModelPtr rigid_model(new RigidModel);
      model.name = "rigid";
      rigid_model->setModel(model);
      rigid_model->fitModel();
      it->state = rigid_model;
    }
    else if (i >= freemodel_samples_ + remaining_models && i < freemodel_samples_ + remaining_models*2)
    {
      ArticulationModelPtr rotational_model(new RotationalModel);
      model.name = "rotational";
      rotational_model->setModel(model);
      rotational_model->fitModel();
      it->state = rotational_model;
    }
    else
    {
      ArticulationModelPtr prismatic_model(new PrismaticModel);
      model.name = "prismatic";
      prismatic_model->setModel(model);
      prismatic_model->fitModel();
      it->state = prismatic_model;
    }
    it->weight = 1.0 / (double) size;
  }
  if (logLikelihoods_)
  {
    weightsToLogWeights(particles);
  }
}

// another constructor for articulation model but faster
template <> ParticleFilter<ArticulationModelPtr>::ParticleFilter(const int& size, articulation_model_msgs::ModelMsg& model,
                                                                 const MotionModel<ArticulationModelPtr>& motion_model, const Eigen::MatrixXd& rigid_cov,
                                                                 const Eigen::MatrixXd& rotational_cov, const Eigen::MatrixXd& prismatic_cov):
  logLikelihoods_(true),freemodel_samples_(0)
{
  this->particles.resize(size);
  const double remaining_models_temp = static_cast<double> ((size - freemodel_samples_) / (MODELS_NUMBER - 1));
  const int remaining_models = static_cast<uint> (remaining_models_temp);
  const int fitmodels_number = 9;
  uint i = 0;
  uint rigid_models_counter = 0;
  uint rotational_models_counter = 0;
  uint prismatic_models_counter = 0;

  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin();
    it != particles.end(); it++, i++)
  {
    if (i < freemodel_samples_)
    {
      it->state.reset(new FreeModel);
    }
    else if (i >= freemodel_samples_ && i < freemodel_samples_ + remaining_models)
    {
      ArticulationModelPtr rigid_model(new RigidModel);
      model.name = "rigid";
      rigid_model->setModel(model);
      if (rigid_models_counter < fitmodels_number)
      {
        rigid_model->fitModel();
      }
      else
      {
        Eigen::VectorXd noise = Random::multivariateGaussian(rigid_cov);
        Eigen::VectorXd u = Eigen::VectorXd::Ones(10);
        rigid_model = motion_model.move(particles[freemodel_samples_].state, u, noise);
      }
      it->state = rigid_model;
      ++rigid_models_counter;
    }
    else if (i >= freemodel_samples_ + remaining_models && i < freemodel_samples_ + remaining_models*2)
    {
      ArticulationModelPtr rotational_model(new RotationalModel);
      model.name = "rotational";
      rotational_model->setModel(model);
      if (rotational_models_counter < fitmodels_number)
      {
        rotational_model->fitModel();
      }
      else
      {
        Eigen::VectorXd noise = Random::multivariateGaussian(rotational_cov);
        Eigen::VectorXd u = Eigen::VectorXd::Ones(10);
        rotational_model = motion_model.move(particles[freemodel_samples_ + remaining_models].state, u, noise);
      }
      it->state = rotational_model;
      ++rotational_models_counter;
    }
    else
    {
      ArticulationModelPtr prismatic_model(new PrismaticModel);
      model.name = "prismatic";
      prismatic_model->setModel(model);

      if (prismatic_models_counter < fitmodels_number)
      {
        prismatic_model->fitModel();
      }
      else
      {
        Eigen::VectorXd noise = Random::multivariateGaussian(prismatic_cov);
        Eigen::VectorXd u = Eigen::VectorXd::Ones(10);

        prismatic_model = motion_model.move(particles[freemodel_samples_ + remaining_models*2].state, u, noise);
      }
      it->state = prismatic_model;
      ++prismatic_models_counter;
    }
    it->weight = 1.0 / (double) size;
  }
  if (logLikelihoods_)
  {
    weightsToLogWeights(particles);
  }
}

//initialization of state for articulation models
template <> ParticleFilter<ArticulationModelPtr>::ParticleFilter(const int& size,
                                                                 const Eigen::VectorXd& rigid_mean, const Eigen::MatrixXd& rigid_cov,
                                                                 const Eigen::VectorXd& rotational_mean, const Eigen::MatrixXd& rotational_cov,
                                                                 const Eigen::VectorXd& prismatic_mean, const Eigen::MatrixXd& prismatic_cov):
  logLikelihoods_(true), freemodel_samples_(1)
{
  this->particles.resize(size);

  const double remaining_models_temp = static_cast<double> ((size - freemodel_samples_) / (MODELS_NUMBER - 1));
  const int remaining_models = static_cast<uint> (remaining_models_temp);
  uint i = 0;
  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin();
    it != particles.end(); it++, i++)
  {
    if (i < freemodel_samples_)
    {
      it->state.reset(new FreeModel);
    }
    else if (i >= freemodel_samples_ && i < freemodel_samples_ + remaining_models)
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
    else if (i >= freemodel_samples_ + remaining_models && i < freemodel_samples_ + remaining_models*2)
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
      rotational_model->axis_roll = state_vector(7);
      rotational_model->axis_pitch = state_vector(8);
      rotational_model->axis_yaw = state_vector(9);

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
      prismatic_model->yaw = state_vector(5);
      prismatic_model->axis_x = state_vector(6);
      prismatic_model->axis_y = state_vector(7);
      prismatic_model->axis_z = state_vector(8);

      it->state = static_cast<ArticulationModelPtr> (prismatic_model);
    }
    it->weight = 1.0 / (double) size;
  }
  if (logLikelihoods_)
    weightsToLogWeights(particles);
}

template <class ParticleType> ParticleFilter<ParticleType>::~ParticleFilter()
{
}

template <class ParticleType> void ParticleFilter<ParticleType>::printParticles( const std::vector<Particle <ParticleType> >& particles) const
{
  std::cout << "printing particles: " << std::endl;
  for (typename std::vector <Particle <ParticleType> >::const_iterator it = particles.begin();
      it != particles.end(); it++)
  {
    std::cout << *it;
  }
}


template <class ParticleType> bool ParticleFilter<ParticleType>::normalizeLogWeights(std::vector<Particle <ParticleType> >& particles, const bool& visualization_only )
{
  sortParticles(particles);
  Particle <ParticleType> maxParticle = particles.back();
  double sumLikelihoods= 0;

  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin();
      it != particles.end(); it++)
  {
    if (visualization_only)
    {
      it->weight_to_print_only = it->weight - maxParticle.weight;
    }
    else
    {
      it->weight = it->weight - maxParticle.weight;
    }
// TODO: TRY to do with precision -> disgard very low weights; not sure if needed
//    if (it->weight < -40)
//    {
//      continue;
//    }
    if (visualization_only)
    {
      it->weight_to_print_only = exp(it->weight_to_print_only);
      sumLikelihoods += it->weight_to_print_only;
    }
    else
    {
      it->weight = exp(it->weight);
      sumLikelihoods += it->weight;
    }
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
    if (visualization_only)
    {
      it->weight_to_print_only = it->weight_to_print_only / sumLikelihoods;
    }
    else
    {
      it->weight = it->weight / sumLikelihoods;
      it->weight_to_print_only = it->weight;
    }
  }
  return true;
}

template <class ParticleType> bool ParticleFilter<ParticleType>::normalizeWeights(std::vector<Particle <ParticleType> >& particles, const bool& visualization_only)
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
    if(visualization_only)
    {
      it->weight_to_print_only = it->weight / weights_sum;
    }
    else
    {
      it->weight = it->weight / weights_sum;
      it->weight_to_print_only = it->weight;
    }
  }

  return true;
}

template <class ParticleType> bool ParticleFilter<ParticleType>::getLogLikelihoodsFlag() const
{
  return logLikelihoods_;
}

//TODO: figure out how to do it with particles vector as input
//template <class ParticleType> void ParticleFilter<ParticleType>::setLogLikelihoodsFlag(const bool &flag)
//{
//  if (logLikelihoods_ != flag)
//  {
//    if (flag)
//    {
//      weightsToLogWeights();
//    }
//    else
//    {
//      logWeightsToWeights();
//    }
//  }
//  logLikelihoods_ = flag;
//}

template <class ParticleType> void ParticleFilter<ParticleType>::sortParticles( std::vector<Particle <ParticleType> >& particles)
{
  std::sort(particles.begin(), particles.end());
}

template <class ParticleType> double ParticleFilter<ParticleType>::getWeightsSum(const std::vector<Particle <ParticleType> >& particles) const
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
template <> Eigen::VectorXd ParticleFilter<Eigen::VectorXd>::getWeightedAvg(std::vector<Particle <Eigen::VectorXd> >& particles, const double& particles_fraction)
{
  if (logLikelihoods_)
    logWeightsToWeights(particles);

  sortParticles(particles);
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
        if (logLikelihoods_)
        {
          weightsToLogWeights(particles);
        }
        return Eigen::Vector3d::Zero();
      }
      if (logLikelihoods_)
      {
        weightsToLogWeights(particles);
      }
      return state_sum/ weights_sum;
    }
    weights_sum += it->weight;
    state_sum += it->state * it->weight;
  }
  if (logLikelihoods_)
  {
    weightsToLogWeights(particles);
  }
  ROS_ERROR ("returning 0 in getWeightedAvg because i was never >= weighted_num");
  return Eigen::Vector3d::Zero();
}

template <> void ParticleFilter<ArticulationModelPtr>::splitArticulationModels()
{
  particles_rigid.clear();
  particles_prismatic.clear();
  particles_rotational.clear();
  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    ArticulationModelPtr new_state;

    switch (it->state->model)
    {
      case (RIGID):
      {
//        new_state.reset(new RigidModel);
//        new_state = it->state->getCopy();
//        p.state = new_state;
//        p.weight = it->weight;
        Particle<ArticulationModelPtr> p = *it;
        particles_rigid.push_back(p);
        break;
      }
      case (PRISMATIC):
      {
        Particle<ArticulationModelPtr> p = *it;
        particles_prismatic.push_back(p);
        break;
      }
      case (ROTATIONAL):
      {
        Particle<ArticulationModelPtr> p = *it;
        particles_rotational.push_back(p);
        break;
      }
    }
  }

}

template <> void ParticleFilter<ArticulationModelPtr>::mergeArticulationModels()
{
  particles.clear();
  const int all_particles_number = particles_rigid.size() + particles_prismatic.size() + particles_rotational.size();

  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles_rigid.begin(); it != particles_rigid.end();
       it++)
  {
    Particle<ArticulationModelPtr> p = *it;
    p.weight = 1.0/(double)all_particles_number;
    particles.push_back(p);
  }
  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles_prismatic.begin(); it != particles_prismatic.end();
       it++)
  {
    Particle<ArticulationModelPtr> p = *it;
    p.weight = 1.0/(double)all_particles_number;
    particles.push_back(p);
  }
  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles_rotational.begin(); it != particles_rotational.end();
       it++)
  {
    Particle<ArticulationModelPtr> p = *it;
    p.weight = 1.0/(double)all_particles_number;
    particles.push_back(p);
  }
  if (logLikelihoods_)
  {
    weightsToLogWeights(particles);
  }

}



template <class ParticleType> bool ParticleFilter<ParticleType>::normalize(std::vector<Particle <ParticleType> >& particles, const bool& visualization_only)
{
  if (logLikelihoods_)
    {
    if (!normalizeLogWeights(particles, visualization_only))
      {
        return false;
      }
    }
  else
    {
    if (!normalizeWeights(particles))
      {
        return false;
      }
    }
  ROS_INFO ("switching to normal likelihoods for resampling");
//  printParticles(particles);
  return true;
}

template <class ParticleType> bool ParticleFilter<ParticleType>::stratifiedResample(const int& particles_number, std::vector<Particle <ParticleType> >& particles)
{
  //at this point these are normal likelihoods, not log
  typename std::vector <Particle <ParticleType> > new_particles;
  new_particles.reserve(particles_number);
  sortParticles(particles);
  std::vector <double> cumultative_weights;
  double cumultative_weight = 0;
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    cumultative_weight += it->weight;
    cumultative_weights.push_back(cumultative_weight);
  }
  double random_double = Random::uniform(0.0, 1.0/(double)particles_number);
  int idx = 0;
  for (int i = 0; i < particles_number; i++)
  {
    while (random_double > cumultative_weights[idx])
    {
      ++idx;
    }
    new_particles.push_back(particles[idx]);
    new_particles.back().weight = 1.0 / particles_number;
    random_double += 1.0/(double)particles_number;
  }
  particles = new_particles;

  if (logLikelihoods_)
    weightsToLogWeights(particles);
  return true;
}

template <class ParticleType> bool ParticleFilter<ParticleType>::resample(const int& particles_number, std::vector<Particle <ParticleType> >& particles)
{
  //at this point these are normal likelihoods, not log
  typename std::vector <Particle <ParticleType> > new_particles;
  new_particles.reserve(particles_number);
  sortParticles(particles);
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

  if (logLikelihoods_)
    weightsToLogWeights(particles);
  return true;
}

template <class ParticleType> void ParticleFilter<ParticleType>::logWeightsToWeights(std::vector<Particle <ParticleType> >& particles)
{
  ROS_INFO ("switching to normal likelihoods");
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    it->weight = exp(it->weight);
  }
}

template <class ParticleType> void ParticleFilter<ParticleType>::weightsToLogWeights(std::vector<Particle <ParticleType> >& particles)
{
  ROS_INFO ("switching to log likelihoods");
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
       it++)
  {
    it->weight = log(it->weight);
  }
}


template <> void ParticleFilter<ArticulationModelPtr>::removeAddedParticleFlags(std::vector<Particle <ArticulationModelPtr> >& particles)
{
  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
        it++)
  {
    it->state->setParam("added", 0, articulation_model_msgs::ParamMsg::PRIOR);
  }
}

//particles have to be normalized
template <class ParticleType> double ParticleFilter<ParticleType>::calculateEntropy(const std::vector<Particle <ParticleType> >& particles) const
{
  double sum = 0;
  for (typename std::vector <Particle <ParticleType> >::const_iterator it = particles.begin(); it != particles.end();
        it++)
  {
    if (logLikelihoods_)
    {
      sum += exp(it->weight)*log2(exp(it->weight));
    }
    else
    {
      sum += it->weight*log2(it->weight);
    }
  }
  return -sum;
}




//TODO: add free model
template <> void ParticleFilter<ArticulationModelPtr>::addParticles(const articulation_model_msgs::TrackMsg& uptodate_track, const int& rigid_particles_number, const int& rotational_particles_number, const int& prismatic_particles_number)
{
  articulation_model_msgs::ModelMsg model;
  model.track.pose = uptodate_track.pose;//particles.back().state->getModel().track.pose;
  model.track.header.frame_id = uptodate_track.header.frame_id;//particles.back().state->getModel().track.header.frame_id;
  model.track.header.seq = 0;
  model.track.header.stamp = ros::Time::now();

  const double standard_weight = log(1.0/(double)particles.size());
  int rigid_particles_counter = 0;
  int rotational_particles_counter = 0;
  int prismatic_particles_counter = 0;

  while(true)
  {
    if (rigid_particles_counter < rigid_particles_number)
    {
      ArticulationModelPtr rigid_model(new RigidModel);
      model.name = "rigid";
      rigid_model->setModel(model);
      rigid_model->fitModel();
      ++rigid_particles_counter;
      Particle<ArticulationModelPtr> p;
      p.state = rigid_model;
      p.weight = standard_weight;
//      rigid_model->evaluateModel();
//      p.weight = standard_weight + rigid_model->getParam("loglikelihood");
      p.state->setParam("added",1,articulation_model_msgs::ParamMsg::PRIOR);
      particles.push_back(p);
    }
    if (prismatic_particles_counter < prismatic_particles_number)
    {
      ArticulationModelPtr prismatic_model(new PrismaticModel);
      model.name = "prismatic";
      prismatic_model->setModel(model);
      prismatic_model->fitModel();
      ++prismatic_particles_counter;
      Particle<ArticulationModelPtr> p;
      p.state = prismatic_model;
      p.weight = standard_weight;
//      prismatic_model->evaluateModel();
//      p.weight = standard_weight + prismatic_model->getParam("loglikelihood");
      p.state->setParam("added",1, articulation_model_msgs::ParamMsg::PRIOR);
      particles.push_back(p);
    }
    if (rotational_particles_counter < rotational_particles_number)
    {
      ArticulationModelPtr rotational_model(new RotationalModel);
      model.name = "rotational";
      rotational_model->setModel(model);
      rotational_model->fitModel();
      ++rotational_particles_counter;
      Particle<ArticulationModelPtr> p;
      p.state = rotational_model;
      p.weight = standard_weight;
//      rotational_model->evaluateModel();
//      p.weight = standard_weight + rotational_model->getParam("loglikelihood");
      p.state->setParam("added",1,articulation_model_msgs::ParamMsg::PRIOR);
      particles.push_back(p);
    }
    if ((rigid_particles_counter == rigid_particles_number) && (prismatic_particles_counter == prismatic_particles_number) &&
        (rotational_particles_counter == rotational_particles_number))
    {
      break;
    }
  }

}

//particles have to normalized(loglikelihoods only)
template <>  template <class ZType, class AType>
double ParticleFilter<ArticulationModelPtr>::calculateExpectedZaArticulation(std::vector<Particle <ArticulationModelPtr> >& particles, const ZType z, const AType a, const Eigen::MatrixXd& noiseCov,
                     const SensorActionModel<ArticulationModelPtr, ZType, AType> &model)
{
  double z_exp = 0;
  for (std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
        it++)
  {
    if (it->state->model == RIGID)
    {
      z_exp += 0;
    }
    else
    {
      if (logLikelihoods_)
      {
        z_exp += exp( model.senseLogLikelihood(z, a, it->state, noiseCov) + it->weight);
      }
      else
      {
        ROS_ERROR ("This function works only for loglikelihoods!");
      }
    }
  }
  return z_exp;
}


template <class ParticleType> void ParticleFilter<ParticleType>::propagate(std::vector<Particle <ParticleType> >& particles, const Eigen::VectorXd& u, const Eigen::MatrixXd& noiseCov,
                       const MotionModel<ParticleType>& model)
{
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
                       it++)
  {
    Eigen::VectorXd noise = Random::multivariateGaussian(noiseCov);
    it->state = model.move(it->state, u, noise);
  }
}

template <class ParticleType>  template <class ZType>
void ParticleFilter<ParticleType>::correct(std::vector<Particle <ParticleType> >& particles, const ZType z, const Eigen::MatrixXd& noiseCov,
                     const SensorModel<ParticleType, ZType> &model)
{
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
                       it++)
  {
    if (!logLikelihoods_)
      it->weight *= model.senseLikelihood(z, it->state, noiseCov);
    else
      it->weight += model.senseLogLikelihood(z, it->state, noiseCov);
  }
}

template <class ParticleType>  template <class ZType, class AType>
void ParticleFilter<ParticleType>::correctAction(std::vector<Particle <ParticleType> >& particles, const ZType z, const AType a, const Eigen::MatrixXd& noiseCov,
                     const SensorActionModel<ParticleType, ZType, AType> &model)
{
  for (typename std::vector <Particle <ParticleType> >::iterator it = particles.begin(); it != particles.end();
                       it++)
  {
    if (!logLikelihoods_)
      it->weight *= model.senseLikelihood(z, a, it->state, noiseCov);
    else
      it->weight += model.senseLogLikelihood(z, a, it->state, noiseCov);
  }
}

template class ParticleFilter <Eigen::VectorXd>;
template class ParticleFilter <ArticulationModelPtr>;
template void ParticleFilter<Eigen::VectorXd>::correct(std::vector<Particle <Eigen::VectorXd> >& particles, const Eigen::VectorXd z, const Eigen::MatrixXd& noiseCov,
                                                       const SensorModel<Eigen::VectorXd, Eigen::VectorXd> &model);
template void ParticleFilter<ArticulationModelPtr>::correct(std::vector<Particle <ArticulationModelPtr> >& particles, const Eigen::VectorXd z, const Eigen::MatrixXd& noiseCov,
                                                       const SensorModel<ArticulationModelPtr, Eigen::VectorXd> &model);
template void ParticleFilter<ArticulationModelPtr>::correct(std::vector<Particle <ArticulationModelPtr> >& particles, const articulation_model_msgs::TrackMsg z, const Eigen::MatrixXd& noiseCov,
                                                       const SensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg> &model);
template void ParticleFilter<ArticulationModelPtr>::correctAction(std::vector<Particle <ArticulationModelPtr> >& particles, const int z, const ActionPtr a, const Eigen::MatrixXd& noiseCov,
                                                       const SensorActionModel<ArticulationModelPtr, int, ActionPtr> &model);
template double ParticleFilter<ArticulationModelPtr>::calculateExpectedZaArticulation(std::vector<Particle <ArticulationModelPtr> >& particles, const int z, const ActionPtr a, const Eigen::MatrixXd& noiseCov,
                                                       const SensorActionModel<ArticulationModelPtr, int, ActionPtr> &model);

