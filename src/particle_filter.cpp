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
      ArticulationModelPtr free_model(new FreeModel);
      model.name = "free";
      free_model->setModel(model);
      it->state = free_model;
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
  logLikelihoods_(true),freemodel_samples_(1)
{
  this->particles.resize(size);
  const double remaining_models_temp = static_cast<double> ((size - freemodel_samples_) / (MODELS_NUMBER - 1));
  const int remaining_models = static_cast<uint> (remaining_models_temp);
  const int fitmodels_number = 300; //9
  uint i = 0;
  uint rigid_models_counter = 0;
  uint rotational_models_counter = 0;
  uint prismatic_models_counter = 0;

  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin();
    it != particles.end(); it++, i++)
  {
    if (i < freemodel_samples_)
    {
      ArticulationModelPtr free_model(new FreeModel);
      model.name = "free";
      free_model->setModel(model);
      it->state = free_model;
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
        ROS_INFO("using motion model for initalization");
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
        ROS_INFO("using motion model for initalization");
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
        ROS_INFO("using motion model for initalization");
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
      it->weight_to_print_only = it->weight;

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
//      it->weight_to_print_only = it->weight;
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


template <> void ParticleFilter<ArticulationModelPtr>::particlesToDataPoints (const std::vector<Particle <ArticulationModelPtr> >& particles, std::vector<WeightedDataPoint>& data_points, const double weight_multiplier)
{
  data_points.clear();
  for (typename std::vector <Particle <ArticulationModelPtr> >::const_iterator it = particles.begin(); it != particles.end();
       it++)
  {
    WeightedDataPoint dp;
    if (logLikelihoods_)
    {
      dp.weight = exp(it->weight) * weight_multiplier;
    }
    else
    {
      dp.weight = it->weight * weight_multiplier;
    }


    switch (it->state->model)
    {
      case (RIGID):
      {
        Eigen::VectorXd params = Eigen::VectorXd(6);
        boost::shared_ptr<RigidModel> rigid_model = boost::dynamic_pointer_cast< RigidModel > (it->state);
        params(0) = rigid_model->pos_x;
        params(1) = rigid_model->pos_y;
        params(2) = rigid_model->pos_z;
        params(3) = rigid_model->roll;
        params(4) = rigid_model->pitch;
        params(5) = rigid_model->yaw;
        dp.data_point = params;
        data_points.push_back(dp);
        break;
      }
      case (PRISMATIC):
      {
        Eigen::VectorXd params = Eigen::VectorXd(9);
        boost::shared_ptr<PrismaticModel> prismatic_model = boost::dynamic_pointer_cast< PrismaticModel > (it->state);
        params(0) = prismatic_model->pos_x;
        params(1) = prismatic_model->pos_y;
        params(2) = prismatic_model->pos_z;
        params(3) = prismatic_model->roll;
        params(4) = prismatic_model->pitch;
        params(5) = prismatic_model->yaw;
        params(6) = prismatic_model->axis_x;
        params(7) = prismatic_model->axis_y;
        params(8) = prismatic_model->axis_z;
        dp.data_point = params;
        data_points.push_back(dp);
        break;
      }
      case (ROTATIONAL):
      {
        Eigen::VectorXd params = Eigen::VectorXd(10);
        boost::shared_ptr<RotationalModel> rotational_model = boost::dynamic_pointer_cast< RotationalModel > (it->state);
        params(0) = rotational_model->rot_center_x;
        params(1) = rotational_model->rot_center_y;
        params(2) = rotational_model->rot_center_z;
        params(3) = rotational_model->roll;
        params(4) = rotational_model->pitch;
        params(5) = rotational_model->yaw;
        params(6) = rotational_model->axis_roll;
        params(7) = rotational_model->axis_pitch;
        params(8) = rotational_model->axis_yaw;
        params(9) = rotational_model->radius;
        dp.data_point = params;
        data_points.push_back(dp);
        break;
      }
     }
  }
}


// TODO: think of free model
template <> void ParticleFilter<ArticulationModelPtr>::splitArticulationModels(const std::vector<Particle <ArticulationModelPtr> >& particles, double &weights_rigid, double &weights_prismatic, double &weights_rotational, double &weights_free)
{
  particles_rigid.clear();
  particles_prismatic.clear();
  particles_rotational.clear();
  particles_free.clear();
  weights_rigid = 0;
  weights_prismatic = 0;
  weights_rotational = 0;
  weights_free = 0;

  for (typename std::vector <Particle <ArticulationModelPtr> >::const_iterator it = particles.begin(); it != particles.end();
       it++)
  {

    switch (it->state->model)
    {
      case (RIGID):
      {
        Particle<ArticulationModelPtr> p = *it;
        particles_rigid.push_back(p);
        weights_rigid += exp(p.weight);
        break;
      }
      case (PRISMATIC):
      {
        Particle<ArticulationModelPtr> p = *it;
        particles_prismatic.push_back(p);
        weights_prismatic += exp(p.weight);
        break;
      }
      case (ROTATIONAL):
      {
        Particle<ArticulationModelPtr> p = *it;
        particles_rotational.push_back(p);
        weights_rotational += exp(p.weight);
        break;
      }
      case (FREE):
      {
        Particle<ArticulationModelPtr> p = *it;
        particles_free.push_back(p);
        weights_free += exp(p.weight);
        break;
      }
    }
  }

}

// TODO: think of free model
template <> void ParticleFilter<ArticulationModelPtr>::mergeArticulationModels()
{
  particles.clear();
  const int all_particles_number = particles_rigid.size() + particles_prismatic.size() + particles_rotational.size() + particles_free.size();

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
  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles_free.begin(); it != particles_free.end();
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
  if (!visualization_only)
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

template <> void ParticleFilter<ArticulationModelPtr>::printStatistics(const std::vector<Particle <ArticulationModelPtr> >& particles) const
{
  uint rigid_counter = 0;
  uint rotational_counter = 0;
  uint prismatic_counter = 0;
  uint free_counter = 0;
  for (typename std::vector <Particle <ArticulationModelPtr> >::const_iterator it = particles.begin(); it != particles.end();
        it++)
  {
    switch(it->state->model)
    {
      case (RIGID):
      {
        ++rigid_counter;
        break;
      }
      case (PRISMATIC):
      {
        ++prismatic_counter;
        break;
      }
      case (ROTATIONAL):
      {
        ++rotational_counter;
        break;
      }
      case (FREE):
      {
        ++free_counter;
        break;
      }
    }
  }
  double sum_all = rigid_counter + prismatic_counter + rotational_counter + free_counter;
  double rigid_percentage = rigid_counter/sum_all * 100;
  double prismatic_percentage = prismatic_counter/sum_all * 100;
  double rotational_percentage = rotational_counter/sum_all * 100;
  double free_percentage = free_counter/sum_all * 100;

  ROS_ERROR("STATS: \n RIGID: %d (%f %%) \n PRISMATIC: %d (%f %%) \n ROTATIONAL: %d (%f %%) \n FREE: %d (%f %%) \n", rigid_counter, rigid_percentage, prismatic_counter, prismatic_percentage, rotational_counter, rotational_percentage, free_counter, free_percentage);

  double entropy = 0;
  double rigid_entropy = 0;
  double prismatic_entropy = 0;
  double rotational_entropy = 0;
  double free_entropy = 0;
  if (rigid_percentage > 0)
    rigid_entropy = rigid_percentage/100*log2(rigid_percentage/100);
  if (prismatic_percentage > 0)
    prismatic_entropy = prismatic_percentage/100*log2(prismatic_percentage/100);
  if (rotational_percentage > 0)
    rotational_entropy = rotational_percentage/100*log2(rotational_percentage/100);
  if (free_entropy > 0)
    free_entropy = free_percentage/100*log2(free_percentage/100);

  entropy = rigid_entropy + prismatic_entropy + rotational_entropy + free_entropy;

  ROS_ERROR("ENTROPY OF MODELS: %f", -entropy);

//  ROS_ERROR_STREAM("STATISTICS: \n" << "RIGID: " << rigid_percentage << "\n PRISMATIC: " << prismatic_percentage << "\n ROTATIONAL: " << rotational_percentage <<  "\n FREE: " << free_percentage << "\n");
}


template <> void ParticleFilter<ArticulationModelPtr>::removeAddedParticleFlags(std::vector<Particle <ArticulationModelPtr> >& particles)
{
  for (typename std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
        it++)
  {
    it->state->setParam("added", 0, articulation_model_msgs::ParamMsg::PRIOR);
  }
}

//assumes only one type of particles
template <> double ParticleFilter<ArticulationModelPtr>::calculateHMean(const std::vector<Particle <ArticulationModelPtr> >& particles)
{
  Model model;
  double d = 0;
  switch(particles[0].state->model)
  {
    case (RIGID):
    {
      model = RIGID;
      d = 6;
      break;
    }
    case (PRISMATIC):
    {
      model = PRISMATIC;
      d = 8;
      break;
    }
    case (ROTATIONAL):
    {
      model = ROTATIONAL;
      d = 9;
      break;
    }
    case (FREE):
    {
      return 0.0;
      break;
    }
  }

  tf::Vector3 rot_center, rot_center_comp, rigid_position, rigid_position_comp, prismatic_dir, prismatic_dir_comp;
  tf::Quaternion rot_axis, rot_axis_comp;
  tf::Quaternion rot_orientation, rot_orientation_comp, rigid_orientation, rigid_orientation_comp;
  double rot_radius, rot_radius_comp;

  int n = particles.size();
  double H_mean = 0;

  for (typename std::vector <Particle <ArticulationModelPtr> >::const_iterator it = particles.begin(); it != particles.end();
      it++)
  {
    for (typename std::vector <Particle <ArticulationModelPtr> >::const_iterator it_comp = particles.begin(); it_comp != particles.end();
         it_comp++)
    {
      if (it == it_comp)
        continue;

      if(model == ROTATIONAL)
      {
        it->state->getParam("rot_center",rot_center);
        it->state->getParam("rot_axis",rot_axis);
        it->state->getParam("rot_radius",rot_radius);
        it->state->getParam("rot_orientation",rot_orientation);

        it_comp->state->getParam("rot_center",rot_center_comp);
        it_comp->state->getParam("rot_axis",rot_axis_comp);
        it_comp->state->getParam("rot_radius",rot_radius_comp);
        it_comp->state->getParam("rot_orientation",rot_orientation_comp);

        rot_orientation.normalize();
        rot_orientation_comp.normalize();
        rot_axis.normalize();
        rot_axis_comp.normalize();

        H_mean += log(rot_center.distance(rot_center_comp) + rot_axis.angleShortestPath(rot_axis_comp) + fabs(rot_radius - rot_radius_comp) + rot_orientation.angleShortestPath(rot_orientation_comp));
      }
      else if (model == PRISMATIC)
      {
        it->state->getParam("rigid_position",rigid_position);
        it->state->getParam("rigid_orientation",rigid_orientation);

        it_comp->state->getParam("rigid_position",rigid_position_comp);
        it_comp->state->getParam("rigid_orientation",rigid_orientation_comp);

        rigid_orientation.normalize();
        rigid_orientation_comp.normalize();

        H_mean += log(rigid_position.distance(rigid_position_comp) + rigid_orientation.angleShortestPath(rigid_orientation_comp));
      }
      else if (model == RIGID)
      {
        it->state->getParam("rigid_position",rigid_position);
        it->state->getParam("rigid_orientation",rigid_orientation);
        it->state->getParam("prismatic_dir",prismatic_dir);

        it_comp->state->getParam("rigid_position",rigid_position_comp);
        it_comp->state->getParam("rigid_orientation",rigid_orientation_comp);
        it_comp->state->getParam("prismatic_dir",prismatic_dir_comp);

        rigid_orientation.normalize();
        rigid_orientation_comp.normalize();

        H_mean += log(rigid_position.distance(rigid_position_comp) + rigid_orientation.angleShortestPath(rigid_orientation_comp) + prismatic_dir.distance(prismatic_dir_comp));
      }
    }
  }

  return H_mean * d / (n * (n-1));
}


template <> double ParticleFilter<ArticulationModelPtr>::calculateAverageHMean(const std::vector<Particle <ArticulationModelPtr> >& particles)
{
  double weights_rigid, weights_prismatic, weights_rotational, weights_free;
  splitArticulationModels(particles, weights_rigid, weights_prismatic, weights_rotational, weights_free);
  double weights_sum = weights_rigid + weights_prismatic + weights_rotational + weights_free;
  ROS_INFO("weights: rigid: %f, prismatic: %f, rotational: %f, free: %f", weights_rigid, weights_prismatic, weights_rotational, weights_free);
  double h_mean_rigid = 0;
  double h_mean_free = 0;
  double h_mean_prismatic = 0;
  double h_mean_rotational = 0;

  if (particles_rigid.size() > 0)
    h_mean_rigid = calculateHMean(particles_rigid);
  if (particles_rotational.size() > 0)
    h_mean_rotational = calculateHMean(particles_rotational);
  if (particles_prismatic.size() > 0)
    h_mean_prismatic = calculateHMean(particles_prismatic);
  ROS_INFO("H_MEANs: rigid: %f, prismatic: %f, rotational: %f", h_mean_rigid, h_mean_prismatic, h_mean_rotational);


  return weights_rigid/weights_sum * h_mean_rigid + weights_prismatic/weights_sum * h_mean_prismatic + weights_rotational/weights_sum * h_mean_rotational + weights_free/weights_sum;
}



template <> double ParticleFilter<ArticulationModelPtr>::calculateKDEEntropy(const std::vector<Particle <ArticulationModelPtr> >& particles)
{
  double weights_rigid, weights_prismatic, weights_rotational, weights_free;
  splitArticulationModels(particles, weights_rigid, weights_prismatic, weights_rotational, weights_free);
  double weights_sum = weights_rigid + weights_prismatic + weights_rotational + weights_free;
  ROS_INFO("weights: rigid: %f, prismatic: %f, rotational: %f, free: %f", weights_rigid, weights_prismatic, weights_rotational, weights_free);

  std::vector<WeightedDataPoint> rigid_dps;
  if (particles_rigid.size() > 0)
    particlesToDataPoints(particles_rigid, rigid_dps);
  std::vector<WeightedDataPoint> prismatic_dps;
  if (particles_prismatic.size() > 0)
    particlesToDataPoints(particles_prismatic, prismatic_dps);
  std::vector<WeightedDataPoint> rotational_dps;
  if (particles_rotational.size() > 0)
    particlesToDataPoints(particles_rotational, rotational_dps);




  KernelEstimator kernel_estimator;
  Eigen::MatrixXd H_rigid, H_prismatic, H_rotational;
  double entropy_rigid = 0;
  double entropy_prismatic = 0;
  double entropy_rotational = 0;
  double entropy_rigid_weighted = 0;
  double entropy_prismatic_weighted = 0;
  double entropy_rotational_weighted = 0;

  if (rigid_dps.size() > 0)
  {
    if (kernel_estimator.estimateWeightedH(rigid_dps, H_rigid))
    {
      entropy_rigid = kernel_estimator.estimateWeightedEntropyKernelND(rigid_dps, "gaussian", H_rigid);
      entropy_rigid_weighted = kernel_estimator.estimateWeightedEntropyKernelND(rigid_dps, "gaussian", H_rigid, weights_rigid/weights_sum);
  //    std::cerr << "estimated H rigid: \n" << H_rigid << std::endl;
    }
  }

  if (prismatic_dps.size() > 0)
  {
    if(kernel_estimator.estimateWeightedH(prismatic_dps, H_prismatic))
    {
      entropy_prismatic = kernel_estimator.estimateWeightedEntropyKernelND(prismatic_dps, "gaussian", H_prismatic);
      entropy_prismatic_weighted = kernel_estimator.estimateWeightedEntropyKernelND(prismatic_dps,"gaussian",H_prismatic, weights_prismatic/weights_sum);
//      double entropy_prismatic_weights_0_5 = kernel_estimator.estimateWeightedEntropyKernelND(prismatic_dps,"gaussian",H_prismatic, 0.5);
//      double entropy_prismatic_weights_0_9 = kernel_estimator.estimateWeightedEntropyKernelND(prismatic_dps,"gaussian",H_prismatic, 0.9);
//      std::cerr << "entropy prismatic with prismatic filter weights 0.5: " << entropy_prismatic_weights_0_5 << std::endl;
//      std::cerr << "entropy prismatic with prismatic filter weights 0.9: " << entropy_prismatic_weights_0_9 << std::endl;
      //  std::cerr << "estimated H_prismatic: \n" << H_prismatic << std::endl;
    }
  }

  if (rotational_dps.size() > 0)
  {
    if(kernel_estimator.estimateWeightedH(rotational_dps, H_rotational))
    {
      entropy_rotational = kernel_estimator.estimateWeightedEntropyKernelND(rotational_dps,"gaussian",H_rotational);
      entropy_rotational_weighted = kernel_estimator.estimateWeightedEntropyKernelND(rotational_dps,"gaussian",H_rotational, weights_rotational/weights_sum);

//      double entropy_rotational_weights_0_5 = kernel_estimator.estimateWeightedEntropyKernelND(rotational_dps,"gaussian",H_rotational, 0.5);
//      double entropy_rotational_weights_0_1 = kernel_estimator.estimateWeightedEntropyKernelND(rotational_dps,"gaussian",H_rotational, 0.1);
//      std::cerr << "entropy rotational with rotational filter weights 0.5: " << entropy_rotational_weights_0_5 << std::endl;
//      std::cerr << "entropy rotational with rotational filter weights 0.1: " << entropy_rotational_weights_0_1 << std::endl;
      //  std::cerr << "estimated H_rotational: \n" << H_rotational << std::endl;
    }
  }

  double entropy_free = 0;
  if (particles_free.size()>0)
    entropy_free = calculateEntropy(particles_free);

  ROS_INFO("ENTROPIES: rigid: %f, prismatic: %f, rotational: %f, free: %f", entropy_rigid, entropy_prismatic, entropy_rotational, entropy_free);
  ROS_INFO("ENTROPIES WITH DENSITY: rigid: %f, prismatic: %f, rotational: %f, free: %f", entropy_rigid_weighted, entropy_prismatic_weighted, entropy_rotational_weighted, entropy_free);


//  double entropy = (weights_rigid/weights_sum)*entropy_rigid + (weights_prismatic/weights_sum)*entropy_prismatic
//                   + (weights_rotational/weights_sum)*entropy_rotational + (weights_free/weights_sum)*entropy_free;
  double entropy = entropy_rigid_weighted + entropy_prismatic_weighted + entropy_rotational_weighted + entropy_free;

  return entropy;
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
      if (!isinf(it->weight))
      {
        sum += exp(it->weight)*log2(exp(it->weight));
      }
      else
      {
        sum += 0;
      }
    }
    else
    {
      sum += it->weight*log2(it->weight);
    }
  }
  return -sum;
}




template <> void ParticleFilter<ArticulationModelPtr>::addParticles(const articulation_model_msgs::TrackMsg& uptodate_track, const int& rigid_particles_number, const int& rotational_particles_number, const int& prismatic_particles_number, const int& free_particles_number)
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
  int free_particles_counter = 0;


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
    if (free_particles_counter < free_particles_number)
    {
      ArticulationModelPtr free_model(new FreeModel);
      model.name = "free";
      Particle<ArticulationModelPtr> p;
      p.state = free_model;
      p.weight = standard_weight;
      ++free_particles_counter;
//      rotational_model->evaluateModel();
//      p.weight = standard_weight + rotational_model->getParam("loglikelihood");
      p.state->setParam("added",1,articulation_model_msgs::ParamMsg::PRIOR);
      particles.push_back(p);
    }


    if ((rigid_particles_counter == rigid_particles_number) && (prismatic_particles_counter == prismatic_particles_number) &&
        (rotational_particles_counter == rotational_particles_number) && (free_particles_counter == free_particles_number))
    {
      break;
    }
  }

}

template <>  template <class ZType, class AType>
double ParticleFilter<ArticulationModelPtr>::calculateKLdivergence(std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const AType a, const Eigen::MatrixXd& noiseCov,
                     const SensorActionModel<ArticulationModelPtr, ZType, AType> &model)
{
  double prob_exp = 0;
  double prob_likelihood = 0;
  double result = 0;
  for (std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
        it++)
  {
    if (logLikelihoods_)
    {
      if (it->state->model == RIGID)
      {
        prob_exp = 1-z_exp;
      }
      else
      {
        int z_prob = 1;
        prob_likelihood = exp(model.senseLogLikelihood(z_prob, a, it->state, noiseCov));
//        ROS_INFO("model = %s, weight = %f, prob_likelihood = %f", it->state->model_msg.name.c_str(), exp(it->weight), prob_likelihood);
        prob_exp = (z_exp * prob_likelihood) + ((1 - z_exp) * (1 - prob_likelihood));
      }
      it->expected_weight = it->weight + log(prob_exp);
//      ROS_INFO("it->weight: %f", it->weight);
      result += log(it->weight / it->expected_weight) * it->weight;
    }
    else
    {
      ROS_ERROR ("This function works only for loglikelihoods!");
    }
  }

  return result;
}


template <>  template <class ZType, class AType>
double ParticleFilter<ArticulationModelPtr>::calculateExpectedKDEEntropy(std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const AType a, const Eigen::MatrixXd& noiseCov,
                     const SensorActionModel<ArticulationModelPtr, ZType, AType> &model)
{
  ROS_ERROR("If Z=1 (thing moves)");
  double entropy_1 = calculateExpectedMeasurementKDEEntropy(particles, 1, a, noiseCov, model);
  ROS_ERROR("If Z=0 (thing doesnt move)");
  double entropy_0 = calculateExpectedMeasurementKDEEntropy(particles, 0, a, noiseCov, model);
  ROS_INFO("entropy 1: %f, entropy 0: %f", entropy_1, entropy_0);

  return z_exp*entropy_1 + (1-z_exp)*entropy_0;

}

template <>  template <class ZType, class AType>
double ParticleFilter<ArticulationModelPtr>::calculateExpectedMeasurementKDEEntropy(std::vector<Particle <ArticulationModelPtr> > particles, const double z_exp, const AType a, const Eigen::MatrixXd& noiseCov,
                     const SensorActionModel<ArticulationModelPtr, ZType, AType> &model)
{
  double prob_exp = 0;
  double prob_likelihood = 0;
  for (std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
        it++)
  {
    if (logLikelihoods_)
    {
      if (it->state->model == RIGID)
      {
        prob_exp = 1-z_exp;
      }
      else
      {
        int z_prob = 1;
        prob_likelihood = exp(model.senseLogLikelihood(z_prob, a, it->state, noiseCov));
//        ROS_INFO("model = %s, weight = %f, prob_likelihood = %f", it->state->model_msg.name.c_str(), exp(it->weight), prob_likelihood);
        prob_exp = (z_exp * prob_likelihood) + ((1 - z_exp) * (1 - prob_likelihood));
      }
      // HACK: expected weight is equal to normal weight and normal is equal to expected, because normalization can be faster this way
      it->expected_weight = it->weight;
      it->weight = it->weight + log(prob_exp);
//      ROS_INFO("it->weight: %f", it->weight);
    }
    else
    {
      ROS_ERROR ("This function works only for loglikelihoods!");
    }
  }
  //TODO: think if needed
  normalize(particles);
  weightsToLogWeights(particles);
  double result = calculateKDEEntropy(particles);


//  double temp_weight_holder = 0;
//  for (std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
//        it++)
//  {
//    // HACK: switch weights back to normal after previous hack
//    temp_weight_holder = it->expected_weight;
//    it->expected_weight = it->weight;
//    it->weight = temp_weight_holder;
//  }

  return result;
}



// 1 - doesnt stop
template <>  template <class ZType, class AType>
double ParticleFilter<ArticulationModelPtr>::calculateExpectedDownweightAfterAction(std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const AType a, const Eigen::MatrixXd& noiseCov,
                     const SensorActionModel<ArticulationModelPtr, ZType, AType> &model)
{
  double change_in_log_mass = 0;
  double prob_exp = 0;
  double prob_likelihood = 0;
  for (std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
        it++)
  {
    if (logLikelihoods_)
    {
      if (it->state->model == RIGID)
      {
        prob_exp = 1-z_exp;
      }
      else
      {
        int z_prob = 1;
        prob_likelihood = exp(model.senseLogLikelihood(z_prob, a, it->state, noiseCov));
        prob_exp = (z_exp * prob_likelihood) + ((1 - z_exp) * (1 - prob_likelihood));
      }
      // HACK: expected weight is equal to normal weight and normal is equal to expected, because normalization can be faster this way
      it->expected_weight = it->weight;
      it->weight = it->weight + log(prob_exp);
    }
    else
    {
      ROS_ERROR ("This function works only for loglikelihoods!");
    }
  }
  normalize(particles);
  weightsToLogWeights(particles);
  int particles_left = particles.size();

  double temp_weight_holder = 0;
  for (std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
        it++, --particles_left)
  {
    // HACK: switch weights back to normal after previous hack
    temp_weight_holder = it->expected_weight;
    it->expected_weight = it->weight;
    it->weight = temp_weight_holder;

    if (particles_left <= 3)
    {
      ROS_ERROR("exp(it->weight) : %f", exp(it->weight));
      ROS_ERROR("exp(it->expected_weight) : %f", exp(it->expected_weight));
      ROS_ERROR("fabs(exp(it->weight) - exp(it->expected_weight)) : %f \n", fabs(exp(it->weight) - exp(it->expected_weight)));
    }
//    if (!isinf(it->weight) && !isinf(it->expected_weight))
//    {
      change_in_log_mass += fabs(exp(it->weight) - exp(it->expected_weight));
//    }
//    else
//    {
      //TODO: think if this is correct
//      change_in_log_mass += 0;
//    }
  }

  return change_in_log_mass;
}


template <>  template <class ZType, class AType>
double ParticleFilter<ArticulationModelPtr>::calculateExpectedEntropy(std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const AType a, const Eigen::MatrixXd& noiseCov,
                     const SensorActionModel<ArticulationModelPtr, ZType, AType> &model)
{
  double prob_exp = 0;
  double sum = 0;
  double log2part = 0;
  double prob_likelihood = 0;
  double sum_to_print = 0;
  int particles_left = particles.size();

  for (std::vector <Particle <ArticulationModelPtr> >::iterator it = particles.begin(); it != particles.end();
        it++, --particles_left)
  {
    if (it->state->model == RIGID)
    {
      prob_exp = 1-z_exp;
    }
    else
    {
      if (logLikelihoods_)
      {
        int z_prob = 1;
        prob_likelihood = exp(model.senseLogLikelihood(z_prob, a, it->state, noiseCov));
        prob_exp = (z_exp * prob_likelihood) + ((1 - z_exp) * (1 - prob_likelihood));
      }
      else
      {
        ROS_ERROR ("This function works only for loglikelihoods!");
      }
    }
    if ( !isinf(it->weight) && !isinf(log(prob_exp)) && prob_exp != 0)
    {
      it->expected_weight = it->weight + log(prob_exp);

      log2part = log2(exp(it->expected_weight));
      if (isinf(log2part))
      {
        sum += 0;
      }
      else
      {
        sum += exp(it->expected_weight)*log2part;
        if (particles_left <= 5)
        {
          ROS_ERROR("prop likelihood : %f", prob_likelihood);
          ROS_ERROR("prop exp: %f", prob_exp);
          ROS_ERROR("it->weight: %f", it->weight);
          ROS_ERROR("log(prob_exp): %f", log(prob_exp));
          ROS_ERROR("it->expected_weight: %f", it->expected_weight);
          ROS_ERROR("particle: %d, sum: %f \n", particles_left, exp(it->expected_weight)*log2part);
          sum_to_print += exp(it->expected_weight)*log2part;
        }
      }
    }
    else
    {
      sum += 0;
    }
  }
  ROS_ERROR("sum of last printed particles: %f \n", sum_to_print);

  return -sum;
}


//particles have to normalized(loglikelihoods only)
template <>  template <class ZType, class AType>
double ParticleFilter<ArticulationModelPtr>::calculateExpectedZaArticulation(std::vector<Particle <ArticulationModelPtr> >& particles, const AType a, const Eigen::MatrixXd& noiseCov,
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
        int z_prob = 1;
        double z = exp( model.senseLogLikelihood(z_prob, a, it->state, noiseCov) + it->weight);
//        ROS_ERROR("prob = %f", z_other);
//        ROS_ERROR("weight = %f", exp(it->weight));
//        ROS_ERROR("z_other = %f", z_other*exp(it->weight));
//        ROS_ERROR("z = %f", z);
        z_exp += z;
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
template double ParticleFilter<ArticulationModelPtr>::calculateExpectedZaArticulation(std::vector<Particle <ArticulationModelPtr> >& particles, const ActionPtr a, const Eigen::MatrixXd& noiseCov,
                                                       const SensorActionModel<ArticulationModelPtr, int, ActionPtr> &model);
template double ParticleFilter<ArticulationModelPtr>::calculateExpectedEntropy(std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const ActionPtr a, const Eigen::MatrixXd& noiseCov,
                                                       const SensorActionModel<ArticulationModelPtr, int, ActionPtr> &model);
template double ParticleFilter<ArticulationModelPtr>::calculateExpectedDownweightAfterAction(std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const ActionPtr a, const Eigen::MatrixXd& noiseCov,
                                                       const SensorActionModel<ArticulationModelPtr, int, ActionPtr> &model);
template double ParticleFilter<ArticulationModelPtr>::calculateExpectedMeasurementKDEEntropy(std::vector<Particle <ArticulationModelPtr> > particles, const double z_exp, const ActionPtr a, const Eigen::MatrixXd& noiseCov,
                                                       const SensorActionModel<ArticulationModelPtr, int, ActionPtr> &model);
template double ParticleFilter<ArticulationModelPtr>::calculateExpectedKDEEntropy(std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const ActionPtr a, const Eigen::MatrixXd& noiseCov,
                                                       const SensorActionModel<ArticulationModelPtr, int, ActionPtr> &model);
template double ParticleFilter<ArticulationModelPtr>::calculateKLdivergence(std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const ActionPtr a, const Eigen::MatrixXd& noiseCov,
                                                       const SensorActionModel<ArticulationModelPtr, int, ActionPtr> &model);
