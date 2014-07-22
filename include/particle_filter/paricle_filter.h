#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle_filter/particle.h"
#include <Eigen/Core>
#include "particle_filter/motion_model.h"
#include "particle_filter/sensor_model.h"
#include "articulation_model.h"

template <class ParticleType>
class ParticleFilter
{
public:
  ParticleFilter(const std::vector <Particle <ParticleType> >& particles);
  ParticleFilter(const int& size, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);
  ParticleFilter(const int& size, const Eigen::VectorXd &rigid_mean, const Eigen::MatrixXd &rigid_cov,
                 const Eigen::VectorXd &rotational_mean, const Eigen::MatrixXd &rotational_cov,
                 const Eigen::VectorXd &prismatic_mean, const Eigen::MatrixXd &prismatic_cov);



  virtual ~ParticleFilter();

  bool normalizeWeights();

  bool normalizeLogWeights();

  void sortParticles();

  void weightsToLogWeights();

  void logWeightsToWeights();

  void printParticles() const;

  bool getLogLikelihoodsFlag() const;

  void setLogLikelihoodsFlag(const bool& flag);

  double getWeightsSum()const;

  Eigen::VectorXd getWeightedAvg(const double& particles_fraction);

  virtual void propagate(const Eigen::VectorXd& u, const Eigen::MatrixXd& noiseCov,
                         const MotionModel<ParticleType>& model);

  virtual void correct(const Eigen::VectorXd z, const Eigen::MatrixXd& noiseCov,
                       const SensorModel<ParticleType>& model);

  virtual bool resample(const int& particles_number);

  std::vector <Particle <ParticleType> > particles;


private:
  bool logLikelihoods_;
  uint freemodel_samples_;

};

#endif // PARTICLE_FILTER_H
