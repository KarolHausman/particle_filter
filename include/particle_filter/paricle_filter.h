#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle_filter/particle.h"
#include <Eigen/Core>
#include "particle_filter/motion_model.h"
#include "particle_filter/sensor_model.h"

class ParticleFilter
{
public:
  ParticleFilter(const std::vector <Particle>& particles);
  ParticleFilter(const int& size, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);
  virtual ~ParticleFilter();

  bool normalizeWeights();

  void sortParticles();

  double getWeightsSum()const;

  Eigen::VectorXd getWeightedAvg(const double& particles_fraction);

  virtual void propagate(const Eigen::VectorXd& u, const Eigen::MatrixXd& noiseCov,
                         const MotionModel& model);

  virtual void correct(const Eigen::VectorXd z, const Eigen::MatrixXd& noiseCov,
                       const SensorModel& model);

  virtual bool resample(const int& particles_number);

  std::vector <Particle> particles;
};

#endif // PARTICLE_FILTER_H
