#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle_filter/particle.h"
#include <Eigen/Core>
#include "particle_filter/motion_model.h"
#include "particle_filter/sensor_model.h"
#include "particle_filter/sensor_action_model.h"
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
  ParticleFilter(const int &size, articulation_model_msgs::ModelMsg &model);
  ParticleFilter(const int& size, articulation_model_msgs::ModelMsg& model,
                 const MotionModel<ArticulationModelPtr> &motion_model, const Eigen::MatrixXd& rigid_cov,
                 const Eigen::MatrixXd &rotational_cov, const Eigen::MatrixXd& prismatic_cov);


  virtual ~ParticleFilter();

  bool normalize(std::vector<Particle <ParticleType> >& particles, const bool& visualization_only=false);

  void removeAddedParticleFlags(std::vector<Particle <ArticulationModelPtr> >& particles);

  void splitArticulationModels();

  void mergeArticulationModels();

  bool normalizeWeights(std::vector<Particle <ParticleType> >& particles, const bool& visualization_only=false);

  bool normalizeLogWeights(std::vector<Particle <ParticleType> >& particles, const bool& visualization_only=false);

  void sortParticles(std::vector<Particle <ParticleType> >& particles);

  void weightsToLogWeights(std::vector<Particle <ParticleType> >& particles);

  void logWeightsToWeights(std::vector<Particle <ParticleType> >& particles);

  void printParticles( const std::vector<Particle <ParticleType> >& particles) const;

  bool getLogLikelihoodsFlag() const;

//  void setLogLikelihoodsFlag(const bool& flag);

  double getWeightsSum(const std::vector<Particle <ParticleType> >& particles)const;

  Eigen::VectorXd getWeightedAvg(std::vector<Particle <Eigen::VectorXd> >& particles, const double& particles_fraction);

  virtual void propagate(std::vector<Particle <ParticleType> >& particles, const Eigen::VectorXd& u, const Eigen::MatrixXd& noiseCov,
                         const MotionModel<ParticleType>& model);

  template <class ZType> void correct(std::vector<Particle <ParticleType> >& particles, const ZType z, const Eigen::MatrixXd& noiseCov,
                       const SensorModel<ParticleType, ZType>& model);

  template <class ZType, class AType> void correctAction(std::vector<Particle <ParticleType> >& particles, const ZType z, const AType a, const Eigen::MatrixXd& noiseCov,
                       const SensorActionModel<ParticleType, ZType, AType>& model);

  virtual bool resample(const int& particles_number, std::vector<Particle <ParticleType> >& particles);

  virtual bool stratifiedResample(const int& particles_number, std::vector<Particle <ParticleType> >& particles);

  void addParticles(const articulation_model_msgs::TrackMsg& uptodate_track, const int &rigid_particles_number, const int &rotational_particles_number, const int &prismatic_particles_number);

  std::vector <Particle <ParticleType> > particles;
  std::vector <Particle <ArticulationModelPtr> > particles_rigid;
  std::vector <Particle <ArticulationModelPtr> > particles_prismatic;
  std::vector <Particle <ArticulationModelPtr> > particles_rotational;



private:
  bool logLikelihoods_;
  uint freemodel_samples_;

};

#endif // PARTICLE_FILTER_H
