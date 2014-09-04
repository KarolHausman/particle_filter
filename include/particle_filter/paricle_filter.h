#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle_filter/particle.h"
#include <Eigen/Core>
#include "particle_filter/motion_model.h"
#include "particle_filter/sensor_model.h"
#include "particle_filter/sensor_action_model.h"
#include "articulation_model.h"
#include "particle_filter/kernel_density_estimator.h"

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

  void splitArticulationModels(const std::vector<Particle <ArticulationModelPtr> >& particles, double &weights_rigid, double &weights_prismatic, double &weights_rotational, double &weights_free);

  void mergeArticulationModels();

  bool normalizeWeights(std::vector<Particle <ParticleType> >& particles, const bool& visualization_only=false);

  bool normalizeLogWeights(std::vector<Particle <ParticleType> >& particles, const bool& visualization_only=false);

  void sortParticles(std::vector<Particle <ParticleType> >& particles);

  void weightsToLogWeights(std::vector<Particle <ParticleType> >& particles);

  void logWeightsToWeights(std::vector<Particle <ParticleType> >& particles);

  void printParticles( const std::vector<Particle <ParticleType> >& particles) const;

  bool getLogLikelihoodsFlag() const;

  double calculateEntropy (const std::vector<Particle <ParticleType> >& particles) const;

  double calculateKDEEntropy (const std::vector<Particle <ArticulationModelPtr> >& particles);

  void particlesToDataPoints (const std::vector<Particle <ArticulationModelPtr> >& particles, std::vector<WeightedDataPoint>& data_points);

  template <class ZType, class AType> double calculateExpectedKDEEntropy (std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const AType a, const Eigen::MatrixXd& noiseCov,
                                                                              const SensorActionModel<ArticulationModelPtr, ZType, AType>& model);

  template <class ZType, class AType> double calculateExpectedDownweightAfterAction (std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const AType a, const Eigen::MatrixXd& noiseCov,
                                                                              const SensorActionModel<ArticulationModelPtr, ZType, AType>& model);

  template <class ZType, class AType> double calculateExpectedEntropy (std::vector<Particle <ArticulationModelPtr> >& particles, const double z_exp, const AType a, const Eigen::MatrixXd& noiseCov,
                                                                              const SensorActionModel<ArticulationModelPtr, ZType, AType>& model);

  template <class ZType, class AType> double calculateExpectedZaArticulation (std::vector<Particle <ArticulationModelPtr> >& particles, const AType a, const Eigen::MatrixXd& noiseCov,
                                                                              const SensorActionModel<ArticulationModelPtr, ZType, AType>& model);

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

  void addParticles(const articulation_model_msgs::TrackMsg& uptodate_track, const int &rigid_particles_number, const int &rotational_particles_number, const int &prismatic_particles_number, const int& free_particles_number);

  std::vector <Particle <ParticleType> > particles;
  std::vector <Particle <ArticulationModelPtr> > particles_rigid;
  std::vector <Particle <ArticulationModelPtr> > particles_prismatic;
  std::vector <Particle <ArticulationModelPtr> > particles_rotational;
  std::vector <Particle <ArticulationModelPtr> > particles_free;




private:
  bool logLikelihoods_;
  uint freemodel_samples_;

};

#endif // PARTICLE_FILTER_H
