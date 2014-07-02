
#include <iostream>
#include "particle_filter/particle.h"
#include "particle_filter/paricle_filter.h"
#include "particle_filter/identity_motionmodel.h"
#include "particle_filter/gaussian_sensormodel.h"
#include "particle_filter/visualizer.h"


#include "particle_filter/random.h"

int main(int argc, char **argv)
{
  const int particles_number = 100;
  ros::init(argc, argv, "particle_filter");

  Visualizer::getInstance()->init();

  Eigen::Vector3d mean = Eigen::Vector3d::Ones();
  Eigen::Matrix3d cov;
  cov << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
  ParticleFilter pf(particles_number, mean, cov);

  MotionModel* motionModel = new IdentityMotionModel;
  SensorModel* sensorModel = new GaussianSensorModel;

  Eigen::Vector3d u = Eigen::Vector3d::Ones();
  Eigen::Matrix3d motionNoiseCov = cov / 10;

  Eigen::Vector3d z = Eigen::Vector3d::Ones();
  Eigen::Matrix3d sensorNoiseCov = cov / 2;

  std::cout << "probability for mean: " << Random::multivariateGaussianProbability(mean, sensorNoiseCov, mean) << std::endl;

  ros::Rate r(2);
  int loop_count = 1;
  while (ros::ok())
  {
    std::cerr << "loop_count: " << loop_count << std::endl;
    pf.propagate(u, motionNoiseCov, *motionModel);

    if (loop_count % 10 == 0)
    {
      std::cout << "Correction step executed." << std::endl;
      z += (Eigen::VectorXd(3) << 1, 1, 1).finished();
      pf.correct(z, sensorNoiseCov, *sensorModel);
      pf.resample(particles_number);
    }
    Visualizer::getInstance()->publishParticles(pf.particles);
    r.sleep();
    ++loop_count;

    double fraction = 0.02;
    std::cerr << "measurement equals: \n" << z << std::endl;
    std::cerr << "weighted average of " << fraction << " equals: \n" << pf.getWeightedAvg(fraction) << std::endl;
  }
  delete motionModel;
  delete sensorModel;
}
