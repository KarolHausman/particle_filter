
#include <iostream>
#include "particle_filter/particle.h"
#include "particle_filter/paricle_filter.h"
#include "particle_filter/identity_motionmodel.h"
#include "particle_filter/gaussian_sensormodel.h"
#include "particle_filter/visualizer.h"

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
  Eigen::Matrix3d motionNoiseCov = cov / 5;

  Eigen::Vector3d z = Eigen::Vector3d::Ones();
  Eigen::Matrix3d sensorNoiseCov = cov / 20;


  ros::Rate r(2);
  while (ros::ok())
  {
    pf.propagate(u, motionNoiseCov, *motionModel);

    pf.correct(z, sensorNoiseCov, *sensorModel);

    pf.resample(particles_number);
    Visualizer::getInstance()->publishParticles(pf.particles);
    r.sleep();
  }


}
