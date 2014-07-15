
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

  ros::Rate r(2);
  int loop_count = 1;
  while (ros::ok())
  {
    ROS_INFO_STREAM ("loop_count: " << loop_count);
    pf.propagate(u, motionNoiseCov, *motionModel);

    if (loop_count % 10 == 0)
    {
      ROS_INFO ("Correction step started.");
      z += (Eigen::VectorXd(3) << 1, 1, 1).finished();
      ROS_INFO_STREAM ("measurement equals: \n" << z);
      pf.correct(z, sensorNoiseCov, *sensorModel);
      ROS_INFO ("Correction step executed.");
      if (!pf.resample(particles_number))
      {
        ROS_ERROR ("no particles left, quiting");
        return -1;
      }
      ROS_INFO ("Resample step executed.");
    }
    Visualizer::getInstance()->publishParticles(pf.particles);
    r.sleep();
    ++loop_count;

    double fraction = 0.02;
    ROS_INFO_STREAM ("weighted average of " << fraction << " equals: \n" <<
                     pf.getWeightedAvg(fraction));
  }
  delete motionModel;
  delete sensorModel;
}
