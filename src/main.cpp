
#include <iostream>
#include "particle_filter/particle.h"
#include "particle_filter/paricle_filter.h"
#include "particle_filter/identity_motionmodel.h"
#include "particle_filter/gaussian_sensormodel.h"
#include "particle_filter/visualizer.h"
#include "particle_filter/random.h"

#include "particle_filter/articulation_io_utils.h"

int main(int argc, char **argv)
{
  const int particles_number = 20;
  ros::init(argc, argv, "particle_filter");

  Visualizer::getInstance()->init();

//  Eigen::Vector3d mean = Eigen::Vector3d::Ones();
//  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();



  Eigen::VectorXd rigid_mean = Eigen::VectorXd::Ones(6);
  Eigen::MatrixXd rigid_cov = Eigen::MatrixXd::Identity(6, 6);
  rigid_cov = rigid_cov / 10;

  Eigen::VectorXd rotational_mean = Eigen::VectorXd::Ones(10);
  Eigen::MatrixXd rotational_cov = Eigen::MatrixXd::Identity(10, 10);
  rotational_cov = rotational_cov / 10;

  Eigen::VectorXd prismatic_mean = Eigen::VectorXd::Ones(9);
  Eigen::MatrixXd prismatic_cov = Eigen::MatrixXd::Identity(9, 9);
  prismatic_cov = prismatic_cov / 10;


  ParticleFilter<ArticulationModelPtr> pf (particles_number, rigid_mean, rigid_cov,
                                           rotational_mean, rotational_cov,
                                           prismatic_mean, prismatic_cov);

//  MotionModel<Eigen::VectorXd>* motionModel = new IdentityMotionModel<Eigen::VectorXd>;
//  SensorModel<Eigen::VectorXd>* sensorModel = new GaussianSensorModel<Eigen::VectorXd>;

  MotionModel<ArticulationModelPtr>* motionModel = new IdentityMotionModel<ArticulationModelPtr>;
  SensorModel<ArticulationModelPtr, Eigen::VectorXd>* sensorModel = new GaussianSensorModel<ArticulationModelPtr, Eigen::VectorXd>;



  Eigen::VectorXd u = Eigen::VectorXd::Ones(10);
  Eigen::MatrixXd motionNoiseCov = rotational_cov / 10;

  Eigen::VectorXd z = Eigen::VectorXd::Ones(10);
  Eigen::MatrixXd sensorNoiseCov = rotational_cov / 2;

  ros::Rate r(2);
  uint loop_count = 1;

  while (ros::ok())
  {
    ROS_INFO_STREAM ("loop_count: " << loop_count);
    pf.propagate(pf.particles, u, motionNoiseCov, *motionModel);

    if (loop_count % 10 == 0)
    {

      ROS_INFO ("Correction step started.");
      z = (Eigen::VectorXd(10) << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1).finished();
      ROS_INFO_STREAM ("measurement equals: \n" << z);

      pf.correct<Eigen::VectorXd>(pf.particles, z, sensorNoiseCov, *sensorModel);
      ROS_INFO ("Correction step executed.");
      if (!pf.resample(particles_number, pf.particles))
      {
        ROS_ERROR ("no particles left, quiting");
        return -1;
      }
      ROS_INFO ("Resample step executed.");
      pf.printParticles(pf.particles);
    }
//    Visualizer::getInstance()->publishParticles(pf.particles);
    r.sleep();
    ++loop_count;

    double fraction = 0.02;
//    ROS_INFO_STREAM ("weighted average of " << fraction << " equals: \n" <<
//                     pf.getWeightedAvg(fraction));
  }
  delete motionModel;
  delete sensorModel;
}
