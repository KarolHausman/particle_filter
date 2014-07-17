#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <Eigen/Core>
#include <ros/ros.h>
#include "particle_filter/particle.h"

class Visualizer
{
public:
  static Visualizer* getInstance ();

  void init();

  void publishPoints (const std::vector <Eigen::VectorXd>& points, const double& scale);
  void publishParticles (const std::vector <Particle <Eigen::VectorXd> > &particles);


private:
  Visualizer (){}
  ~Visualizer (){delete instance_;}
  Visualizer (Visualizer const&){}
  Visualizer& operator = (Visualizer const&){}

  static Visualizer* instance_;
  ros::Publisher marker_pub_;
  ros::NodeHandle nh_;
};

#endif //VISUALIZER_H
