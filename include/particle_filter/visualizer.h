#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <Eigen/Core>
#include <ros/ros.h>


class Visualizer
{
public:
  static Visualizer* getInstance ();

  void init();

  void publishPoints (const std::vector <Eigen::Vector3d>& points);

private:
  Visualizer (){}
  Visualizer (Visualizer const&){}
  Visualizer& operator = (Visualizer const&){}

  static Visualizer* instance_;
  ros::Publisher marker_pub_;
  ros::NodeHandle nh_;
};

#endif //VISUALIZER_H
