#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <vector>
#include <Eigen/Core>

class MotionModel
{
  MotionModel();
  virtual ~MotionModel();

  virtual Eigen::VectorXd move (const Eigen::VectorXd& state, const Eigen::VectorXd&
                                controls, const Eigen::VectorXd& noise) const = 0;


};



#endif //MOTION_MODEL_H
