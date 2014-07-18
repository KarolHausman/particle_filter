#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <vector>
#include <Eigen/Core>
#include <particle_filter/articulation_model.h>

template <class StateType>
class MotionModel
{
public:
  MotionModel();
  virtual ~MotionModel();

  virtual StateType move (const StateType& state, const Eigen::VectorXd&
                                controls, const Eigen::VectorXd& noise) const = 0;


};



#endif //MOTION_MODEL_H
