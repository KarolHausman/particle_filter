#ifndef IDENTITY_MOTIONMODEL_H
#define IDENTITY_MOTIONMODEL_H

#include "particle_filter/motion_model.h"

template <class StateType>
class IdentityMotionModel : public MotionModel <StateType>
{
public:
  IdentityMotionModel();
  virtual ~IdentityMotionModel();

  StateType move (const StateType& state, const Eigen::VectorXd&
                                controls, const Eigen::VectorXd& noise)const;
};

#endif // IDENTITY_MOTIONMODEL_H
