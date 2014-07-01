#ifndef IDENTITY_MOTIONMODEL_H
#define IDENTITY_MOTIONMODEL_H

#include "particle_filter/motion_model.h"

class IdentityMotionModel : public MotionModel
{
public:
  IdentityMotionModel();
  virtual ~IdentityMotionModel();


  Eigen::VectorXd move (const Eigen::VectorXd& state, const Eigen::VectorXd&
                                controls, const Eigen::VectorXd& noise)const;
};

#endif // IDENTITY_MOTIONMODEL_H
