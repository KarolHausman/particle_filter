#include "particle_filter/identity_motionmodel.h"

IdentityMotionModel::IdentityMotionModel()
{
}

IdentityMotionModel::~IdentityMotionModel()
{
}

Eigen::VectorXd IdentityMotionModel::move(const Eigen::VectorXd &state, const Eigen::VectorXd &controls,
                          const Eigen::VectorXd &noise) const
{
  return state; //+ noise;
}
