#include "particle_filter/gaussian_sensormodel.h"
#include "particle_filter/random.h"
#include "particle_filter/rigid_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rotational_model.h"

template <class StateType> GaussianSensorModel<StateType>::GaussianSensorModel()
{
}

template <class StateType> GaussianSensorModel<StateType>::~GaussianSensorModel()
{
}

template <class StateType> Eigen::VectorXd GaussianSensorModel<StateType>::sense(const StateType &state, const Eigen::VectorXd &noise) const
{

}

template <class StateType> double GaussianSensorModel<StateType>::senseLikelihood(const Eigen::VectorXd &z, const StateType &state, const Eigen::MatrixXd &cov) const
{
  // construct a gaussian with state as mean and noiseCov
  // check where z lies in the gaussian
  // return this value
  return Random::multivariateGaussianProbability(state, cov, z);
}

template <> double GaussianSensorModel<ArticulationModelPtr>::senseLikelihood(const Eigen::VectorXd &z, const ArticulationModelPtr &state, const Eigen::MatrixXd &cov) const
{
  Eigen::VectorXd state_combined = Eigen::VectorXd::Zero(9);

  switch (state->model)
  {
    case (RIGID):
      {
        boost::shared_ptr<RigidModel> rigid = boost::dynamic_pointer_cast< RigidModel > (state);
        state_combined(0) = rigid->pos_x;
        state_combined(1) = rigid->pos_y;
        state_combined(2) = rigid->pos_z;
        state_combined(3) = rigid->roll;
        state_combined(4) = rigid->pitch;
        state_combined(5) = rigid->yaw;
        break;
      }
    case (PRISMATIC):
      {
        boost::shared_ptr<PrismaticModel> prismatic = boost::dynamic_pointer_cast< PrismaticModel > (state);
        state_combined(0) = prismatic->pos_x;
        state_combined(1) = prismatic->pos_y;
        state_combined(2) = prismatic->pos_z;
        state_combined(3) = prismatic->roll;
        state_combined(4) = prismatic->pitch;
        break;
      }
    case (ROTATIONAL):
      {
        boost::shared_ptr<RotationalModel> rotational = boost::dynamic_pointer_cast< RotationalModel > (state);
        state_combined(0) = rotational->rot_center_x;
        state_combined(1) = rotational->rot_center_y;
        state_combined(2) = rotational->rot_center_z;
        state_combined(3) = rotational->roll;
        state_combined(4) = rotational->pitch;
        state_combined(5) = rotational->yaw;
        state_combined(6) = rotational->radius;
        state_combined(7) = rotational->axis_x;
        state_combined(8) = rotational->axis_y;
        break;
      }
  }
  return Random::multivariateGaussianProbability(state_combined, cov, z);
}

template class GaussianSensorModel<Eigen::VectorXd>;
template class GaussianSensorModel<ArticulationModelPtr>;

