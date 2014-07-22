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

//TODO: add free model
template <> double GaussianSensorModel<ArticulationModelPtr>::senseLikelihood(const Eigen::VectorXd &z, const ArticulationModelPtr &state, const Eigen::MatrixXd &cov) const
{
  Eigen::VectorXd z_adjusted, state_adjusted;
  Eigen::MatrixXd cov_adjusted;

  switch (state->model)
  {
    case (RIGID):
      {
        boost::shared_ptr<RigidModel> rigid = boost::dynamic_pointer_cast< RigidModel > (state);
        state_adjusted = Eigen::VectorXd::Zero(6);
        z_adjusted = z.head(6);
        cov_adjusted = cov.block(0, 0, 6, 6);
        state_adjusted(0) = rigid->pos_x;
        state_adjusted(1) = rigid->pos_y;
        state_adjusted(2) = rigid->pos_z;
        state_adjusted(3) = rigid->roll;
        state_adjusted(4) = rigid->pitch;
        state_adjusted(5) = rigid->yaw;
        break;
      }
    case (PRISMATIC):
      {
        boost::shared_ptr<PrismaticModel> prismatic = boost::dynamic_pointer_cast< PrismaticModel > (state);
        state_adjusted = Eigen::VectorXd::Zero(5);
        z_adjusted = z.head(5);
        cov_adjusted = cov.block(0, 0, 5, 5);
        state_adjusted(0) = prismatic->pos_x;
        state_adjusted(1) = prismatic->pos_y;
        state_adjusted(2) = prismatic->pos_z;
        state_adjusted(3) = prismatic->roll;
        state_adjusted(4) = prismatic->pitch;
        break;
      }
    case (ROTATIONAL):
      {
        boost::shared_ptr<RotationalModel> rotational = boost::dynamic_pointer_cast< RotationalModel > (state);
        state_adjusted = Eigen::VectorXd::Zero(9);
        z_adjusted = z.head(9);
        cov_adjusted = cov.block(0, 0, 9, 9);
        state_adjusted(0) = rotational->rot_center_x;
        state_adjusted(1) = rotational->rot_center_y;
        state_adjusted(2) = rotational->rot_center_z;
        state_adjusted(3) = rotational->roll;
        state_adjusted(4) = rotational->pitch;
        state_adjusted(5) = rotational->yaw;
        state_adjusted(6) = rotational->radius;
        state_adjusted(7) = rotational->axis_x;
        state_adjusted(8) = rotational->axis_y;
        break;
      }
  }
//  ROS_ERROR_STREAM ("\n \n \n state_adjusted: \n" << state_adjusted << "\n, cov_adjusted: \n" << cov_adjusted << "\n, z_adjusted: \n" << z_adjusted);
//  ROS_ERROR_STREAM ("\n score: \n" << Random::multivariateGaussianProbability(state_adjusted, cov_adjusted, z_adjusted));
  return Random::multivariateGaussianProbability(state_adjusted, cov_adjusted, z_adjusted);
}

template class GaussianSensorModel<Eigen::VectorXd>;
template class GaussianSensorModel<ArticulationModelPtr>;

