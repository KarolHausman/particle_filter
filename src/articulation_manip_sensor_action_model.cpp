#include "particle_filter/articulation_manip_sensor_action_model.h"
#include "particle_filter/action_prismatic.h"
#include "particle_filter/action_rotational.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rotational_model.h"
#include "particle_filter/random.h"


template <class StateType, class ZType, class AType> ArtManipSensorActionModel<StateType, ZType, AType>::ArtManipSensorActionModel()
{
}

template <class StateType, class ZType, class AType> ArtManipSensorActionModel<StateType, ZType, AType>::~ArtManipSensorActionModel()
{
}


template <class StateType, class ZType, class AType> ZType ArtManipSensorActionModel<StateType, ZType, AType>::sense(const StateType &state, const Eigen::VectorXd &noise) const
{
}

template <class StateType, class ZType, class AType> double ArtManipSensorActionModel<StateType, ZType, AType>::senseLikelihood(const ZType &z, const AType& a,
                                                                                                                                const StateType &state,
                                                                                                                                const Eigen::MatrixXd &cov) const
{
}

template <class StateType, class ZType, class AType> double ArtManipSensorActionModel<StateType, ZType, AType>::senseLogLikelihood(const ZType &z, const AType& a,
                                                                                                                                const StateType &state,
                                                                                                                                const Eigen::MatrixXd &cov) const
{
}


template <> double ArtManipSensorActionModel<ArticulationModelPtr, int, ActionPtr>::senseLikelihood(const int &z,
                                                                                                   const ActionPtr& a,
                                                                                                   const ArticulationModelPtr &state,
                                                                                                   const Eigen::MatrixXd &cov) const
{
  return 0.0;
}
// z = 1 means it doesnt stop
//TODO: check angle comparisons
template <> double ArtManipSensorActionModel<ArticulationModelPtr, int, ActionPtr>::senseLogLikelihood(const int &z,
                                                                                                      const ActionPtr& a,
                                                                                                      const ArticulationModelPtr &state,
                                                                                                      const Eigen::MatrixXd &cov) const
{
  double loglikelihood = 0;

  switch (state->model)
  {
    case (RIGID):
      {
        if (z == 1)
        {
          loglikelihood = -std::numeric_limits<double>::lowest();
        }
        else
        {
          //TODO: how to increase likelihood here?
        }
      }

    case (PRISMATIC):
      {
        // calculate the relative angle between prismatic axis
        if (a->action_type == PRISMATIC_ACTION)
        {
          boost::shared_ptr<ActionPrismatic> action_prismatic = boost::dynamic_pointer_cast< ActionPrismatic > (a);
          boost::shared_ptr<PrismaticModel> prismatic_model = boost::dynamic_pointer_cast< PrismaticModel > (state);

          Eigen::Vector3d a(action_prismatic->axis_x, action_prismatic->axis_y, action_prismatic->axis_z);
          Eigen::Vector3d prism_dir(prismatic_model->axis_x, prismatic_model->axis_y, prismatic_model->axis_z);
          a.normalize();
          prism_dir.normalize();
          double angle = acos(a.dot(prism_dir));
          double density = Random::gaussianDensity(0, sqrt(cov(0,0)), angle);
          //TODO: change density into prob or find a better way than 1 - prob
          if (z == 1)
          {
            loglikelihood = log(density);
          }
          else
          {
            loglikelihood = log(1-density);
          }
        }
        else if(a->action_type == ROTATIONAL_ACTION)
        {
          //TODO: what then?
        }

      }
    case (ROTATIONAL):
      {
        //compare rot axises and radiuses in 2-dimensional gaussian
        if (a->action_type == ROTATIONAL_ACTION)
        {
          boost::shared_ptr<ActionRotational> action_rotational = boost::dynamic_pointer_cast< ActionRotational > (a);
          boost::shared_ptr<RotationalModel> rotational_model = boost::dynamic_pointer_cast< RotationalModel > (state);

          //get z axis from both quaternions and compare
          tf::Matrix3x3 rotation_matrix_model(rotational_model->rot_axis);
          tf::Vector3 z_model = rotation_matrix_model.getColumn(2);
          tf::Matrix3x3 rotation_matrix_action(action_rotational->rot_axis);
          tf::Vector3 z_action = rotation_matrix_action.getColumn(2);
          double angle = z_action.dot(z_model);
          //compare radiuses
          double radius_diff = fabs(rotational_model->radius - action_rotational->radius);
          //construct a gaussian
          Eigen::Vector2d diff(angle, radius_diff);
          Eigen::Vector2d mean(0.0, 0.0);
          double density = Random::multivariateGaussianDensity(mean, cov.block<2,2>(0,0), diff);
          if (z == 1)
          {
            loglikelihood = log(density);
          }
          else
          {
            loglikelihood = log(1-density);
          }

        }
        else if(a->action_type == PRISMATIC_ACTION)
        {
          //TODO: what then?
        }

      }
  }



  return loglikelihood;
}



template class ArtManipSensorActionModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg, ActionPtr>;
template class ArtManipSensorActionModel<ArticulationModelPtr, int, ActionPtr>;

