#include "particle_filter/articulation_marker_sensormodel.h"

#include "particle_filter/rigid_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rotational_model.h"

#include "articulation_model_msgs/TrackMsg.h"

template <class StateType, class ZType> ArtMarkerSensorModel<StateType, ZType>::ArtMarkerSensorModel()
{
}

template <class StateType, class ZType> ArtMarkerSensorModel<StateType, ZType>::~ArtMarkerSensorModel()
{
}

template <class StateType, class ZType> ZType ArtMarkerSensorModel<StateType, ZType>::sense(const StateType &state, const Eigen::VectorXd &noise) const
{

}

template <class StateType, class ZType> double ArtMarkerSensorModel<StateType, ZType>::senseLikelihood(const ZType &z, const StateType &state, const Eigen::MatrixXd &cov) const
{

}

//TODO: add free model
template <> double ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>::senseLikelihood(const articulation_model_msgs::TrackMsg &z, const ArticulationModelPtr &state, const Eigen::MatrixXd &cov) const
{
  Eigen::VectorXd z_adjusted, state_adjusted;
  Eigen::MatrixXd cov_adjusted;

  switch (state->model)
  {
    case (RIGID):
      {
        boost::shared_ptr<RigidModel> rigid = boost::dynamic_pointer_cast< RigidModel > (state);
        state_adjusted = Eigen::VectorXd::Zero(6);
//        z_adjusted = z.head(6);
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
        state_adjusted = Eigen::VectorXd::Zero(9);
//        z_adjusted = z.head(9);
        cov_adjusted = cov.block(0, 0, 9, 9);
        state_adjusted(0) = prismatic->pos_x;
        state_adjusted(1) = prismatic->pos_y;
        state_adjusted(2) = prismatic->pos_z;
        state_adjusted(3) = prismatic->roll;
        state_adjusted(4) = prismatic->pitch;
        state_adjusted(5) = prismatic->yaw;
        state_adjusted(6) = prismatic->axis_x;
        state_adjusted(7) = prismatic->axis_y;
        state_adjusted(8) = prismatic->axis_z;

        break;
      }
    case (ROTATIONAL):
      {
        boost::shared_ptr<RotationalModel> rotational = boost::dynamic_pointer_cast< RotationalModel > (state);
        state_adjusted = Eigen::VectorXd::Zero(10);
//        z_adjusted = z.head(10);
        cov_adjusted = cov.block(0, 0, 10, 10);
        state_adjusted(0) = rotational->rot_center_x;
        state_adjusted(1) = rotational->rot_center_y;
        state_adjusted(2) = rotational->rot_center_z;
        state_adjusted(3) = rotational->roll;
        state_adjusted(4) = rotational->pitch;
        state_adjusted(5) = rotational->yaw;
        state_adjusted(6) = rotational->radius;
        state_adjusted(7) = rotational->axis_roll;
        state_adjusted(8) = rotational->axis_pitch;
        state_adjusted(9) = rotational->axis_yaw;
        break;
      }
  }
}

template class ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;

