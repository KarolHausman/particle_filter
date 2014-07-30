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
  state->addTrack(z);
  switch (state->model)
  {
    case (RIGID):
      {
        boost::shared_ptr<RigidModel> rigid = boost::dynamic_pointer_cast< RigidModel > (state);

        break;
      }
    case (PRISMATIC):
      {
        boost::shared_ptr<PrismaticModel> prismatic = boost::dynamic_pointer_cast< PrismaticModel > (state);

        break;
      }
    case (ROTATIONAL):
      {
        boost::shared_ptr<RotationalModel> rotational = boost::dynamic_pointer_cast< RotationalModel > (state);

        break;
      }
  }
}

template class ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;

