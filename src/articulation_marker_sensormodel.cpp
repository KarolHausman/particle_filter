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

// do not use this one as the likelihoods get too big
template <> double ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>::senseLikelihood(const articulation_model_msgs::TrackMsg &z, const ArticulationModelPtr &state, const Eigen::MatrixXd &cov) const
{
  state->addTrack(z);
  state->evaluateModel();
  //TODO: check if loglikelihood knows for which model it is
  return exp(state->getParam("loglikelihood"));
}

//TODO: add free model
template <> double ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>::senseLogLikelihood(const articulation_model_msgs::TrackMsg &z, const ArticulationModelPtr &state,
                                       const Eigen::MatrixXd &cov) const
{
  std::cerr << "LOG CALLED!!!!!!!!!!" << std::endl;
  state->addTrack(z);
  state->evaluateModel();

  return (state->getParam("loglikelihood"));
}

template class ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;

