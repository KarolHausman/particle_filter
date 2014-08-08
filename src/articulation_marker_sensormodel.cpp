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
  return exp(state->getParam("loglikelihood"));
}

//TODO: add free model
template <> double ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>::senseLogLikelihood(const articulation_model_msgs::TrackMsg &z, const ArticulationModelPtr &state,
                                       const Eigen::MatrixXd &cov) const
{  
  state->setTrack(z);
  //  state->addTrack(z);

  state->evaluateModel();
  double zkgm = state->getParam("loglikelihood"); //z_k given model

//  state->addTrack(z);
//  state->evaluateModel();
//  double zkzlgm = state->getParam("loglikelihood"); //z_k, z_lambda given model
//  double zlgm = zkzlgm - zkgm;
  if (state->getParam("added"))
    ROS_ERROR_STREAM ("model: " << state->getModel().name << " Z_K = " << zkgm /*<< ", Z_FULL = " << zkzlgm << ", Z_Lambda = " << zlgm*/);
  else
    ROS_INFO_STREAM ("model: " << state->getModel().name << " Z_K = " << zkgm  /*<< ", Z_FULL = " << zkzlgm << ", Z_Lambda = " << zlgm*/);

//  return (state->getParam("loglikelihood"));
  return zkgm;
}

template class ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;

