#include "particle_filter/articulation_data_sensormodel.h"
#include "articulation_model_msgs/TrackMsg.h"

template <class StateType, class ZType> ArtDataSensorModel<StateType, ZType>::ArtDataSensorModel()
{
  loglikelihood_free_model = 2;//500 -0.11 , 0.2 for stapler
  // 2 works for inital loglikelihood for the cabinet door
}

template <class StateType, class ZType> ArtDataSensorModel<StateType, ZType>::~ArtDataSensorModel()
{
}

template <class StateType, class ZType> ZType ArtDataSensorModel<StateType, ZType>::sense(const StateType &state, const Eigen::VectorXd &noise) const
{

}

template <class StateType, class ZType> double ArtDataSensorModel<StateType, ZType>::senseLikelihood(const ZType &z, const StateType &state, const Eigen::MatrixXd &cov) const
{

}

// do not use this one as the likelihoods get too big
template <> double ArtDataSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>::senseLikelihood(const articulation_model_msgs::TrackMsg &z, const ArticulationModelPtr &state, const Eigen::MatrixXd &cov) const
{
  state->addTrack(z);
  state->evaluateModel();
  return exp(state->getParam("loglikelihood"));
}

template <> double ArtDataSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>::senseLogLikelihood(const articulation_model_msgs::TrackMsg &z, const ArticulationModelPtr &state,
                                       const Eigen::MatrixXd &cov) const
{
  if(state->model == FREE)
  {    
    return loglikelihood_free_model * state->model_msg.track.pose.size();
  }

  state->addTrack(z);
  state->evaluateModel();
  return (state->getParam("loglikelihood"));
}

template class ArtDataSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;

