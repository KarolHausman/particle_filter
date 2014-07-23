#include "particle_filter/articulation_model.h"
#include <limits>

ArticulationModel::ArticulationModel()
{
//  TODO: not needed yet, setId(-1);
  sigma_position = 0.005;
  sigma_orientation = 360 * M_PI/180.0;

  avg_error_position = 0;
  avg_error_orientation = 0;
  bic = 0;

  last_error_jacobian = 0;
  evaluated = false;
  supress_similar = true;
  outlier_ratio = 0.5;
  sac_iterations = 100;
//  optimizer_iterations = 10;
//  optimizer_iterations = 0;

  prior_outlier_ratio = log(0.01) / (- 0.05); // prior over outlier ratio

  complexity = 0;
  jacobian = Eigen::MatrixXd();
  hessian = Eigen::MatrixXd();


}
ArticulationModel::~ArticulationModel()
{}

void ArticulationModel::setTrack(const articulation_model_msgs::TrackMsg& track)
{
  this->model_msg.track = track;
}


void ArticulationModel::readParamsFromModel()
{
  getParam("sigma_position",sigma_position);
  getParam("sigma_orientation",sigma_orientation);
  getParam("supress_similar",supress_similar);
  getParam("avg_error_position",avg_error_position);
  getParam("avg_error_orientation",avg_error_orientation);
  getParam("bic",bic);
  getParam("last_error_jacobian",last_error_jacobian);
  getParam("evaluated",evaluated);
  //	getParam("complexity",complexity);//read-only
  getParam("jacobian",jacobian);
  getParam("hessian",hessian);
  getParam("loglikelihood",loglikelihood);
  getParam("outlier_ratio",outlier_ratio);
  getParam("sac_iterations",sac_iterations);
  getParam("prior_outlier_ratio",prior_outlier_ratio);
}

void ArticulationModel::writeParamsToModel()
{
  setParam("sigma_position",sigma_position,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("sigma_orientation",sigma_orientation,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("supress_similar",supress_similar,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("avg_error_position",avg_error_position,articulation_model_msgs::ParamMsg::EVAL);
  setParam("avg_error_orientation",avg_error_orientation,articulation_model_msgs::ParamMsg::EVAL);
  setParam("loglikelihood",loglikelihood,articulation_model_msgs::ParamMsg::EVAL);
  setParam("bic",bic,articulation_model_msgs::ParamMsg::EVAL);
  setParam("last_error_jacobian",last_error_jacobian,articulation_model_msgs::ParamMsg::EVAL);
  setParam("evaluated",evaluated,articulation_model_msgs::ParamMsg::EVAL);
  setParam("complexity",complexity,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("jacobian",jacobian,articulation_model_msgs::ParamMsg::EVAL);
  setParam("hessian",hessian,articulation_model_msgs::ParamMsg::EVAL);
  setParam("dofs",getDOFs(),articulation_model_msgs::ParamMsg::EVAL);
  setParam("samples",getSamples(),articulation_model_msgs::ParamMsg::EVAL);
  setParam("outlier_ratio",outlier_ratio,articulation_model_msgs::ParamMsg::EVAL);
  setParam("sac_iterations",sac_iterations,articulation_model_msgs::ParamMsg::PRIOR);
  setParam("prior_outlier_ratio",prior_outlier_ratio,articulation_model_msgs::ParamMsg::PRIOR);
}

double ArticulationModel::getParam(std::string name)
{
  for(size_t i = 0; i < model_msg.params.size(); i++)
  {
    if(model_msg.params[i].name == name)
    {
      return(model_msg.params[i].value);
    }
  }
  std::cerr << "WARNING (in ArticulationModel::getParam): undefined parameter '"<<name<<"'" << std::endl;
  return 0.00;
}

void ArticulationModel::getParam(std::string name,double& value)
{
  if(hasParam(name))
    value = getParam(name);
}

void ArticulationModel::getParam(std::string name,Eigen::VectorXd &vec)
{
  for(int i=0;i<vec.rows();i++)
    vec(i) = getParam( str(boost::format( name+"[%1%]") %i ) );
}

void ArticulationModel::getParam(std::string name,tf::Vector3 &vec)
{
  vec.setX( getParam( name+".x" ) );
  vec.setY( getParam( name+".y" ) );
  vec.setZ( getParam( name+".z" ) );
}

void ArticulationModel::getParam(std::string name,tf::Quaternion &quat)
{
  quat.setX( getParam( name+".x" ) );
  quat.setY( getParam( name+".y" ) );
  quat.setZ( getParam( name+".z" ) );
  quat.setW( getParam( name+".w" ) );
}

void ArticulationModel::getParam(std::string name,tf::Transform &t)
{
  tf::Quaternion q;
  tf::Vector3 v;
  getParam(name+".orientation",q);
  getParam(name+".position",v);
  t = tf::Transform(q,v);
}

void ArticulationModel::getParam(std::string name,Eigen::MatrixXd &mat)
{
  for(int r=0;r<mat.rows();r++)
    for(int c=0;c<mat.cols();c++)
      mat(r,c) = getParam( str(boost::format( name+"[%1%][%2%]") %r % c) );
}

void ArticulationModel::setParam(std::string name,double value,int type)
{
  for(size_t i=0;i<model_msg.params.size();i++)
  {
    if(model_msg.params[i].name == name)
    {
      model_msg.params[i].value = value;
      return;
    }
  }
  articulation_model_msgs::ParamMsg data;
  data.name = name;
  data.value = value;
  data.type = type;
  model_msg.params.push_back(data);
}

void ArticulationModel::setParam(std::string name,const Eigen::VectorXd &vec,int type)
{
  for(int i=0;i<vec.rows();i++)
    setParam( boost::str(boost::format( name+"[%1%]") %i ),vec(i),type );
}

void ArticulationModel::setParam(std::string name,const tf::Vector3 &vec,int type)
{
  setParam( name+".x",vec.x(),type );
  setParam( name+".y",vec.y(),type );
  setParam( name+".z",vec.z(),type );
}

void ArticulationModel::setParam(std::string name,const tf::Quaternion &quat,int type)
{
  setParam( name+".x",quat.x(),type );
  setParam( name+".y",quat.y(),type );
  setParam( name+".z",quat.z(),type );
  setParam( name+".w",quat.w(),type );
}

void ArticulationModel::setParam(std::string name,const tf::Transform &t,int type)
{
  setParam(name+".position",t.getOrigin(),type);
  setParam(name+".orientation",t.getRotation(),type);
}

void ArticulationModel::setParam(std::string name,const Eigen::MatrixXd &mat,int type)
{
  for(int r=0;r<mat.rows();r++)
    for(int c=0;c<mat.cols();c++)
      setParam( str(boost::format( name+"[%1%][%2%]") %r%c ),mat(r,c),type );
}

bool ArticulationModel::hasParam(std::string name)
{
  for(size_t i=0;i<model_msg.params.size();i++)
  {
    if(model_msg.params[i].name == name)
      return(true);
  }
  return false;
}



bool ArticulationModel::sampleConsensus()
{
  // sample consensus
  writeParamsToModel();
  std::vector<articulation_model_msgs::ParamMsg> bestParams = model_msg.params;
  double bestLikelihood = -std::numeric_limits<double>::max();
  bool goodGuess = false;
  for(size_t it=0; it < sac_iterations; it++)
  {
    if(!guessParameters()) continue;
    goodGuess = true;
    double likelihood = getLogLikelihood(true);
//    cout <<"RANSAC iteration="<<it<<", likelihood="<<likelihood<<endl;
    if(bestLikelihood < likelihood)
    {
      writeParamsToModel();
      bestParams = model_msg.params;
      bestLikelihood = likelihood;
    }
  }
//	cout <<"RANSAC best likelihood="<<bestLikelihood<<endl;
  model_msg.params = bestParams;
  readParamsFromModel();

  return goodGuess;
}

bool ArticulationModel::guessParameters()
{
  // sample a minimum number of samples to compute parameters
  return false;
}

size_t ArticulationModel::getSamples()
{
  return model_msg.track.pose.size();
}

V_Configuration ArticulationModel::predictConfiguration(geometry_msgs::Pose pose)
{
  V_Configuration q;
  if(getDOFs()>0) q.resize(getDOFs());
  return q;
}

geometry_msgs::Pose ArticulationModel::predictPose(V_Configuration q)
{
  geometry_msgs::Pose p;
  p.orientation.w = 1;
  return p;
}

double ArticulationModel::getInlierLogLikelihood( size_t index )
{
  geometry_msgs::Pose &pose_obs = model_msg.track.pose[index];
  V_Configuration q_estimated = predictConfiguration(pose_obs);
  for(size_t i = 0; i < (size_t)q_estimated.rows(); i++)
  {
    model_msg.track.channels[channelConfiguration[i]].values[index] =  q_estimated[i];
  }

  geometry_msgs::Pose pose_estimated = predictPose(q_estimated);
  model_msg.track.pose_projected[index] = pose_estimated;

  tf::Transform pose_obs_tf, pose_estimated_tf;
  tf::poseMsgToTF(pose_obs, pose_obs_tf);
  tf::poseMsgToTF(pose_estimated, pose_estimated_tf);

  tf::Transform diff = pose_obs_tf.inverseTimes(pose_estimated_tf);//poseToTransform(pose_obs).inverseTimes( poseToTransform(pose_estimated) );
  double err_position = diff.getOrigin().length();
  double err_orientation = fabs(diff.getRotation().getAngle());

  double loglikelihood =
                  - log(2*M_PI * sigma_position*sigma_orientation)
                  - 0.5*(
                                  ( (err_position*err_position) / (sigma_position*sigma_position) ) +
                                  ( (err_orientation*err_orientation) / (sigma_orientation*sigma_orientation)) );	// 2-dim multivariate normal distribution
  return loglikelihood;
}

double ArticulationModel::getOutlierLogLikelihood()
{
  // outside the 95% ellipsoid
  double chi2inv = 1.96;
  //	chi2inv = 6.63;	//99% ellipsoid

  double err_position = chi2inv * sigma_position;
  double err_orientation = chi2inv * sigma_orientation;

  double loglikelihood =
                  - log(2*M_PI * sigma_position*sigma_orientation)
                  - 0.5*(
                                  ( (err_position*err_position) / (sigma_position*sigma_position) ) +
                                  ( (err_orientation*err_orientation) / (sigma_orientation*sigma_orientation)) );	// 2-dim multivariate normal distribution

  return( loglikelihood );
}

double ArticulationModel::getLogLikelihoodForPoseIndex(size_t index) {
//  double gamma_mixing = 0.1;
//  return (1-gamma_mixing)*getInlierLogLikelihood(pose_obs) + gamma_mixing * getOutlierLogLikelihood();
    double inlierLikelihood = getInlierLogLikelihood(index);
    double outlierLikelihood = getOutlierLogLikelihood();

//  mle-sac
    model_msg.track.channels[channelInlierLogLikelihood].values[index] = inlierLikelihood;
    double pi = (1-outlier_ratio) * exp( inlierLikelihood );
    double po = outlier_ratio * exp( outlierLikelihood );
    model_msg.track.channels[channelOutlier].values[index] =  po / (pi+po);
    double p = (log(exp(inlierLikelihood) + exp(outlierLikelihood)));
    model_msg.track.channels[channelLogLikelihood].values[index] =  p;
    return p;

//  // ransac
//  return ((1-gamma_mixing)*inlierLikelihood + (gamma_mixing)*outlierLikelihood);

//  // m-sac
//  return (max(inlierLikelihood,outlierLikelihood));
}


double ArticulationModel::getLogLikelihood(bool estimate_outlier_ratio)
{
  model_msg.track.pose_projected.resize(model_msg.track.pose.size());

  model_msg.track.channels[channelInlierLogLikelihood].values.resize(model_msg.track.pose.size());
  model_msg.track.channels[channelOutlier].values.resize(model_msg.track.pose.size());
  model_msg.track.channels[channelLogLikelihood].values.resize(model_msg.track.pose.size());
  for(size_t i=0; i<(size_t)getDOFs(); i++)
  {
    model_msg.track.channels[channelConfiguration[i]].values.resize(model_msg.track.pose.size());
  }

  double sum_likelihood = 0;
  size_t n = getSamples();
  if(estimate_outlier_ratio)
  {
    outlier_ratio =0.5;
    for(size_t i=0;i<n;i++)
    {
      model_msg.track.channels[channelOutlier].values[i] = 0.5;	// initial estimate of gamma
    }

    int iter = 0;
    double diff = 0;
    do
    {
      sum_likelihood = 0;
      for(size_t i=0; i<n; i++)
      {
        sum_likelihood += getLogLikelihoodForPoseIndex(i);
      }

      double outlier_ratio_new = 0;
      for(size_t i=0;i<n;i++) {
              outlier_ratio_new += model_msg.track.channels[channelOutlier].values[i]/n;
      }
      diff = fabs(outlier_ratio - outlier_ratio_new);
      iter++;
      outlier_ratio = outlier_ratio_new;
  //  cout <<"EM iter="<<iter<<" outlier_ratio="<<outlier_ratio<<endl;
    } while( diff > 0.01 && iter < 10 );
  } else
  {
    for(size_t i = 0; i < n; i++)
    {
      sum_likelihood += getLogLikelihoodForPoseIndex(i);
    }
  }
  sum_likelihood += - prior_outlier_ratio * outlier_ratio * n;
  return sum_likelihood;
}

void ArticulationModel::prepareChannels()
{
  channelOutlier = openChannel(model_msg.track, "outlier");
  channelLogLikelihood = openChannel(model_msg.track, "loglikelihood");
  channelInlierLogLikelihood = openChannel(model_msg.track, "loglikelihood_if_inlier");
  channelConfiguration.resize(getDOFs(),0);
  for(size_t i = 0; i < getDOFs(); i++)
  {
    channelConfiguration[i] = openChannel(model_msg.track, str(boost::format("q%d")%i));
  }

}

size_t ArticulationModel::getDOFs()
{
  return 0;
}

int ArticulationModel::openChannel(articulation_model_msgs::TrackMsg &track, std::string name, bool autocreate)
{
  // find channel
  size_t i = 0;
  for(; i < track.channels.size(); i++)
  {
    if(track.channels[i].name == name)
      break;
  }
  // found?
  if( i == track.channels.size() )
  {
    if(!autocreate)
      return -1;
    // create, if not found
    sensor_msgs::ChannelFloat32 ch;
    ch.name = name;
    track.channels.push_back(ch);
  }
  // ensure that channel has right number of elements
  track.channels[i].values.resize( track.pose.size() );
  // return channel number
  return i;
}



