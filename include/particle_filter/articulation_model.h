#ifndef ARTICULATION_MODEL_H
#define ARTICULATION_MODEL_H

#include <iostream>
#include "boost/shared_ptr.hpp"
#include "articulation_model_msgs/ModelMsg.h"
#include "articulation_model_msgs/TrackMsg.h"
#include "articulation_model_msgs/ParamMsg.h"
#include "Eigen/Core"
#include <tf/tf.h>


class ArticulationModel;
typedef boost::shared_ptr<ArticulationModel> ArticulationModelPtr;
typedef Eigen::VectorXd V_Configuration;
enum Model { FREE, PRISMATIC, RIGID, ROTATIONAL, MODELS_NUMBER };

class ArticulationModel
{
public:
  ArticulationModel();
  virtual ~ArticulationModel();
  virtual ArticulationModelPtr getCopy()
  {
     return ArticulationModelPtr(new ArticulationModel(*this));
  }

  virtual void setTrack(const articulation_model_msgs::TrackMsg& track);
  virtual bool sampleConsensus();
  virtual bool guessParameters();


  virtual double getInlierLogLikelihood( size_t index );
  virtual double getLogLikelihoodForPoseIndex( size_t index );
  virtual double getLogLikelihood( bool estimate_outlier_ratio );
  virtual V_Configuration predictConfiguration(geometry_msgs::Pose pose);
  virtual geometry_msgs::Pose predictPose(V_Configuration q);
  virtual double getOutlierLogLikelihood();


  virtual void prepareChannels();
  virtual size_t getDOFs();
  virtual size_t getSamples();
  int openChannel(articulation_model_msgs::TrackMsg &track,std::string name,bool autocreate=true);
  virtual void readParamsFromModel();
  virtual void writeParamsToModel();


  bool hasParam(std::string name);
  double getParam(std::string name);
  void getParam(std::string name,double& data);
  void getParam(std::string name,tf::Vector3 &vec);
  void getParam(std::string name,tf::Quaternion &quat);
  void getParam(std::string name,tf::Transform &t);
  void getParam(std::string name,Eigen::VectorXd &vec);
  void getParam(std::string name,Eigen::MatrixXd &mat);
  void setParam(std::string name,double value,int type);
  void setParam(std::string name,const tf::Vector3 &vec,int type);
  void setParam(std::string name,const tf::Quaternion &quat,int type);
  void setParam(std::string name,const tf::Transform &t,int type);
  void setParam(std::string name,const Eigen::VectorXd &vec,int type);
  void setParam(std::string name,const Eigen::MatrixXd &mat,int type);



  articulation_model_msgs::ModelMsg model_msg;
  Model model;
  int channelOutlier;
  int channelLogLikelihood;
  int channelInlierLogLikelihood;
  std::vector<int> channelConfiguration;

  // global params
  double sigma_position;
  double sigma_orientation;
  double supress_similar;
  double outlier_ratio;
  double sac_iterations;
//  TODO: not needed yet: double optimizer_iterations;

  // cached variables
  double complexity;
  double avg_error_position;
  double avg_error_orientation;
  double loglikelihood;
  double bic;
  double prior_outlier_ratio;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd hessian;

  double last_error_jacobian;
  double evaluated;

protected:
};



#endif // ARTICULATION_MODEL_H
