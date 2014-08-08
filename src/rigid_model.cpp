#include "particle_filter/rigid_model.h"

RigidModel::RigidModel()
{
  model = RIGID;
  rigid_position = tf::Vector3(0,0,0);
  rigid_orientation = tf::Quaternion(0,0,0,1);
  pos_x = 0;
  pos_y = 0;
  pos_z = 0;
  roll = 0;
  pitch = 0;
  yaw = 0;


  updateStateParametersToModel();
//  rigid_width = 1;
//  rigid_height = 1;
  complexity = 6;
}

RigidModel::~RigidModel()
{
}

void RigidModel::updateStateParametersToModel()
{
  pos_x = rigid_position.getX();
  pos_y = rigid_position.getY();
  pos_z = rigid_position.getZ();
  tf::Matrix3x3 m(rigid_orientation);
  m.getEulerYPR(yaw, pitch, roll);
}

void RigidModel::updateModelToStateParameters()
{
  rigid_position.setX(pos_x);
  rigid_position.setY(pos_y);
  rigid_position.setZ(pos_z);
  rigid_orientation.setRPY(roll, pitch, yaw);
}

void RigidModel::readParamsFromModel()
{
  ArticulationModel::readParamsFromModel();
  getParam("rigid_position",rigid_position);
  getParam("rigid_orientation",rigid_orientation);
//  getParam("rigid_width",rigid_width);
//  getParam("rigid_height",rigid_height);

 updateStateParametersToModel();
}

void RigidModel::writeParamsToModel()
{
  ArticulationModel::writeParamsToModel();
  updateModelToStateParameters();

  setParam("rigid_position",rigid_position,articulation_model_msgs::ParamMsg::PARAM);
  setParam("rigid_orientation",rigid_orientation,articulation_model_msgs::ParamMsg::PARAM);
//  setParam("rigid_width",rigid_width,articulation_model_msgs::ParamMsg::PARAM);
//  setParam("rigid_height",rigid_height,articulation_model_msgs::ParamMsg::PARAM);
}

geometry_msgs::Pose RigidModel::predictPose(V_Configuration q)
{
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(tf::Transform(rigid_orientation,rigid_position), pose);
  return pose;
}


bool RigidModel::guessParameters()
{
  if(model_msg.track.pose.size() == 0)
          return false;

  size_t i = rand() % getSamples();

  tf::Transform pose;
  tf::poseMsgToTF(model_msg.track.pose[i], pose);
  rigid_position = pose.getOrigin();
  rigid_orientation = pose.getRotation();
  updateStateParametersToModel();
  return true;
}

void RigidModel::updateParameters(std::vector<double> delta)
{
  rigid_position = rigid_position + tf::Vector3(delta[0],delta[1],delta[2]);
  tf::Quaternion q;
  q.setEuler(delta[3],delta[4],delta[5]);
  rigid_orientation = rigid_orientation * q;
}
