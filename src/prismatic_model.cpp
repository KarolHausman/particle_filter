#include "particle_filter/prismatic_model.h"

PrismaticModel::PrismaticModel()
{
  model = PRISMATIC;
  complexity = 6+2; //rigid + orientation - yaw
  prismatic_dir = tf::Vector3(0,0,0);
}

PrismaticModel::~PrismaticModel()
{
}

// -- params
void PrismaticModel::readParamsFromModel()
{
  RigidModel::readParamsFromModel();
  getParam("prismatic_dir",prismatic_dir);
}

void PrismaticModel::writeParamsToModel()
{
  RigidModel::writeParamsToModel();
  setParam("prismatic_dir",prismatic_dir,articulation_model_msgs::ParamMsg::PARAM);
}

V_Configuration PrismaticModel::predictConfiguration(geometry_msgs::Pose pose)
{
  tf::Vector3 vector_position = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
  tf::Vector3 diff = (vector_position - rigid_position);

  V_Configuration q( 1 );
  q(0) = diff.dot(prismatic_dir);

  return q;
}

geometry_msgs::Pose PrismaticModel::predictPose(V_Configuration q) {
//	cout << "predictPose q(0)=" <<q(0) << endl;
//	PRINT(rigid_orientation);
//	PRINT(rigid_position);
//	PRINT(prismatic_dir);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(tf::Transform( rigid_orientation, rigid_position + q(0) * prismatic_dir ), pose);
  return pose;
}

bool PrismaticModel::guessParameters()
{
  if(model_msg.track.pose.size() < 2)
    return false;

  size_t i,j;
  do
  {
    i = rand() % getSamples();
    j = rand() % getSamples();
  } while (i==j);

  tf::Transform pose1, pose2;
  tf::poseMsgToTF(model_msg.track.pose[i], pose1);
  tf::poseMsgToTF(model_msg.track.pose[j], pose2);

  rigid_position = pose1.getOrigin();
  rigid_orientation = pose1.getRotation();
  prismatic_dir =  pose2.getOrigin() - pose1.getOrigin();
  prismatic_dir.normalize();

  if(!check_values(rigid_position)) return false;
  if(!check_values(rigid_orientation)) return false;
  if(!check_values(prismatic_dir)) return false;

  return true;
}

void PrismaticModel::updateParameters(std::vector<double> delta)
{
  RigidModel::updateParameters(delta);

  tf::Quaternion q;
  q.setRPY(delta[6],delta[7],0.00);
  prismatic_dir = tf::Matrix3x3(q) * prismatic_dir;
}

bool PrismaticModel::normalizeParameters()
{
  if(model_msg.track.pose.size()>2)
  {
    rigid_position = rigid_position + predictConfiguration(model_msg.track.pose.front())[0] * prismatic_dir;
    if(predictConfiguration(model_msg.track.pose.back())[0] < 0)
      prismatic_dir *= -1;
  }
  return true;
}

M_CartesianJacobian PrismaticModel::predictHessian(V_Configuration q,double delta)
{
  M_CartesianJacobian H;
  H.setZero(3*getDOFs(),getDOFs());
  return H;
}

