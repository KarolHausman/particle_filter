#include "particle_filter/rotational_model.h"

RotationalModel::RotationalModel()
{
  model = ROTATIONAL;
  rot_center = tf::Vector3 (0,0,0);
  rot_axis = tf::Quaternion (0,0,0,1);
  rot_radius = 1;
  rot_orientation = tf::Quaternion (0,0,0,1);
  complexity = 3+2+1+3;	// rotation center + rotation axis + radius + orientation
  rot_mode = 0;// use position (=0) or orientation (=1) for configuration estimation
  updateStateParametersToModel();
}

RotationalModel::~RotationalModel()
{
}

void RotationalModel::updateStateParametersToModel()
{
  rot_center_x = rot_center.getX();
  rot_center_y = rot_center.getY();
  rot_center_z = rot_center.getZ();
  tf::Matrix3x3 m(rot_orientation);
  m.getEulerYPR(yaw, pitch, roll);
  radius = rot_radius;
  tf::Matrix3x3 ma(rot_axis);
  ma.getEulerYPR(axis_yaw, axis_pitch, axis_roll);
}

void RotationalModel::updateModelToStateParameters()
{
  rot_center.setX(rot_center_x);
  rot_center.setY(rot_center_y);
  rot_center.setZ(rot_center_z);
  rot_orientation.setRPY(roll, pitch, yaw);
  rot_radius = radius;
  rot_axis.setRPY(axis_roll, axis_pitch, axis_yaw);
}

void RotationalModel::readParamsFromModel()
{
  ArticulationModel::readParamsFromModel();
  getParam("rot_center",rot_center);
  getParam("rot_axis",rot_axis);
  getParam("rot_radius",rot_radius);
  getParam("rot_orientation",rot_orientation);
  getParam("rot_mode",rot_mode);

  updateStateParametersToModel();
}

void RotationalModel::writeParamsToModel()
{
  ArticulationModel::writeParamsToModel();
  updateModelToStateParameters();

  setParam("rot_center",rot_center,articulation_model_msgs::ParamMsg::PARAM);
  setParam("rot_axis",rot_axis,articulation_model_msgs::ParamMsg::PARAM);
  setParam("rot_radius",rot_radius,articulation_model_msgs::ParamMsg::PARAM);
  setParam("rot_orientation",rot_orientation,articulation_model_msgs::ParamMsg::PARAM);
  setParam("rot_mode",rot_mode,articulation_model_msgs::ParamMsg::PARAM);
}

V_Configuration RotationalModel::predictConfiguration(geometry_msgs::Pose pose)
{
  V_Configuration q(1);

  if(rot_mode==1)
  {
    tf::Transform tf_pose;
    tf::poseMsgToTF(pose, tf_pose);
    // estimate configuration from pose orientation
    tf::Matrix3x3 m_pose( tf_pose.getBasis() );
    tf::Matrix3x3 m_axis(rot_axis);
    tf::Matrix3x3 m_orient(rot_orientation);

    tf::Matrix3x3 rot_z = m_axis.inverse() * m_pose * m_orient.inverse();
    tf::Quaternion rot_z_quat;
    rot_z.getRotation(rot_z_quat);

    q(0) = rot_z_quat.getAngle();
  } else
  {
    tf::Transform center(rot_axis,rot_center);
    tf::Transform tf_pose;
    tf::poseMsgToTF(pose, tf_pose);
    tf::Transform rel = center.inverseTimes( tf_pose );
    q(0) = -atan2(rel.getOrigin().y(), rel.getOrigin().x());
  }
  return q;
}

geometry_msgs::Pose RotationalModel::predictPose(V_Configuration q)
{
  geometry_msgs::Pose pose;

  tf::Transform center(rot_axis,rot_center);
  tf::Transform rotZ(tf::Quaternion(tf::Vector3(0, 0, 1), -q[0]), tf::Vector3(0, 0, 0));
  tf::Transform r(tf::Quaternion(0,0,0,1), tf::Vector3(rot_radius, 0, 0));
  tf::Transform offset(rot_orientation, tf::Vector3(0, 0, 0));

  tf::poseTFToMsg(center * rotZ * r * offset, pose);
  return pose;
}

void RotationalModel::updateParameters(std::vector<double> delta)
{
  rot_center = rot_center + tf::Vector3(delta[0],delta[1],delta[2]);
  tf::Quaternion q;
  q.setRPY(delta[3],delta[4],0.00);
  rot_axis = rot_axis * q;

  rot_radius = rot_radius + delta[5];

  tf::Quaternion q2;
  q2.setRPY(delta[6],delta[7],delta[8]);
  rot_orientation = rot_orientation * q2;
}

bool RotationalModel::normalizeParameters() {
//	if(model.track.pose.size()>2) {
//		{
//			V_Configuration q = predictConfiguration(model.track.pose.front());
//			rot_axis = rot_axis * tf::Quaternion(tf::Vector3(0, 0, 1), -q[0]);
//		}
//		if(predictConfiguration(model.track.pose.back())[0]<0)
//			rot_axis = rot_axis * tf::Quaternion(tf::Vector3(1,0,0),M_PI);
//
//		rot_orientation = rot_axis.inverse() * poseToTransform(model.track.pose.front()).getRotation();
//	}
  return true;
}

bool RotationalModel::guessParameters()
{
  if(model_msg.track.pose.size() < 3)
    return false;

  size_t i,j,k;
  do
  {
    i = rand() % getSamples();
    j = rand() % getSamples();
    k = rand() % getSamples();
  } while (i==j || j==k || i==k);

  tf::Transform pose1, pose2, pose3;
  tf::poseMsgToTF(model_msg.track.pose[i], pose1);
  tf::poseMsgToTF(model_msg.track.pose[j], pose2);
  tf::poseMsgToTF(model_msg.track.pose[k], pose3);

//	if( pose1.getOrigin().distance(pose2.getOrigin())/sigma_position <
//		pose1.getRotation().angle(pose2.getRotation())/sigma_orientation
  if(rand() % 2 == 0)
  {
    rot_mode = 1;
//  cout <<"using rot computation"<<endl;

    // angle between pose 1 and pose 2
    double angle_12 = pose1.inverseTimes(pose2).getRotation().getAngle();

    // distance between pose 1 and pose 2
    double dist_12 = pose1.getOrigin().distance( pose2.getOrigin() );

    // rot axis between pose 1 and pose 2
    tf::Vector3 rot_12 = pose1.getBasis() * pose1.inverseTimes(pose2).getRotation().getAxis() * -1;
//  PRINT(rot_12);

    rot_center = tf::Vector3(0.5,0.5,0.5);
    rot_axis = tf::Quaternion(0,0,0,1);
    rot_orientation = tf::Quaternion(0,0,0,1);
    rot_radius = 0.0;

    rot_radius = (dist_12 * 0.5) / sin( angle_12 * 0.5 );
//  PRINT(pose1.getOrigin());
//  PRINT(pose2.getOrigin());
    tf::Vector3 c1 = (pose1.getOrigin() + pose2.getOrigin())/2;
//  PRINT(c1);
    tf::Vector3 v1 = (pose2.getOrigin() - pose1.getOrigin());
//  PRINT(v1);
    v1.normalize();
//  PRINT(v1);
    v1 = v1 - rot_12.dot(v1)*rot_12;
//  PRINT(v1);
    v1.normalize();
//  PRINT(v1);
//  PRINT(rot_12);
    rot_center = c1 + v1.cross(rot_12) * rot_radius * cos(angle_12/2);
//  PRINT(rot_center);


    tf::Vector3 d(1,0,0);
    d = pose1.getOrigin() - rot_center;
    d.normalize();
//  PRINT(d);

    tf::Vector3 rot_z = rot_12;
    tf::Vector3 rot_x = d - rot_z.dot(d)*rot_z;
    rot_x.normalize();
    tf::Vector3 rot_y = rot_z.cross(rot_x);
//  rot_x = tf::Vector3(-1,0,0);
//  rot_y = tf::Vector3(0,-1,0);
//  rot_z = tf::Vector3(0,0,-1);
    tf::Matrix3x3(
                rot_x.x(),rot_y.x(),rot_z.x(),
                rot_x.y(),rot_y.y(),rot_z.y(),
                rot_x.z(),rot_y.z(),rot_z.z()).getRotation(rot_axis);
//  PRINT(rot_x);
//  PRINT(rot_y);
//  PRINT(rot_z);
//  PRINT(rot_axis);
//  rot_axis=tf::Quaternion(0,0,0,1);

    rot_orientation = rot_axis.inverse() * pose1.getRotation();

    // eval ---------------------
    tf::Transform t_rotaxis(rot_axis,tf::Vector3(0,0,0));
    tf::Transform t_radius(tf::Quaternion(0,0,0,1),tf::Vector3(rot_radius,0,0));
    tf::Transform t_orient(rot_orientation,tf::Vector3(0,0,0));
    tf::Transform t_rotcenter(tf::Quaternion(0,0,0,1),rot_center);

// show diff to ppose1 and ppose2
    tf::Transform diff;
    tf::Transform pp1 =
                    t_rotcenter *
                    t_rotaxis *
                    tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),0.00),tf::Vector3(0,0,0)) *
                    t_radius *
                    t_orient;
    diff = pose1.inverseTimes(pp1);
//  cout <<"pp1: angle=" <<0.00<<" poserr="<<diff.getOrigin().length()<<" orienterr="<<diff.getRotation().getAngle()<<endl;

    tf::Transform pp2 =
                    t_rotcenter *
                    t_rotaxis *
                    tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),angle_12),tf::Vector3(0,0,0)) *
                    t_radius *
                    t_orient;
    diff = pose2.inverseTimes(pp2);
//  cout <<"pp2: angle=" <<angle_12<<" poserr="<<diff.getOrigin().length()<<" orienterr="<<diff.getRotation().getAngle()<<endl;

    for(size_t a=0;a<getSamples();a++)
    {
      V_Configuration q =predictConfiguration(model_msg.track.pose[a]);
      tf::Transform p2;
      tf::poseMsgToTF(predictPose(q),p2);

      tf::Transform temp_tf;
      tf::poseMsgToTF(model_msg.track.pose[a], temp_tf);
      diff = temp_tf.inverseTimes(p2);
//    cout <<"angle=" <<q[0]<<" poserr="<<diff.getOrigin().length()<<" orienterr="<<diff.getRotation().getAngle()<<endl;
//    tf::Transform pp =
//                      t_rotcenter *
//                      t_rotaxis *
//                      tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),-q[0]),tf::Vector3(0,0,0)) *
//                      t_radius *
//                      t_orient;
//    diff = poseToTransform(model.track.pose[a]).inverseTimes(pp);
//    cout <<"vs angle=" <<q[0]<<" poserr="<<diff.getOrigin().length()<<" orienterr="<<diff.getRotation().getAngle()<<endl;
    }
  } else
  {
    rot_mode = 0;
//  cout<<"using plane computation"<<endl;
    // first, find the plane
    tf::Vector3 plane_pos = pose1.getOrigin();
    tf::Vector3 plane_v = pose2.getOrigin() - pose1.getOrigin();
    tf::Vector3 plane_w = pose3.getOrigin() - pose1.getOrigin();
//  PRINT(plane_pos);
//  PRINT(plane_v);
//  PRINT(plane_w);
    plane_v.normalize();
    plane_w.normalize();

    tf::Vector3 plane_x = plane_v;
    tf::Vector3 plane_y = plane_w - (plane_w.dot(plane_v))*plane_v;
    plane_x.normalize();
    plane_y.normalize();
    tf::Vector3 plane_z = plane_x.cross(plane_y);
//  PRINT(plane_x);
//  PRINT(plane_y);
//  PRINT(plane_z);


    tf::Matrix3x3 plane_rot(
                    plane_x.x(),plane_y.x(),plane_z.x(),
                    plane_x.y(),plane_y.y(),plane_z.y(),
                    plane_x.z(),plane_y.z(),plane_z.z()
                    );

    tf::Transform plane(plane_rot,plane_pos);

    tf::Transform onplane_pose1 = plane.inverseTimes(pose1);
    tf::Transform onplane_pose2 = plane.inverseTimes(pose2);
    tf::Transform onplane_pose3 = plane.inverseTimes(pose3);
  //	cout <<"onplane_pose1"<<VEC2(onplane_pose1)<<endl;
  //	cout <<"onplane_pose2"<<VEC2(onplane_pose2)<<endl;
  //	cout <<"onplane_pose3"<<VEC2(onplane_pose3)<<endl;

  //http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
  tf::Vector3 p1 = (onplane_pose1.getOrigin() + onplane_pose2.getOrigin())/2;
  tf::Vector3 p21 = (onplane_pose2.getOrigin() - onplane_pose1.getOrigin()).rotate(tf::Vector3(0,0,1),M_PI/2);;

  tf::Vector3 p3 = (onplane_pose1.getOrigin() + onplane_pose3.getOrigin())/2;
  tf::Vector3 p43 = (onplane_pose3.getOrigin() - onplane_pose1.getOrigin()).rotate(tf::Vector3(0,0,1),M_PI/2);;

  tf::Vector3 p13 = p1 - p3;

  double u = 	( p43.x()*p13.y() - p43.y()*p13.x() ) /
                          ( p43.y()*p21.x() - p43.x()*p21.y() );
  tf::Vector3 onplane_center = p1 + u*p21;

  tf::Transform rotcenter(plane_rot,plane * onplane_center);
  rot_center = rotcenter.getOrigin();
  rot_axis = rotcenter.getRotation();

  rot_radius = rotcenter.inverseTimes(pose1).getOrigin().length();
  rot_orientation = tf::Quaternion(0,0,0,1);
  V_Configuration q = predictConfiguration( model_msg.track.pose[i]);
  //	cout <<"q="<<q[0]<<endl;
  tf::Transform pred1;
  tf::poseMsgToTF(predictPose(q),pred1);

  //	PRINT2(pose1);
  //	PRINT2(pred1);
  rot_orientation = pred1.inverseTimes(pose1).getRotation();
  }

//	cout<<"rot_radius="<<rot_radius<<endl;
//	PRINT(rot_center);
//	PRINT(rot_axis);
//	PRINT(rot_orientation);



  if(!check_values(rot_center)) return false;
  if(!check_values(rot_axis)) return false;
  if(!check_values(rot_radius)) return false;
  if(!check_values(rot_orientation)) return false;

  updateStateParametersToModel();

  return true;
}
