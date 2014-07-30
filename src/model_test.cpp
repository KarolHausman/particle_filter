/*
 * model_fitting_demo.cpp
 *
 *  Created on: Jun 8, 2010
 *      Author: sturm
 */

#include <ros/ros.h>

//#include "articulation_models/models/factory.h"

#include "articulation_model_msgs/ModelMsg.h"
#include "articulation_model_msgs/TrackMsg.h"
#include "articulation_model_msgs/ParamMsg.h"
#include "particle_filter/articulation_model.h"
#include "particle_filter/rotational_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rigid_model.h"


#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_fitting");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);
  int count = 0;

  boost::normal_distribution<> nd(0.0, 0.01);
  boost::mt19937 rng;
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >
                  var_nor(rng, nd);

  ros::Publisher model_pub = n.advertise<articulation_model_msgs::ModelMsg> ("model_track", 5);


  while (ros::ok())
  {
    articulation_model_msgs::ModelMsg model_msg;
    model_msg.name = "rotational";
//    model_msg.name = "prismatic";
//    model_msg.name = "rigid";

    articulation_model_msgs::ParamMsg sigma_param;
    sigma_param.name = "sigma_position";
    sigma_param.value = 0.02;
    sigma_param.type = articulation_model_msgs::ParamMsg::PRIOR;
    model_msg.params.push_back(sigma_param);

    model_msg.track.header.stamp = ros::Time();
    model_msg.track.header.frame_id = "/world";
//    model_msg.track.track_type = articulation_model_msgs::TrackMsg::TRACK_POSITION_ONLY;

    for (int i = 0; i < 150; i++)
    {
      geometry_msgs::Pose pose;
//      pose.position.x = 2 + (static_cast<float> (i)/100.0) + var_nor();
      pose.position.x = cos(static_cast<float> (i) / 100.0 + count / 10.0) + var_nor();
//      pose.position.x = 2 + var_nor();


//      pose.position.y = (static_cast<float> (i)/100.0) + var_nor();
      pose.position.y = sin(static_cast<float> (i) / 100.0 + count / 10.0) + var_nor();
//      pose.position.y = 4 + var_nor();


      pose.position.z = var_nor();
//      pose.position.z = (static_cast<float> (i)/100.0) + var_nor();
//      pose.position.z = 1 + var_nor();


      double yaw = static_cast<float> (i)/100;
      double roll = 0;
      double pitch = 0;

      tf::Quaternion tf_pose_quat;
      tf_pose_quat.setRPY(roll, pitch, yaw);

      pose.orientation.x = tf_pose_quat.getX();
      pose.orientation.y = tf_pose_quat.getY();
      pose.orientation.z = tf_pose_quat.getZ();
      pose.orientation.w = tf_pose_quat.getW();

//      pose.orientation.x = 0;
//      pose.orientation.y = 0;
//      pose.orientation.z = 0;
//      pose.orientation.w = 1;
      model_msg.track.pose.push_back(pose);
    }
    model_msg.track.header.stamp =  ros::Time::now();
    model_msg.track.header.seq = 0;

    std::cout << "creating object" << std::endl;
    ArticulationModelPtr model_instance(new RotationalModel);
//    ArticulationModelPtr model_instance(new PrismaticModel);
//    ArticulationModelPtr model_instance(new RigidModel);

    model_instance->setModel(model_msg);

    std::cout << "fitting" << std::endl;
    model_instance->fitModel();

    std::cout << "evaluating" << std::endl;
    model_instance->evaluateModel();
    std::cout << "done" << std::endl;


//    std::cout << "model class = "<< model_instance->getModelName() << std::endl;

//    std::cout << "       radius = "<<model_instance->getParam("rot_radius")<< std::endl;

//    boost::shared_ptr<RotationalModel> rotational = boost::dynamic_pointer_cast< RotationalModel > (model_instance);
//    std::cout << "    rotational   radius = "<<rotational->rot_radius<< std::endl;

//    tf::Matrix3x3 m(rotational->rot_axis);
//    double yaw, pitch, roll;
//    m.getEulerYPR(yaw, pitch, roll);
//    std::cout << "    rotational   yaw = "<<yaw<< std::endl;
//    std::cout << "    rotational   pitch = "<<pitch<< std::endl;
//    std::cout << "    rotational   roll = "<<roll<< std::endl;


////    std::cout << "       rigid_position.x = "<<model_instance->getParam("rigid_position.x")<< std::endl;
////    boost::shared_ptr<RigidModel> rigid = boost::dynamic_pointer_cast< RigidModel > (model_instance);
////    std::cout << "     pos.x = " << rigid->pos_x << std::endl;



//    std::cout << "       center.x = "<<model_instance->getParam("rot_center.x")<< std::endl;
//    std::cout << "       center.y = "<<model_instance->getParam("rot_center.y")<< std::endl;
//    std::cout << "       center.z = "<<model_instance->getParam("rot_center.z")<< std::endl;
    std::cout << "       log LH = " << model_instance->getParam("loglikelihood") << std::endl; //TODO: change back to getLikelihood
    std::cout << " points= "<<model_msg.track.pose.size()<< std::endl;

    model_pub.publish(model_instance->getModel());
//    ModelMsg fitted_model_msg = model_instance->getModel();

    ros::spinOnce();
    loop_rate.sleep();
//    ++count;
  }
}
