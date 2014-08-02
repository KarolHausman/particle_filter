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


articulation_model_msgs::TrackMsg generateMeasurement(const std::vector<geometry_msgs::Pose>& all_poses, const int& begin_idx, const int& end_idx)
{
  articulation_model_msgs::TrackMsg track;

  track.header.stamp = ros::Time::now();
  track.header.frame_id = "/world";
  track.header.seq = 0;


  if (begin_idx > end_idx)
  {
    std::cerr << "begin_idx is bigger than end_idx!" << std::endl;
    return track;
  }

  std::vector<geometry_msgs::Pose>::const_iterator first = all_poses.begin() + begin_idx;
  std::vector<geometry_msgs::Pose>::const_iterator last = all_poses.begin() + end_idx;

  if ((first >= all_poses.end()) || (last >= all_poses.end()))
  {
    std::cerr << "begin_idx or end_idx are bigger than the vector" << std::endl;
    return track;
  }

  std::vector<geometry_msgs::Pose> measurement_vector(first, last);
  track.pose = measurement_vector;
  return track;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_fitting");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.8);

  boost::normal_distribution<> nd(0.0, 0.01);
  boost::mt19937 rng;
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >
                  var_nor(rng, nd);

  ros::Publisher model_pub = n.advertise<articulation_model_msgs::ModelMsg> ("model_track", 5);



  // ------------------- generate data ----------------------------------
  bool rotational = true;
  bool prismatic = false;
  bool rigid = false;

  const int datapoints_number = 300;

  std::vector <geometry_msgs::Pose> generated_poses;

  for (int i = 0; i < datapoints_number; i++)
  {
    geometry_msgs::Pose pose;

    if (prismatic)
    {
      pose.position.x = 2 + (static_cast<float> (i)/100.0) + var_nor();
      pose.position.y = (static_cast<float> (i)/100.0) + var_nor();
      pose.position.z = (static_cast<float> (i)/100.0) + var_nor();
    }
    else if (rotational)
    {
      pose.position.x = 2 + cos(static_cast<float> (i) / 100.0) + var_nor();
      pose.position.y = sin(static_cast<float> (i) / 100.0) + var_nor();
      pose.position.z = var_nor();
    }
    else if(rigid)
    {
      pose.position.x = 2 + var_nor();
      pose.position.y = 4 + var_nor();
      pose.position.z = 1 + var_nor();
    }

    if (rotational)
    {
      double yaw = static_cast<float> (i)/100;
      double roll = M_PI/4;
      double pitch = 0;

      tf::Quaternion tf_pose_quat;
      tf_pose_quat.setRPY(roll, pitch, yaw);

      pose.orientation.w = tf_pose_quat.getW();
      pose.orientation.x = tf_pose_quat.getX();
      pose.orientation.y = tf_pose_quat.getY();
      pose.orientation.z = tf_pose_quat.getZ();
    }
    else
    {
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
    }
  generated_poses.push_back(pose);
  }



  const int initial_datapoints_number = 40;

  articulation_model_msgs::ModelMsg model_msg;
  articulation_model_msgs::ParamMsg sigma_param;



  sigma_param.name = "sigma_position";
  sigma_param.value = 0.02;
  sigma_param.type = articulation_model_msgs::ParamMsg::PRIOR;
  model_msg.params.push_back(sigma_param);


  model_msg.track = generateMeasurement(generated_poses, 0, initial_datapoints_number);



  std::cout << "creating object" << std::endl;
  ArticulationModelPtr rotational_instance(new RotationalModel);
  ArticulationModelPtr prismatic_instance(new PrismaticModel);
  ArticulationModelPtr rigid_instance(new RigidModel);


  model_msg.name = "rotational";
  rotational_instance->setModel(model_msg);
  model_msg.name = "prismatic";
  prismatic_instance->setModel(model_msg);
  model_msg.name = "rigid";
  rigid_instance->setModel(model_msg);

  std::cout << "fitting" << std::endl;
//  rotational_instance->fitModel();
//  prismatic_instance->fitModel();
//  rigid_instance->fitModel();


  int loop_count = 1;
  int generate_measurement_every_so_many = 10;

  while (ros::ok())
  {


    if (loop_count % generate_measurement_every_so_many == 0)
    {
      // ------------------------ generate measurement -------------------------------
      articulation_model_msgs::TrackMsg z;
      z = generateMeasurement(generated_poses, loop_count + initial_datapoints_number - generate_measurement_every_so_many, loop_count + initial_datapoints_number + 10);
      rotational_instance->addTrack(z);
      prismatic_instance->addTrack(z);
      rigid_instance->addTrack(z);
      std::cout << "measurement taken" << std::endl;
    }


    std::cout << "evaluating" << std::endl;
    rotational_instance->fitModel();
    prismatic_instance->fitModel();
    rigid_instance->fitModel();


    rotational_instance->evaluateModel();
    prismatic_instance->evaluateModel();
    rigid_instance->evaluateModel();

    std::cout << "done" << std::endl;


//    std::cout << "model class = "<< model_instance->getModelName() << std::endl;

//    std::cout << "       radius = "<<rotational_instance->getParam("rot_radius")<< std::endl;

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

    double rotational_loglikelihood = rotational_instance->getParam("loglikelihood");
    double prismatic_loglikelihood = prismatic_instance->getParam("loglikelihood");
    double rigid_loglikelihood = rigid_instance->getParam("loglikelihood");

    std::cout << "     rotational log LH = " << rotational_loglikelihood<< "\n " << std::endl;
    std::cout << "     prismatic log LH = " << prismatic_loglikelihood<< "\n "  << std::endl;
    std::cout << "     rigid log LH = " << rigid_loglikelihood << "\n \n" << std::endl;


    double max_loglikehood = std::max(std::max(rotational_loglikelihood, prismatic_loglikelihood), rigid_loglikelihood);
    rotational_loglikelihood = rotational_loglikelihood - max_loglikehood;
    prismatic_loglikelihood = prismatic_loglikelihood - max_loglikehood;
    rigid_loglikelihood = rigid_loglikelihood - max_loglikehood;

    double rotational_likelihood = exp(rotational_loglikelihood);
    double prismatic_likelihood = exp(prismatic_loglikelihood);
    double rigid_likelihood = exp(rigid_loglikelihood);

    double sum_likelihoods = rotational_likelihood + prismatic_likelihood + rigid_likelihood;

    rotational_likelihood = rotational_likelihood/sum_likelihoods;
    prismatic_likelihood = prismatic_likelihood/sum_likelihoods;
    rigid_likelihood = rigid_likelihood/sum_likelihoods;

    std::cout << "     rotational normalized likelihood = " << rotational_likelihood<< "\n " << std::endl;
    std::cout << "     prismatic normalized likelihood = " << prismatic_likelihood<< "\n "  << std::endl;
    std::cout << "     rigid normalized likelihood = " << rigid_likelihood << "\n \n" << std::endl;


    rotational_instance->setParam("weight",rotational_likelihood,articulation_model_msgs::ParamMsg::EVAL);
    prismatic_instance->setParam("weight",prismatic_likelihood,articulation_model_msgs::ParamMsg::EVAL);
    rigid_instance->setParam("weight",rigid_likelihood,articulation_model_msgs::ParamMsg::EVAL);



    std::cout << "     rotational BIC = " << rotational_instance->getParam("bic")<< "\n " << std::endl;
    std::cout << "     prismatic BIC = " << prismatic_instance->getParam("bic")<< "\n "  << std::endl;
    std::cout << "     rigid BIC = " << rigid_instance->getParam("bic") << "\n \n" << std::endl;


//    std::cout << " points= "<<model_msg.track.pose.size() << "\n " << std::endl;

    model_pub.publish(rotational_instance->getModel());

    model_pub.publish(prismatic_instance->getModel());

    model_pub.publish(rigid_instance->getModel());



    ros::spinOnce();
    loop_rate.sleep();
    loop_count++;
  }
}
