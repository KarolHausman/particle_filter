
#include <iostream>
#include "particle_filter/particle.h"
#include "particle_filter/paricle_filter.h"
#include "particle_filter/identity_motionmodel.h"
#include "particle_filter/articulation_marker_sensormodel.h"
#include "particle_filter/visualizer.h"

#include "articulation_model_msgs/ModelMsg.h"
#include "articulation_model_msgs/TrackMsg.h"

#include "pr2_lfd_utils/WMData.h"


#include "particle_filter/random.h"

#include "particle_filter/articulation_io_utils.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

articulation_model_msgs::TrackMsg data_track;
articulation_model_msgs::TrackMsg incremental_track;
int incremental_track_msgs_counter = 0;


// -------------------------------- generate measurement -----------------------
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


void trackCB(const articulation_model_msgs::TrackMsgConstPtr msg)
{
  data_track = *msg;
}

//TODO:marker ID as a param
void trackIncrementalCB(const pr2_lfd_utils::WMDataConstPtr msg)
{
  for (std::vector<pr2_lfd_utils::WMObject>::const_iterator it = msg->objects.begin(); it != msg->objects.end(); ++it)
  {
    if (it->id == 12)
    {
      incremental_track.header.seq = incremental_track_msgs_counter;
      ++incremental_track_msgs_counter;
      std::cerr << "incremental counter: " << incremental_track_msgs_counter << std::endl;
      incremental_track.pose.push_back(it->pose.pose);
    }

  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "particle_filter");

  Visualizer::getInstance()->init();

  // -------------------------------- motion and sensor models ----------------

  MotionModel<ArticulationModelPtr>* motionModel = new IdentityMotionModel<ArticulationModelPtr>;
  SensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>* sensorModel = new ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;

  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(10, 10);
  Eigen::VectorXd u = Eigen::VectorXd::Ones(10);
  Eigen::MatrixXd motionNoiseCov = covariance / 1000;
  Eigen::MatrixXd sensorNoiseCov = covariance / 2;



  // -------------------------------- generate data -----------------------
  boost::normal_distribution<> nd(0.0, 0.01);
  boost::mt19937 rng;
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >
                  var_nor(rng, nd);

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
  // -------------------------------- end of generate data -----------------------



  //---------------------------------- particle filter initalization ----------
  const int particles_number = 300;
  const int initial_datapoints_number = 50;

  articulation_model_msgs::ModelMsg model_msg;
  articulation_model_msgs::ParamMsg sigma_param;
  sigma_param.name = "sigma_position";
  sigma_param.value = 0.02;
  sigma_param.type = articulation_model_msgs::ParamMsg::PRIOR;
  model_msg.params.push_back(sigma_param);


  //use first initial_datapoints_number datapoints
//  model_msg.track = generateMeasurement(generated_poses, 0, initial_datapoints_number);

  const int initial_trackdatapoints_number = 30;
  ros::NodeHandle nh;
  ros::Subscriber data_track_sub = nh.subscribe("ar_world_model", 1000, trackIncrementalCB);
  ros::Subscriber track_sub = nh.subscribe("marker_topic",1, trackCB);
  bool incremental = true;
  //------------------------------- track from recorded data --------------------
  if (!incremental)
  {
    bool got_track = false;
    while (ros::ok() && !got_track)
    {
      ros::spinOnce();
      if (!data_track.header.stamp.isZero())
      {
        model_msg.track = data_track;
        std::cout << "taking: " << model_msg.track.pose.size() << " poses" << std::endl;
        got_track = true;
      }
    }
  }
  else
  {
  // ------------------------------- track from data as it comes ------------------
    bool got_track_for_init = false;
    incremental_track.header.stamp = ros::Time::now();
    incremental_track.header.frame_id = "/world";

    while (ros::ok() && !got_track_for_init)
    {
      ros::spinOnce();
      std::cerr << "incremental_track seq: " << incremental_track.header.seq << std::endl;
      if (incremental_track.header.seq >= initial_trackdatapoints_number)
      {
        model_msg.track = incremental_track;
        std::cout << "taking: " << model_msg.track.pose.size() << " poses" << std::endl;
        //remove all the poses used already
        incremental_track.pose.clear();
        got_track_for_init = true;
      }
    }
  }




  //----------------------------------init of particle filter -------------------------

  //slower init
//  ParticleFilter<ArticulationModelPtr> pf (particles_number, model_msg);

  //faster init
  ParticleFilter<ArticulationModelPtr> pf (particles_number, model_msg, *motionModel, motionNoiseCov, motionNoiseCov, motionNoiseCov);



  //  ------------------------------ particle filter loop ------------------------------

  ros::Rate r(2);
  uint loop_count = 1;

  while (ros::ok())
  {
    //get the measurement every iteration
    ros::spinOnce();



    ROS_INFO_STREAM ("loop_count: " << loop_count);
    pf.propagate(u, motionNoiseCov, *motionModel);

    if (loop_count % 10 == 0)
    {

      ROS_INFO ("Correction step started.");

      // ------------------------ generate measurement -------------------------------

      //start with 20
      articulation_model_msgs::TrackMsg z;
//      z = generateMeasurement(generated_poses, loop_count + initial_datapoints_number - 10, loop_count + initial_datapoints_number + 0);

      //check if there was enough time
      if (incremental_track.header.seq >= initial_trackdatapoints_number + loop_count)
      {
        z = incremental_track;
        std::cout << "taking: " << z.pose.size() << " poses" << std::endl;
        incremental_track.pose.clear();
      }
      ROS_INFO_STREAM ("measurement taken");

      ROS_INFO_STREAM ("executing correction step");
      pf.correct<articulation_model_msgs::TrackMsg>(z, sensorNoiseCov, *sensorModel);

      ROS_INFO_STREAM ("adding particles");
      pf.addParticles(1, 1, 1);

      if (!pf.normalize())
      {
        ROS_ERROR ("no particles left, quiting");
        return -1;
      }

      pf.sortParticles();
      Visualizer::getInstance()->publishParticles(pf.particles);


      std::cerr << "ALL TOGETHER TOOK: " << pf.particles[0].state->getModel().track.pose.size() << " poses" << std::endl;
      ROS_INFO ("Correction step executed.");
      if (!pf.resample(particles_number))
      {
        ROS_ERROR ("no particles left, quiting");
        return -1;
      }
      ROS_INFO ("Resample step executed.");
//      pf.printParticles();
    }
    r.sleep();
    ++loop_count;

//    double fraction = 0.02;
//    ROS_INFO_STREAM ("weighted average of " << fraction << " equals: \n" <<
//                     pf.getWeightedAvg(fraction));
  }
  delete motionModel;
  delete sensorModel;
}
