
#include <iostream>
#include "particle_filter/particle.h"
#include "particle_filter/paricle_filter.h"
#include "particle_filter/identity_motionmodel.h"
#include "particle_filter/articulation_marker_sensormodel.h"
#include "particle_filter/articulation_data_sensormodel.h"
#include "particle_filter/visualizer.h"

#include "articulation_model_msgs/ModelMsg.h"
#include "articulation_model_msgs/TrackMsg.h"

#include "pr2_lfd_utils/WMData.h"

#include "particle_filter/random.h"

#include "particle_filter/articulation_io_utils.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "particle_filter/articulation_manip_sensor_action_model.h"

#include "particle_filter/handle_finder.h"


articulation_model_msgs::TrackMsg data_track;
articulation_model_msgs::TrackMsg incremental_track;
articulation_model_msgs::TrackMsg full_track;

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
  full_track.pose.insert(full_track.pose.end(), track.pose.begin(), track.pose.end());

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
      incremental_track.pose.push_back(it->pose.pose);
      full_track.pose.push_back(it->pose.pose);
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "particle_filter");

  Visualizer::getInstance()->init();


  // ------------------------ find handle -----------------------------------------

  HandleFinder hf;
  hf.findHandle("ar_marker_15");
  Eigen::VectorXd pregrasp_offset(6);
  pregrasp_offset << -0.4, -0.25, 0.05, M_PI/2, 0.0, 0.0;
  Eigen::Vector3d grasp_offset(0.115, 0, 0);
  hf.executeHandleGrasp(pregrasp_offset, grasp_offset);


// ---------------------------------- options to run it -------------------------------------
  bool incremental = true;

  bool use_generated_data = false;
  bool double_arcs = false;

  bool hierarchical_pf = true;

  // -------------------------------- motion and sensor models ----------------

  MotionModel<ArticulationModelPtr>* motionModel = new IdentityMotionModel<ArticulationModelPtr>;
  SensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>* sensorModel;
  if(incremental)
  {
    sensorModel = new ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;
  }
  else
  {
    sensorModel = new ArtDataSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;
  }

  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(10, 10);
  Eigen::VectorXd u = Eigen::VectorXd::Ones(10);
  Eigen::MatrixXd motionNoiseCov = covariance / 10000;

  //orientation
  motionNoiseCov(3,3) = motionNoiseCov(4,4) = motionNoiseCov(5,5) = 0.001;
  //rest
//  motionNoiseCov(6,6) = motionNoiseCov(7,7) = motionNoiseCov(8,8) = motionNoiseCov(9,9)  = 0.001;

  Eigen::MatrixXd sensorNoiseCov = covariance / 2;



  // -------------------------------- generate data -----------------------
  boost::normal_distribution<> nd(0.0, 0.01);
  boost::mt19937 rng;
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >
                  var_nor(rng, nd);
  std::vector <geometry_msgs::Pose> generated_poses;


  const int datapoints_number = 100;
  if (use_generated_data)
  {
    bool rotational = true;
    bool prismatic = false;
    bool rigid = false;


    for (int i = 0; i < datapoints_number; i++)
    {
      geometry_msgs::Pose pose,pose2;

      if (prismatic)
      {
        pose.position.x = 2 + (static_cast<float> (i)/100.0) + var_nor();
        pose.position.y = (static_cast<float> (i)/100.0) + var_nor();
        pose.position.z = (static_cast<float> (i)/100.0) + var_nor();
      }
      else if (rotational)
      {
        pose.position.x = 0 + cos(static_cast<float> (i) / 100.0) + var_nor();
        pose.position.y = sin(static_cast<float> (i) / 100.0) + var_nor();
        pose.position.z = var_nor();

        pose2.position.x = cos(static_cast<float> (i) / 100.0) + var_nor();
        pose2.position.y = 2 + sin(static_cast<float> (i) / 100.0) + var_nor();
        pose2.position.z = var_nor();

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
        pose2.orientation = pose.orientation;
      }
      else
      {
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
      }
    generated_poses.push_back(pose);
    if (rotational && double_arcs)
      generated_poses.push_back(pose2);
    }
  }
  // -------------------------------- end of generate data -----------------------



  //---------------------------------- particle filter initalization ----------
  const int particles_number = 300;

  //params for generated data
  int initial_datapoints_number = datapoints_number - 1;
  if (incremental)
    initial_datapoints_number = 10;

  articulation_model_msgs::ModelMsg model_msg;
//  articulation_model_msgs::ParamMsg sigma_param;
//  sigma_param.name = "sigma_position";
//  sigma_param.value = 0.02;
//  sigma_param.type = articulation_model_msgs::ParamMsg::PRIOR;
//  model_msg.params.push_back(sigma_param);

  //params for real data
  const int initial_trackdatapoints_number = 30;
  ros::NodeHandle nh;
  ros::Subscriber data_track_sub = nh.subscribe("ar_world_model", 1000, trackIncrementalCB);
  ros::Subscriber track_sub = nh.subscribe("marker_topic",1, trackCB);


  full_track.header.stamp = ros::Time::now();
  full_track.header.frame_id = "/world";

  //use first initial_datapoints_number datapoints
  if (use_generated_data)
  {
    model_msg.track = generateMeasurement(generated_poses, 0, initial_datapoints_number);
    full_track = model_msg.track;
  }
  else
  {
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


      std::cerr << "Initializing particles... " << std::endl;
      while (ros::ok() && !got_track_for_init)
      {
        ros::spinOnce();
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
  }




  //----------------------------------init of particle filter -------------------------

  //slower init
//  ParticleFilter<ArticulationModelPtr> pf (particles_number, model_msg);

  //faster init
  ParticleFilter<ArticulationModelPtr> pf (particles_number, model_msg, *motionModel, motionNoiseCov, motionNoiseCov, motionNoiseCov);


  //  ------------------------------ particle filter loop ------------------------------

  ros::Rate r(2);
  uint loop_count = 1;
  uint measurement_cnt = 0;
  ros::Publisher model_pub = nh.advertise<articulation_model_msgs::ModelMsg>("model_track", 10);
  articulation_model_msgs::ModelMsg full_model;

  while (ros::ok())
  {
    //get the measurement every iteration
    ros::spinOnce();



    ROS_INFO_STREAM ("loop_count: " << loop_count);
    pf.propagate(pf.particles, u, motionNoiseCov, *motionModel);

    Visualizer::getInstance()->publishParticlesOnly(pf.particles);

    if(incremental)
    {
      full_model.header = full_track.header;
      full_model.track = full_track;
      model_pub.publish(full_model);
    }

    if (loop_count % 10 == 0)
    {

      ROS_INFO ("Correction step started.");

      // ------------------------ generate measurement -------------------------------

      //start with 20
      articulation_model_msgs::TrackMsg z;
      if (use_generated_data)
      {
//        z = generateMeasurement(generated_poses, loop_count + initial_datapoints_number - 10, loop_count + initial_datapoints_number + 0);
        z = generateMeasurement(generated_poses, initial_datapoints_number + 30 * measurement_cnt, initial_datapoints_number + 30*(measurement_cnt+1));
        ++measurement_cnt;

      }
      else
      {
        if (incremental)
        {
          //check if there was enough time
          if (incremental_track.header.seq >= initial_trackdatapoints_number + loop_count)
          {
            z = incremental_track;
            std::cout << "taking: " << z.pose.size() << " poses" << std::endl;
            incremental_track.pose.clear();
          }
        }
      }


      ROS_INFO_STREAM ("measurement taken");

      if (incremental)
      {
        if(loop_count >= 50 && loop_count < 60 )
        {
          ROS_INFO_STREAM ("adding particles");
          pf.addParticles(full_track, 30, 50, 30, 2);
        }
      }
      else
      {
        if(loop_count >= 20 && loop_count < 30 )
        {
          ROS_INFO_STREAM ("adding particles");
          pf.addParticles(model_msg.track, 30, 50, 30, 2);
        }
      }

      ROS_INFO_STREAM ("executing correction step");
      pf.correct<articulation_model_msgs::TrackMsg>(pf.particles, z, sensorNoiseCov, *sensorModel);

      //add action here I think....
      //get the best prismatic particle
//      pf.sortParticles(pf.particles);
//      boost::shared_ptr<PrismaticModel> p;
//      for (typename std::vector <Particle <ArticulationModelPtr> >::const_reverse_iterator it = pf.particles.rbegin(); it != pf.particles.rend();
//           it++)
//      {
//        if (it->state->model == PRISMATIC)
//        {
//          p = boost::dynamic_pointer_cast< PrismaticModel > (it->state);
//          break;
//        }
//      }

//      ActionPtr a(new ActionPrismatic(*p));
//      int z_action = 1;
//      boost::shared_ptr < SensorActionModel<ArticulationModelPtr, int, ActionPtr> > sensorActionModel (new ArtManipSensorActionModel<ArticulationModelPtr, int, ActionPtr>);
//      pf.correctAction<int, ActionPtr>(pf.particles, z_action, a, sensorNoiseCov, *sensorActionModel);

      // ---- end of action sensor model ---


      pf.sortParticles(pf.particles);
      pf.printParticles(pf.particles);

      std::cerr << "ALL TOGETHER TOOK: " << full_track.pose.size() << " poses" << std::endl;
      ROS_INFO ("Correction step executed.");
      if (hierarchical_pf)
      {
        // visualization only !!! doesnt normalize the weights
        pf.normalize(pf.particles, true);
        pf.sortParticles(pf.particles);
        Visualizer::getInstance()->publishParticles(pf.particles);

        double weights_rigid, weights_prismatic, weights_rotational, weights_free;
        pf.splitArticulationModels(pf.particles, weights_rigid, weights_prismatic, weights_rotational, weights_free);
        if ((!pf.normalize(pf.particles_rigid)) || (!pf.normalize(pf.particles_prismatic)) || (!pf.normalize(pf.particles_rotational)))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }
      }
      else
      {
        if (!pf.normalize(pf.particles))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }
      }

      if(!hierarchical_pf)
      {
        pf.sortParticles(pf.particles);
        Visualizer::getInstance()->publishParticles(pf.particles);
      }





      if(hierarchical_pf)
      {
        if ((!pf.stratifiedResample(particles_number/3, pf.particles_rigid)) || (!pf.stratifiedResample(particles_number/3, pf.particles_prismatic)) || (!pf.stratifiedResample(particles_number/3, pf.particles_rotational)))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }
      }
      else
      {
        if (!pf.stratifiedResample(particles_number, pf.particles))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }
      }
      ROS_INFO ("Resample step executed.");
//      pf.printParticles(pf.particles);

      if (hierarchical_pf)
      {
        pf.mergeArticulationModels();
      }
      pf.removeAddedParticleFlags(pf.particles);
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
