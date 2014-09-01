
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

#include "particle_filter/action.h"

#include "particle_filter/action_generator.h"


articulation_model_msgs::TrackMsg data_track;

void trackCB(const articulation_model_msgs::TrackMsgConstPtr msg)
{
  data_track = *msg;
}







int main(int argc, char **argv)
{

  ros::init(argc, argv, "particle_filter");
  Visualizer::getInstance()->init();




  // ------------------------ find handle -----------------------------------------
//   drawer
  /*ROS_INFO("Grasping handle");
  HandleFinder hf;
  hf.findHandle("ar_marker_15");
  Eigen::VectorXd pregrasp_offset(6);
  pregrasp_offset << -0.4, -0.25, 0.05, M_PI/2, 0.0, 0.0;
  Eigen::Vector3d grasp_offset(0.175, 0, 0);
  hf.executeHandleGrasp(pregrasp_offset, grasp_offset);
  ROS_INFO("Handle grasp executed");*/

  // erasers
  /*HandleFinder hf;
  hf.findHandle("ar_marker_2");
  Eigen::VectorXd pregrasp_offset(6);
  pregrasp_offset << 0.0, -0.05, 0.2, M_PI/2, M_PI/2, 0.0 ;
  Eigen::Vector3d grasp_offset(0.0, 0.0, -0.05);
  hf.executeHandleGrasp(pregrasp_offset, grasp_offset);
  ROS_INFO("Handle grasp executed");*/


  // -------------------------------- motion and sensor models ----------------

  MotionModel<ArticulationModelPtr>* motionModel = new IdentityMotionModel<ArticulationModelPtr>;
  SensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>* sensorModel;
  sensorModel = new ArtDataSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;


  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(10, 10);
  Eigen::VectorXd u = Eigen::VectorXd::Ones(10);
  Eigen::MatrixXd motionNoiseCov = covariance / 10000;

  //orientation
  motionNoiseCov(3,3) = motionNoiseCov(4,4) = motionNoiseCov(5,5) = 0.001;
  //rest
//  motionNoiseCov(6,6) = motionNoiseCov(7,7) = motionNoiseCov(8,8) = motionNoiseCov(9,9)  = 0.001;

  Eigen::MatrixXd sensorNoiseCov = covariance / 2;




  //---------------------------------- particle filter initalization ----------

  const int particles_number = 500;
  //params for real data
  ros::NodeHandle nh;
  ros::Subscriber track_sub = nh.subscribe("marker_topic",1, trackCB);

    //------------------------------- track from recorded data --------------------

  articulation_model_msgs::ModelMsg model_msg;
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

  ParticleFilter<ArticulationModelPtr> pf (particles_number, model_msg, *motionModel, motionNoiseCov, motionNoiseCov, motionNoiseCov);





  ActionGenerator action_gen;
  std::vector <tf::Vector3> generated_actions = action_gen.generateActionDirections(5, 3);


  ActionPtr action(new Action);

  //  ------------------------------ particle filter loop ------------------------------

  ros::Rate r(2);
  uint loop_count = 1;
  tf::TransformListener tf_listener;
  tf::StampedTransform marker_static_to_marker;

  bool hierarchy = true;

  while (ros::ok())
  {

// ------------------ current marker measurement ------------------------------------
    try
    {
      // static marker, moving marker
      tf_listener.lookupTransform("ar_marker_1", "/ar_marker_2", ros::Time(0), marker_static_to_marker);
//      tf_listener.lookupTransform("ar_marker_4", "/ar_marker_15", ros::Time(0), marker_static_to_marker);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    for (std::vector <Particle <ArticulationModelPtr> >::iterator it = pf.particles.begin(); it!= pf.particles.end(); ++it)
    {
      it->state->setParam("current_pose_trans", marker_static_to_marker.getOrigin(), articulation_model_msgs::ParamMsg::PRIOR);
      it->state->setParam("current_pose_quat", marker_static_to_marker.getRotation(), articulation_model_msgs::ParamMsg::PRIOR);

      geometry_msgs::Pose pose_obs, pose_proj;
      V_Configuration q;
      tf::poseTFToMsg(marker_static_to_marker, pose_obs);
      it->state->getCurrentPoseProjected(pose_obs, pose_proj, q);
      tf::Transform tf_pose_proj;
      tf::poseMsgToTF(pose_proj,tf_pose_proj);

      it->state->setParam("current_proj_pose_trans", tf_pose_proj.getOrigin(), articulation_model_msgs::ParamMsg::PRIOR);
      it->state->setParam("current_proj_pose_quat", tf_pose_proj.getRotation(), articulation_model_msgs::ParamMsg::PRIOR);

      if (it->state->model_msg.name == "rotational")
      {
        tf::Vector3 rot_proj_dir;
        boost::shared_ptr<RotationalModel> rotational_model = boost::dynamic_pointer_cast< RotationalModel > (it->state);
        tf::Matrix3x3 rot_axis_m(rotational_model->rot_axis);
        tf::Vector3 rot_axis_z = rot_axis_m.getColumn(2);
        tf::Vector3 radius = tf_pose_proj.getOrigin() - rotational_model->rot_center;
        rot_proj_dir = radius.cross(rot_axis_z);
        it->state->setParam("current_proj_pose_rot_dir", rot_proj_dir, articulation_model_msgs::ParamMsg::PRIOR);
      }
    }



// ----------------- propagate particles -------------------------------------------
    ros::spinOnce();
    ROS_INFO_STREAM ("loop_count: " << loop_count);
    pf.propagate(pf.particles, u, motionNoiseCov, *motionModel);

    Visualizer::getInstance()->publishParticlesOnly(pf.particles);

    if (loop_count % 10 == 0)
    {



// ----------------- adding particles -------------------
      if(loop_count == 20)
      {
        ROS_INFO_STREAM ("adding particles");
        pf.addParticles(model_msg.track, 30, 50, 30, 0);
      }





// ----------------- marker correction step -------------------
      ROS_INFO ("executing correction step");
      articulation_model_msgs::TrackMsg z;     
      pf.correct<articulation_model_msgs::TrackMsg>(pf.particles, z, sensorNoiseCov, *sensorModel);
      ROS_INFO ("Correction step executed.");










// ---------------- action selection -------------------------------
      std::vector <Particle <ArticulationModelPtr> > temp_particles;
      temp_particles = pf.particles;

      pf.normalize(temp_particles);

      if (hierarchy)
      {
        pf.sortParticles(temp_particles);
        pf.printParticles(temp_particles);
      }

      pf.weightsToLogWeights(temp_particles);


      ROS_INFO("Entropy: %f", pf.calculateEntropy(temp_particles));

      action_gen.publishGenActions();

      boost::shared_ptr < SensorActionModel<ArticulationModelPtr, int, ActionPtr> > sensorActionModel (new ArtManipSensorActionModel<ArticulationModelPtr, int, ActionPtr>);
      double min_expected_entropy = std::numeric_limits<double>::max();
      double max_expected_downweight = -std::numeric_limits<double>::max();

      tf::Vector3 best_action;
      for (std::vector<tf::Vector3>::iterator it = generated_actions.begin(); it!= generated_actions.end(); ++it)
      {
        action->setActionDirection(*it);
        double za_expected = pf.calculateExpectedZaArticulation<int, ActionPtr>(temp_particles, action, sensorNoiseCov, *sensorActionModel);
        ROS_INFO("Expected Za: %f", za_expected);
        ROS_INFO("Action: x = %f, y = %f, z = %f", it->getX(), it->getY(), it->getZ());
        double expected_entropy = pf.calculateExpectedEntropy<int, ActionPtr>(temp_particles, za_expected, action, sensorNoiseCov, *sensorActionModel);
        ROS_ERROR("Expected Entropy: %f \n", expected_entropy);
        double expected_downweight = pf.calculateExpectedDownweightAfterAction<int, ActionPtr>(temp_particles, za_expected, action, sensorNoiseCov, *sensorActionModel);
        ROS_ERROR("Expected Downweight: %f \n \n", expected_downweight);

        if (expected_entropy < min_expected_entropy)
        {
          min_expected_entropy = expected_entropy;
          best_action = *it;
        }

//        if (expected_downweight > max_expected_downweight)
//        {
//          max_expected_downweight = expected_downweight;
//          best_action = *it;
//        }
      }
      action->plan(best_action);






// ---------------------action execution ---------------------------
      if(loop_count >= 40)
      {
        hierarchy = false;
        pf.normalize(pf.particles);
        pf.weightsToLogWeights(pf.particles);

        ROS_INFO("Executing action correction step");

//        drawer
//        bool success = action->execute(best_action, "ar_marker_15");

//        eraser
        bool both_ways = false;
        bool success = action->execute(best_action, "ar_marker_2", both_ways);
        std::cerr << "Action successful? " << success << std::endl;
//        // 1 - doesnt stop
        int z_action = action->getActionResult();
//        int z_action = 0;

        boost::shared_ptr < SensorActionModel<ArticulationModelPtr, int, ActionPtr> > sensorActionModelExecution (new ArtManipSensorActionModel<ArticulationModelPtr, int, ActionPtr>);
        pf.correctAction<int, ActionPtr> (pf.particles, z_action, action, sensorNoiseCov, *sensorActionModelExecution);
        ROS_INFO("Action correction step executed");
      }




// -----------------------visualization only !!! doesnt normalize the weights -----------------
      pf.normalize(pf.particles, true);
      pf.sortParticles(pf.particles);
      Visualizer::getInstance()->publishParticles(pf.particles);





// ----------------- normalize and resample ------------------------------------
      ROS_INFO("Executing normalization and resampling step");
      if (hierarchy)
      {
        //TODO: think of free model here, but it should be fine I think...
        pf.splitArticulationModels();
        if ((!pf.normalize(pf.particles_rigid)) || (!pf.normalize(pf.particles_prismatic)) || (!pf.normalize(pf.particles_rotational)))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }
        if ((!pf.stratifiedResample(particles_number/3, pf.particles_rigid)) || (!pf.stratifiedResample(particles_number/3, pf.particles_prismatic)) || (!pf.stratifiedResample(particles_number/3, pf.particles_rotational)))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }
        pf.mergeArticulationModels();
      }
      else
      {
        if (!pf.normalize(pf.particles))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }

        pf.sortParticles(pf.particles);
        pf.printParticles(pf.particles);


        if (!pf.stratifiedResample(particles_number, pf.particles))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }
      }
      ROS_INFO ("Resample step executed.");

      pf.removeAddedParticleFlags(pf.particles);
      ROS_INFO("Normalization and resampling step executed");
    }
    r.sleep();
    ++loop_count;
  }





  delete motionModel;
  delete sensorModel;
}
