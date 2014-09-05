
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


#include "particle_filter/kernel_density_estimator.h"
#include <iostream>
#include <fstream>

articulation_model_msgs::TrackMsg data_track;

void trackCB(const articulation_model_msgs::TrackMsgConstPtr msg)
{
  data_track = *msg;
}







int main(int argc, char **argv)
{

  ros::init(argc, argv, "particle_filter");
  Visualizer::getInstance()->init();



  // 1D case
//  {
//    std::vector<double> evaluation_data;

//    for (uint i = 0; i < 200; ++i)
//    {
//      double x = Random::gaussian(0, 1);
//      double y = Random::gaussian(3, 0.5);
//      evaluation_data.push_back(x);
//      evaluation_data.push_back(y);
//    }

//    KernelEstimator kernel_estimator;
//    double result = 0;
//    double increment = -5;
//    double integral = 0;
//    std::ofstream myfile;
//    myfile.open ("kernel_test1D.txt");
//    for (uint i = 0; i < 1000; ++i)
//    {
//      result = kernel_estimator.estimateKernelFunction1D(evaluation_data, "gaussian", 0.45, increment);
//      myfile << increment << " " << result << "\n";
//      integral += result * 0.01;
//      increment += 0.01;
//    }
//    myfile.close();

//    std::cout << "integral: " << integral << std::endl;
//    std::cout << "result: " << result << std::endl;
//  }


  //2D case
  {
//    std::vector<Eigen::VectorXd> evaluation_data2D;
/*
    std::vector<WeightedDataPoint> evaluation_data2D;

    Eigen::VectorXd first_mean = Eigen::Vector2d(0,0);
    Eigen::VectorXd second_mean = Eigen::Vector2d(2,3);
    Eigen::MatrixXd first_cov = Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd second_cov = Eigen::MatrixXd::Identity(2,2);

    for (uint i = 0; i < 100; ++i)
    {
      Eigen::VectorXd first_gaussian = Random::multivariateGaussian(first_cov, &first_mean);
      Eigen::VectorXd second_gaussian = Random::multivariateGaussian(second_cov, &second_mean);
      WeightedDataPoint dp1;
      dp1.data_point = first_gaussian;
      dp1.weight = 2;
      evaluation_data2D.push_back(dp1);
      dp1.data_point = second_gaussian;
      dp1.weight = 1;
      evaluation_data2D.push_back(dp1);
    }

    KernelEstimator kernel_estimator;
    double result = 0;
    double increment_y = -5;
    double integral = 0;
    std::ofstream myfile;
    myfile.open ("kernel_test2D.txt");
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2)* 0.25;


//    Eigen::MatrixXd H_Scotts = kernel_estimator.estimateH(evaluation_data2D);
    Eigen::MatrixXd H_Scotts = kernel_estimator.estimateWeightedH(evaluation_data2D);
    ROS_INFO_STREAM("Scott's formula for H: " << H_Scotts);


    for (uint i = 0; i < 100; ++i)
    {
      double increment_x = -5;
      for (uint j = 0; j < 100; ++j)
      {
        Eigen::VectorXd sample = Eigen::Vector2d(increment_x, increment_y);
        result = kernel_estimator.estimateWeightedKernelFunctionND(evaluation_data2D, "gaussian", H_Scotts, sample);
        myfile << increment_x << " " << increment_y << " " << result << "\n";
        myfile.flush();
//      integral += result * 0.01;
        increment_x += 0.1;
      }
      increment_y += 0.1;
    }
    myfile.close();

    std::cout << "integral: " << integral << std::endl;
    std::cout << "result: " << result << std::endl;
    double entropy = kernel_estimator.estimateWeightedEntropyKernelND(evaluation_data2D,"gaussian",H_Scotts);
    std::cout << "entropy: " << entropy << std::endl;*/
  }





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

  //TODO
  // cabinet
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
  Eigen::MatrixXd motionNoiseCov = covariance / 100000; //10000

  //orientation
//  motionNoiseCov(3,3) = motionNoiseCov(4,4) = motionNoiseCov(5,5) = 0.001;
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
      //TODO: wait for transform?
      // static marker, moving marker
//      tf_listener.lookupTransform("ar_marker_1", "/ar_marker_2", ros::Time(0), marker_static_to_marker);
//      tf_listener.lookupTransform("ar_marker_4", "/ar_marker_15", ros::Time(0), marker_static_to_marker);
      tf_listener.waitForTransform("ar_marker_9", "/ar_marker_12", ros::Time(0), ros::Duration(3.0));
      tf_listener.lookupTransform("ar_marker_9", "/ar_marker_12", ros::Time(0), marker_static_to_marker);

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

      // get tf_pose_proj in the current_pose frame
//      tf::Transform tf_pose_proj_orient = tf_pose_proj.inverseTimes(marker_static_to_marker);

      it->state->setParam("current_proj_pose_trans", tf_pose_proj.getOrigin(), articulation_model_msgs::ParamMsg::PRIOR);
      it->state->setParam("current_proj_pose_quat", tf_pose_proj.getRotation(), articulation_model_msgs::ParamMsg::PRIOR);

      if (it->state->model_msg.name == "prismatic")
      {
        tf::Vector3 prismatic_dir;
        boost::shared_ptr<PrismaticModel> prismatic_model = boost::dynamic_pointer_cast< PrismaticModel > (it->state);
        prismatic_dir.setX(prismatic_model->axis_x);
        prismatic_dir.setY(prismatic_model->axis_y);
        prismatic_dir.setZ(prismatic_model->axis_z);
        prismatic_dir = prismatic_dir * marker_static_to_marker.getBasis();
        it->state->setParam("current_proj_pose_prismatic_dir_in_current", prismatic_dir, articulation_model_msgs::ParamMsg::PRIOR);
      }


      if (it->state->model_msg.name == "rotational")
      {
        tf::Vector3 rot_proj_dir;
        boost::shared_ptr<RotationalModel> rotational_model = boost::dynamic_pointer_cast< RotationalModel > (it->state);
        tf::Matrix3x3 rot_axis_m(rotational_model->rot_axis);
        tf::Vector3 rot_axis_z = rot_axis_m.getColumn(2);
        tf::Vector3 radius = tf_pose_proj.getOrigin() - rotational_model->rot_center;
        rot_proj_dir = radius.cross(rot_axis_z);

        rot_proj_dir = rot_proj_dir * marker_static_to_marker.getBasis();

        it->state->setParam("current_proj_pose_rot_dir", rot_proj_dir, articulation_model_msgs::ParamMsg::PRIOR);
      }
    }

// ----------------- propagate particles -------------------------------------------
    ROS_INFO_STREAM ("loop_count: " << loop_count);

//    if (loop_count <= 40)
//    {
      pf.propagate(pf.particles, u, motionNoiseCov, *motionModel);
//    }

    Visualizer::getInstance()->publishParticlesOnly(pf.particles);
    ros::spinOnce();

    if (loop_count % 10 == 0 || loop_count == 1)
    {



// ----------------- adding particles -------------------
      if(loop_count == 20)
      {
        ROS_INFO_STREAM ("adding particles");
        pf.addParticles(model_msg.track, 30, 50, 30, 0);
//        pf.addParticles(model_msg.track, 30, 50, 1, 0);
      }





// ----------------- marker correction step -------------------
//      if (loop_count <= 40)
//      {
        ROS_INFO ("executing correction step");
        articulation_model_msgs::TrackMsg z;
        pf.correct<articulation_model_msgs::TrackMsg>(pf.particles, z, sensorNoiseCov, *sensorModel);
        ROS_INFO ("Correction step executed.");
//      }










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

//      ROS_INFO("Entropy old: %f", pf.calculateEntropy(temp_particles));
      ROS_INFO("Entropy NEW: %f \n\n", pf.calculateKDEEntropy(temp_particles));

//      action_gen.publishGenActions();

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
        double expected_entropy = pf.calculateExpectedKDEEntropy<int, ActionPtr>(temp_particles, za_expected, action, sensorNoiseCov, *sensorActionModel);
        ROS_ERROR("Expected Entropy: %f \n", expected_entropy);
//        double expected_downweight = pf.calculateExpectedDownweightAfterAction<int, ActionPtr>(temp_particles, za_expected, action, sensorNoiseCov, *sensorActionModel);
//        ROS_ERROR("Expected Downweight: %f \n \n", expected_downweight);

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
      geometry_msgs::Pose poseMsg;
      tf::poseTFToMsg(marker_static_to_marker, poseMsg);
      action->plan(best_action, poseMsg);
      ROS_INFO("BEST Action: x = %f, y = %f, z = %f", best_action.getX(), best_action.getY(), best_action.getZ());






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
//        bool both_ways = false;
//        bool success = action->execute(best_action, "ar_marker_2", both_ways);
//        std::cerr << "Action successful? " << success << std::endl;
//        // 1 - doesnt stop
//        int z_action = action->getActionResult();
        int z_action = 0;

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
        double weights_rigid, weights_prismatic, weights_rotational, weights_free;
        pf.splitArticulationModels(pf.particles, weights_rigid, weights_prismatic, weights_rotational, weights_free);
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
