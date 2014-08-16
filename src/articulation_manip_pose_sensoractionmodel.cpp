#include "particle_filter/articulation_manip_pose_sensoractionmodel.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rotational_model.h"
#include "particle_filter/random.h"


template <class StateType, class ZType, class AType> ArtManipPoseSensorActionModel<StateType, ZType, AType>::ArtManipPoseSensorActionModel()
{
  distance_threshold = 0.07;
  angle_threshold = 20 * M_PI/180;
}

template <class StateType, class ZType, class AType> ArtManipPoseSensorActionModel<StateType, ZType, AType>::~ArtManipPoseSensorActionModel()
{
}


template <class StateType, class ZType, class AType> ZType ArtManipPoseSensorActionModel<StateType, ZType, AType>::sense(const StateType &state, const Eigen::VectorXd &noise) const
{
}

template <class StateType, class ZType, class AType> double ArtManipPoseSensorActionModel<StateType, ZType, AType>::senseLikelihood(const ZType &z, const AType& a,
                                                                                                                                const StateType &state,
                                                                                                                                const Eigen::MatrixXd &cov) const
{
}

template <class StateType, class ZType, class AType> double ArtManipPoseSensorActionModel<StateType, ZType, AType>::senseLogLikelihood(const ZType &z, const AType& a,
                                                                                                                                const StateType &state,
                                                                                                                                const Eigen::MatrixXd &cov) const
{
}


template <> double ArtManipPoseSensorActionModel<ArticulationModelPtr, tf::Transform, ActionPtr>::senseLogLikelihood(const tf::Transform &z, const ActionPtr& a,
                                                                                                                                const ArticulationModelPtr &state,
                                                                                                                                const Eigen::MatrixXd &cov) const
{
  //add a cylinder around the action for the controller
  //find an intersection point between the cylinder and the model
  //project measurement into configuration space
  // find distance between predicted configuration space point(previously computed intersection) and the z projected on the configuration space(wide gaussian)
  // find distance between the z projected and the z (narrow gaussian)

/*  double loglikelihood = 0;
  uint points_to_generate = 2000;

  switch (state->model)
  {
    case (RIGID):
      {
        geometry_msgs::Pose *z_pose = new geometry_msgs::Pose;
        tf::poseTFToMsg(z, *z_pose);
        loglikelihood = state->getInlierLogLikelihood(0, z_pose);
        delete z_pose;
      }
    case (PRISMATIC):
      {
        if (a->action_type == PRISMATIC_ACTION)
        {
          boost::shared_ptr<ActionPrismatic> action_prismatic = boost::dynamic_pointer_cast< ActionPrismatic > (a);
          boost::shared_ptr<PrismaticModel> prismatic_model = boost::dynamic_pointer_cast< PrismaticModel > (state);
          //generate points on the model line and calculate the distance to the action line
          // calculate the angle between these two
          // have two thresholds for the angle and the distance; if anything exceeds then that's the point
          double prism_axis_x = prismatic_model->axis_x;
          double prism_axis_y = prismatic_model->axis_y;
          double prism_axis_z = prismatic_model->axis_z;
          tf::Vector3 pose_model_generated;
          tf::Vector3 prismatic_action_x1(action_prismatic->axis_x, action_prismatic->axis_y, action_prismatic->axis_z);
          tf::Vector3 prismatic_action_x2(action_prismatic->axis_x*2, action_prismatic->axis_y*2, action_prismatic->axis_z*2);


          for (uint i = 0; i < points_to_generate; i++)
          {
            pose_model_generated.setX(prism_axis_x * (static_cast<double> (i)/10.0));
            pose_model_generated.setY(prism_axis_y * (static_cast<double> (i)/10.0));
            pose_model_generated.setZ(prism_axis_z * (static_cast<double> (i)/10.0));

            //from here: http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
            double distance = ( (pose_model_generated - prismatic_action_x1).cross((pose_model_generated - prismatic_action_x2)) ).length()  / ( (prismatic_action_x2 - prismatic_action_x1).length() );
            double angle = pose_model_generated.angle(prismatic_action_x1);
            if (distance > distance_threshold || angle > angle_threshold)
            {
              break;
            }
            //TODO what happens if it doesnt stop
          }
          //wide gaussian
          geometry_msgs::Pose configuration_pose_expected;
          tf::poseTFToMsg(tf::Transform(tf::Quaternion(0,0,0,1), pose_model_generated), configuration_pose_expected);
          Eigen::VectorXd q_expected = state->predictConfiguration(configuration_pose_expected);
          //compute q measured and get the likelihood
          geometry_msgs::Pose configuration_pose_measured;
          tf::poseTFToMsg(z, configuration_pose_measured);
          Eigen::VectorXd q_measured = state->predictConfiguration(configuration_pose_measured);
          loglikelihood = Random::gaussianDensity(0, sqrt(cov(0,0)), fabs(q_measured(0) - q_expected(0)));

          //narrow gaussian
          geometry_msgs::Pose *z_pose = new geometry_msgs::Pose;
          tf::poseTFToMsg(z, *z_pose);
          // TODO: this is between the closest point on the model, how about to comparing to predictPose(q_expected)?
          // TODO: combine likelihoods, just add?
          loglikelihood += state->getInlierLogLikelihood(0, z_pose);
          delete z_pose;
        }
        else if (a->action_type == ROTATIONAL_ACTION)
        {

        }

      }
    case (ROTATIONAL):
      {
        if (a->action_type == PRISMATIC_ACTION)
        {

        }
        else if (a->action_type == ROTATIONAL_ACTION)
        {

        }
      }
  }


  return loglikelihood;*/
}

template class ArtManipPoseSensorActionModel<ArticulationModelPtr, tf::Transform, ActionPtr>;
