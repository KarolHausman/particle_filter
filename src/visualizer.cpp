#include "particle_filter/visualizer.h"
#include <visualization_msgs/Marker.h>


Visualizer* Visualizer::instance_ = NULL;

Visualizer* Visualizer::getInstance()
{
  if (instance_ == NULL)
  {
    instance_ = new Visualizer;
  }
  return instance_;
}

void Visualizer::init()
{
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualizion_marker", 10);
  model_pub_ = nh_.advertise<articulation_model_msgs::ModelMsg>("model_track", 10);
  particles_pub_ = nh_.advertise<articulation_model_msgs::ParticlesMsg>("model_particles", 10);

}

void Visualizer::publishParticles(const std::vector <Particle <Eigen::VectorXd> > &particles)
{
  std::vector <Eigen::VectorXd> points;
  points.reserve(particles.size());

  for (std::vector <Particle <Eigen::VectorXd> >::const_iterator it = particles.begin(); it != particles.end();
      ++it)
  {
    points.push_back(it->state);
  }
  publishPoints(points, 0.1);
}


void Visualizer::publishParticlesOnly(const std::vector <Particle <ArticulationModelPtr> > &particles)
{
  articulation_model_msgs::ParticlesMsg particles_msg;
  particles_msg.header = particles.back().state->getModel().header;
  for (std::vector <Particle <ArticulationModelPtr> >::const_reverse_iterator it = particles.rbegin(); it != particles.rend();
      ++it)
  {
    it->state->setParam("weight",it->weight,articulation_model_msgs::ParamMsg::EVAL);
    particles_msg.particles.push_back(it->state->getModel());
  }
  ROS_INFO("PUBLISHING PARTICLES ONLY");
  particles_pub_.publish(particles_msg);
}


//assumes that particles are sorted
//publishes the best particle of each kind(if the kind exists)
void Visualizer::publishParticles(const std::vector <Particle <ArticulationModelPtr> > &particles)
{
  uint rigid_counter = 0;
  uint rotational_counter = 0;
  uint prismatic_counter = 0;
  uint free_counter = 0;


  articulation_model_msgs::ParticlesMsg particles_msg;
  particles_msg.header = particles.back().state->getModel().header;

  for (std::vector <Particle <ArticulationModelPtr> >::const_reverse_iterator it = particles.rbegin(); it != particles.rend();
      ++it)
  {
    it->state->model_msg.header.stamp = ros::Time::now();
    it->state->model_msg.header.frame_id = "/world";
    switch(it->state->model)
    {
      case (RIGID):
      {
        ++rigid_counter;
        if(rigid_counter == 1)
        {
          it->state->setParam("weight",it->weight_to_print_only,articulation_model_msgs::ParamMsg::EVAL);
          model_pub_.publish(it->state->getModel());
        }
        else
        {
          it->state->setParam("weight",it->weight_to_print_only,articulation_model_msgs::ParamMsg::EVAL);
          particles_msg.particles.push_back(it->state->getModel());
        }
        break;
      }
      case (ROTATIONAL):
      {
        ++rotational_counter;
        if(rotational_counter == 1)
        {
          it->state->setParam("weight",it->weight_to_print_only,articulation_model_msgs::ParamMsg::EVAL);
          model_pub_.publish(it->state->getModel());
        }
        else
        {
          it->state->setParam("weight",it->weight_to_print_only,articulation_model_msgs::ParamMsg::EVAL);
          particles_msg.particles.push_back(it->state->getModel());
        }
        break;
      }
      case (PRISMATIC):
      {
        ++prismatic_counter;
        if(prismatic_counter == 1)
        {
          it->state->setParam("weight",it->weight_to_print_only,articulation_model_msgs::ParamMsg::EVAL);
          model_pub_.publish(it->state->getModel());
        }
        else
        {
          it->state->setParam("weight",it->weight_to_print_only,articulation_model_msgs::ParamMsg::EVAL);
          particles_msg.particles.push_back(it->state->getModel());
        }
        break;
      }
      case (FREE):
      {
        ++free_counter;
        if(free_counter == 1)
        {
          it->state->setParam("weight",it->weight_to_print_only,articulation_model_msgs::ParamMsg::EVAL);
          model_pub_.publish(it->state->getModel());
        }
        else
        {
          it->state->setParam("weight",it->weight_to_print_only,articulation_model_msgs::ParamMsg::EVAL);
          particles_msg.particles.push_back(it->state->getModel());
        }
        break;
      }
    }
  }

  particles_pub_.publish(particles_msg);

  double sum_all = rigid_counter + prismatic_counter + rotational_counter + free_counter;
  double rigid_percentage = rigid_counter/sum_all * 100;
  double prismatic_percentage = prismatic_counter/sum_all * 100;
  double rotational_percentage = rotational_counter/sum_all * 100;
  double free_percentage = free_counter/sum_all * 100;

  ROS_ERROR_STREAM("STATISTICS: \n" << "RIGID: " << rigid_percentage << "\n PRISMATIC: " << prismatic_percentage << "\n ROTATIONAL: " << rotational_percentage <<  "\n FREE: " << free_percentage << "\n");
}


void Visualizer::publishPoints(const std::vector <Eigen::VectorXd> &points, const double& scale)
{
  visualization_msgs::Marker marker_points;
  marker_points.header.frame_id = "/world";
  marker_points.header.stamp = ros::Time::now();
  marker_points.ns = "points";
  marker_points.action = visualization_msgs::Marker::ADD;
  marker_points.pose.orientation.w = 1.0;

  marker_points.id = 0;
  marker_points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  marker_points.scale.x = scale;
  marker_points.scale.y = scale;

  // Points are green
  marker_points.color.g = 1.0f;
  marker_points.color.a = 1.0;


  for (uint i = 0; i < points.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = points[i](0);
    p.y = points[i](1);
    p.z = points[i](2);

    marker_points.points.push_back(p);
  }
  marker_pub_.publish(marker_points);
}
