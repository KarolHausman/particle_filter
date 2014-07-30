#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include <Eigen/Core>
#include <particle_filter/articulation_model.h>

template <class StateType, class ZType>
class SensorModel
{
public:
  SensorModel();
  virtual ~SensorModel();

  virtual ZType sense (const StateType& state, const Eigen::VectorXd& noise)
                                  const = 0;

  virtual double senseLikelihood (const ZType& z, const StateType& state,
                                  const Eigen::MatrixXd& cov) const = 0;

  virtual double senseLogLikelihood (const ZType& z, const StateType& state,
                                  const Eigen::MatrixXd& cov) const;





};

#endif //SENSOR_MODEL_H
