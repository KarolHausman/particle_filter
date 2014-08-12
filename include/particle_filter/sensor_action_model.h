#ifndef SENSOR_ACTION_MODEL_H
#define SENSOR_ACTION_MODEL_H

#include <Eigen/Core>
#include <particle_filter/articulation_model.h>
#include <particle_filter/action.h>

template <class StateType, class ZType, class AType>
class SensorActionModel
{
public:
  SensorActionModel();
  virtual ~SensorActionModel();

  virtual ZType sense (const StateType& state, const Eigen::VectorXd& noise)
                                  const = 0;

  virtual double senseLikelihood (const ZType& z, const AType& a, const StateType& state,
                                  const Eigen::MatrixXd& cov) const = 0;

  virtual double senseLogLikelihood (const ZType& z, const AType& a, const StateType& state,
                                  const Eigen::MatrixXd& cov) const;


};

#endif //SENSOR_ACTION_MODEL_H
