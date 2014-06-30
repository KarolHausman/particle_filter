#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include <Eigen/Core>

class SensorModel
{
public:
  SensorModel();
  virtual ~SensorModel();

  virtual Eigen::VectorXd sense (const Eigen::VectorXd& state, const Eigen::VectorXd& noise) const = 0;








};

#endif //SENSOR_MODEL_H
