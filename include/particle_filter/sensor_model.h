#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include <Eigen/Core>

class SensorModel
{
public:
  SensorModel();
  virtual ~SensorModel();

  virtual Eigen::VectorXd sense (const Eigen::VectorXd& state, const Eigen::VectorXd& noise)
                                  const = 0;

  virtual double senseLikelihood (const Eigen::VectorXd& z, const Eigen::VectorXd& state,
                                  const Eigen::MatrixXd& cov) const = 0;

  virtual double senseLogLikelihood (const Eigen::VectorXd& z, const Eigen::VectorXd& state,
                                  const Eigen::MatrixXd& cov) const;





};

#endif //SENSOR_MODEL_H
