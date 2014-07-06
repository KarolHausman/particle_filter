#include "particle_filter/sensor_model.h"

SensorModel::SensorModel()
{
}

SensorModel::~SensorModel()
{
}

double SensorModel::senseLogLikelihood(const Eigen::VectorXd &z, const Eigen::VectorXd &state,
                                       const Eigen::MatrixXd &cov) const
{
  return log(senseLikelihood(z, state, cov));
}
