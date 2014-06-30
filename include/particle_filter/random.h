#ifndef RANDOM_H_
#define RANDOM_H_

#include <Eigen/Core>

namespace ranav {

class Random {
public:

  //! initializes random seed
  static void randomize();

  //! returns a sample of the uniform distribution in [0, 1]
  static double uniform();

  //! returns a sample of the uniform distribution in [a, b]
  static double uniform(double a, double b);

  //! returns a sample of the Gaussian distribution N(0, 1)
  static double gaussian();

  //! returns random sample of the Gaussian distribution N(mu, stddev^2) that has the covariance stddev^2
  static double gaussian(double mean, double stddev);

  //! cholesky (optional) is the Cholesky decomposition of the covariance
  static Eigen::VectorXd multivariateGaussian(const Eigen::MatrixXd &covariance, const Eigen::MatrixXd *choleskyL = NULL);
};

} /* namespace ranav */

#endif /* RANDOM_H_ */
