#ifndef RANDOM_H_
#define RANDOM_H_

#include <Eigen/Core>


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

  static double gaussian(double mean, double stddev);

  static Eigen::VectorXd multivariateGaussian(const Eigen::MatrixXd &covariance);
};


#endif /* RANDOM_H_ */
