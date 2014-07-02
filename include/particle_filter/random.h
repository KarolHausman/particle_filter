#ifndef RANDOM_H_
#define RANDOM_H_

#include <Eigen/Core>


namespace Random {

  //! initializes random seed
  void randomize();

  //! returns a sample of the uniform distribution in [0, 1]
  double uniform();

  //! returns a sample of the uniform distribution in [a, b]
  double uniform(double a, double b);

  //! returns a sample of the Gaussian distribution N(0, 1)
  double gaussian();

  double gaussian(double mean, double stddev);

  Eigen::VectorXd multivariateGaussian(const Eigen::MatrixXd &covariance);

  double gaussianProbility(double mean, double stddev, double z);

  double multivariateGaussianProbability(const Eigen::VectorXd& mean,
                                         const Eigen::MatrixXd& cov,
                                         const Eigen::VectorXd& z);

}


#endif /* RANDOM_H_ */
