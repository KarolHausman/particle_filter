#include "particle_filter/random.h"
#include <Eigen/Cholesky>
#include <unistd.h>
#include <random>
#include <iostream>
#include <Eigen/LU>

namespace Random
{

  void randomize() {
    srand(time(NULL) + getpid());
  }

  double uniform() {
    return rand()/(double)RAND_MAX;
  }

  double uniform(double a, double b) {
    assert(b != a);
    return a + (b-a)*uniform();
  }

  //! returns random sample of the gaussian N(0, 1) distribution
  double gaussian()
  {
    std::random_device rd;
    std::default_random_engine generator( rd() );
    std::normal_distribution <double> distribution(0.0,1.0);

    return distribution(generator);
  }

  double gaussian(double mean, double stddev) {
    return gaussian()*stddev + mean;
  } 

  Eigen::VectorXd multivariateGaussian(const Eigen::MatrixXd &covariance, Eigen::VectorXd *mean){
    int size = covariance.rows();
    Eigen::VectorXd sample(size);
    for (int i=0; i<size; i++)
    {
      sample(i) = gaussian();
    }

    Eigen::MatrixXd mL = covariance.llt().matrixL(); // Cholesky decomposition
    if (mean != NULL)
      return mL*sample + *mean;
    else
      return mL*sample;
  }

  double gaussianDensity(double mean, double stddev, double z)
  {
    const double var = stddev* stddev;
    return exp(-1.0 * ((z - mean) * (z - mean) / (2 * var)))/ sqrt(2 * M_PI * var);
  }


  double multivariateGaussianDensity(const Eigen::VectorXd& mean,
                                         const Eigen::MatrixXd& cov,
                                         const Eigen::VectorXd& z)
  {
    Eigen::VectorXd diff = mean - z;

    Eigen::VectorXd exponent = -0.5 * (diff.transpose() * cov.inverse() * diff);

    return pow(2 * M_PI, (double) z.size() / -2.0) * pow(cov.determinant(), -0.5) *
    exp(exponent(0));

  }

}
