#include "random.h"
#include <Eigen/Cholesky>
#include <unistd.h>

namespace ranav {

void Random::randomize() {
  srand(time(NULL) + getpid()); // TODO: use better random number generator such as gsl_rng
}

double Random::uniform() {
  return rand()/(double)RAND_MAX;
}

double Random::uniform(double a, double b) {
  assert(b != a);
  return a + (b-a)*uniform();
}

//! returns random sample of the gaussian N(0, 1) distribution (using
//! the polar method)
double Random::gaussian() {
  static int iset = 0;
  static double gset;
  double fac, rsq, v1, v2;
  if(iset == 0) {
    do {
      v1 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;
      v2 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;
      rsq = v1*v1 + v2*v2;
    } while(rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0*log(rsq)/rsq);
    gset = v1*fac;
    iset = 1;
    return v2*fac;
  }
  else {
    iset = 0;
    return gset;
  }
}

double Random::gaussian(double mean, double stddev) {
  return gaussian()*stddev + mean;
}

//! See: Russell Cheng, Random Variate Generation, Handbook of Simulation,
//! editor Jerry Banks, Wiley, 1998, pages 167-168.
Eigen::VectorXd Random::multivariateGaussian(const Eigen::MatrixXd &covariance, const Eigen::MatrixXd *choleskyL){
  int size = covariance.rows();
  Eigen::VectorXd sample(size);
  for (int i=0; i<size; ++i)
    sample(i) = gaussian();

  if (choleskyL) {
    return (*choleskyL)*sample;
  }
  Eigen::MatrixXd mL = covariance.llt().matrixL(); // Cholesky decomposition
  return mL*sample;
}

} /* namespace ranav */
