#include "particle_filter/random.h"
#include <Eigen/Cholesky>
#include <unistd.h>
#include <random>


void Random::randomize() {
  srand(time(NULL) + getpid());
}

double Random::uniform() {
  return rand()/(double)RAND_MAX;
}

double Random::uniform(double a, double b) {
  assert(b != a);
  return a + (b-a)*uniform();
}

//! returns random sample of the gaussian N(0, 1) distribution
double Random::gaussian()
{
  std::default_random_engine generator;
  std::normal_distribution <double> distribution(0.0,1.0);

  return distribution(generator);
}

double Random::gaussian(double mean, double stddev) {
  return gaussian()*stddev + mean;
}


Eigen::VectorXd Random::multivariateGaussian(const Eigen::MatrixXd &covariance){
  int size = covariance.rows();
  Eigen::VectorXd sample(size);
  for (int i=0; i<size; ++i)
    sample(i) = gaussian();

  Eigen::MatrixXd mL = covariance.llt().matrixL(); // Cholesky decomposition
  return mL*sample;
}

