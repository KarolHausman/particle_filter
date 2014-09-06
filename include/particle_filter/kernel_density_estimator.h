#ifndef KERNEL_ESTIMATOR_H
#define KERNEL_ESTIMATOR_H

#include "particle_filter/particle.h"
#include "particle_filter/random.h"
#include <unsupported/Eigen/MatrixFunctions>

struct WeightedDataPoint
{
  Eigen::VectorXd data_point;
  double weight;
};

class KernelEstimator
{
public:
  KernelEstimator(){}
  virtual ~KernelEstimator(){}

  double estimateKernelFunction1D(const std::vector<double>& data_points, const std::string& kernel_name, const double bandwidth, const double x)
  {
    size_t n = data_points.size();
    double h = 1/bandwidth;
    double result = 0;

    for (uint i = 0; i < n; ++i)
    {
      if (kernel_name == "gaussian")
      {
        result += gaussianKernel( h * (x - data_points[i]) );
      }
    }
    return result * h / n ;
  }

  double estimateWeightedEntropyKernelND(const std::vector<WeightedDataPoint>& data_points, const std::string& kernel_name, const Eigen::MatrixXd& H, const double density_weight = 1)
  {
    size_t n = data_points.size();
    double result = 0;
    for (uint i = 0; i < n; ++i)
    {
      result += log2(estimateWeightedKernelFunctionND(data_points, kernel_name, H, data_points[i].data_point, density_weight));
    }
    return result * (-1.0/(double)n);
  }


  double estimateEntropyKernelND(const std::vector<Eigen::VectorXd>& data_points, const std::string& kernel_name, const Eigen::MatrixXd& H)
  {
    size_t n = data_points.size();
    double result = 0;
    for (uint i = 0; i < n; ++i)
    {
      result += log2(estimateKernelFunctionND(data_points, kernel_name, H, data_points[i]));
    }
    return result * (-1.0/(double)n);
  }

  //assumes that H is diagonal
  double estimateWeightedKernelFunctionND(const std::vector<WeightedDataPoint>& data_points, const std::string& kernel_name, const Eigen::MatrixXd& H, const Eigen::VectorXd& x, const double density_weight = 1)
  {
    size_t n = data_points.size();
    double result = 0;
    double weights_sum = 0;
    double det_H = H.determinant();
    double det_H_pow = pow(det_H, -1/2);
    Eigen::MatrixXd H_pow = H.inverse();
    H_pow.diagonal() = H_pow.diagonal().array().sqrt();


    for (uint i = 0; i < n; ++i)
    {
      if (kernel_name == "gaussian")
      {
        result += det_H_pow * data_points[i].weight * gaussianKernel( H_pow * (x - data_points[i].data_point) );
        weights_sum += data_points[i].weight;
      }
    }
    return density_weight * result/weights_sum;
  }

  //assumes that H is diagonal
  double estimateKernelFunctionND(const std::vector<Eigen::VectorXd>& data_points, const std::string& kernel_name, const Eigen::MatrixXd& H, const Eigen::VectorXd& x)
  {
    size_t n = data_points.size();
    double result = 0;
    double det_H = H.determinant();
    double det_H_pow = pow(det_H, -1/2);
    Eigen::MatrixXd H_pow = H.inverse();
    H_pow.diagonal() = H_pow.diagonal().array().sqrt();


    for (uint i = 0; i < n; ++i)
    {
      if (kernel_name == "gaussian")
      {
        result += det_H_pow * gaussianKernel( H_pow * (x - data_points[i]) );
      }
    }
    return result/n;
  }

  Eigen::MatrixXd estimateH(std::vector<Eigen::VectorXd>& data_points)
  {
    uint d = data_points.front().size();
    std::vector< std::vector <double> > sigmas;
    Eigen::MatrixXd result_H = Eigen::MatrixXd::Identity(d,d);
    for (uint i=0; i<d; ++i)
    {
      std::vector<double> sigmas_i;
      for (std::vector<Eigen::VectorXd>::iterator it=data_points.begin(); it!= data_points.end(); ++it)
      {
        sigmas_i.push_back((*it)(i));
      }
      sigmas.push_back(sigmas_i);
    }
    for (uint i=0; i<sigmas.size(); ++i)
    {
      double scotts_i = pow( computeStdDev(sigmas[i], computeAverage(sigmas[i])) * pow( (double)data_points.size(), -1/(double)(d + 4) ) , 2);
      result_H(i,i) = scotts_i;
    }
    return result_H;
  }

  bool estimateWeightedH(std::vector<WeightedDataPoint>& data_points, Eigen::MatrixXd& result)
  {
    uint d = data_points.front().data_point.size();
    double weights_sum = 0;
    std::vector< std::vector <std::pair<double, double> > > sigmas;
    Eigen::MatrixXd result_H = Eigen::MatrixXd::Identity(d,d);
    for (uint i=0; i<d; ++i)
    {
      std::vector<std::pair<double, double> > sigmas_i;
      for (std::vector<WeightedDataPoint>::iterator it=data_points.begin(); it!= data_points.end(); ++it)
      {
        sigmas_i.push_back(std::make_pair((it->data_point)(i), it->weight));
        weights_sum += it->weight;
      }
      sigmas.push_back(sigmas_i);
    }

    if (weights_sum < 0.0000000001)
    {
      return false;
    }

    for (uint i=0; i<sigmas.size(); ++i)
    {
      double scotts_i = pow( computeWeightedStdDev(sigmas[i], computeWeightedAverage(sigmas[i])) * pow( (double)data_points.size(), -1/(double)(d + 4) ) , 2);
      result_H(i,i) = scotts_i;
    }
    result = result_H;
    return true;
  }

private:

  double computeWeightedAverage(std::vector<std::pair<double, double> > v)
  {
    double sum = 0;
    double sum_weights = 0;
    for(int i = 0; i < v.size(); i++)
    {
      sum += v[i].first*v[i].second;
      sum_weights += v[i].second;
    }
    return sum / sum_weights;
  }

  double computeAverage(std::vector<double> v)
  {
    double sum = 0;
    for(int i = 0; i < v.size(); i++)
     sum += v[i];
    return sum / (double)v.size();
  }

  double computeStdDev(std::vector<double> v, double ave)
  {
    double E = 0;
    for(int i = 0; i < v.size(); i++)
      E += pow(v[i] - ave, 2);
    return sqrt(1 / (double)v.size() * E);
  }

  double computeWeightedStdDev(std::vector<std::pair<double, double> > v, double ave)
  {
    double E = 0;
    double sum_weights = 0;
    for(int i = 0; i < v.size(); i++)
    {
      E += v[i].second * pow(v[i].first - ave, 2);
      sum_weights += v[i].second;
    }
    return sqrt(1 / sum_weights * E);
  }

  double gaussianKernel(const double x)
  {
    return Random::gaussianDensity(0, 1, x);
  }

  double gaussianKernel(const Eigen::VectorXd& x)
  {
    Eigen::VectorXd exponent = -0.5 * (x.transpose() * x);
    return pow(2 * M_PI, (double) x.size() / -2.0) * exp(exponent(0));
//    Eigen::VectorXd mean = Eigen::VectorXd::Zero(x.rows());
//    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(x.rows(), x.rows());
//    return Random::multivariateGaussianDensity(mean, cov, x);
  }


};


#endif //KERNEL_ESTIMATOR_H
