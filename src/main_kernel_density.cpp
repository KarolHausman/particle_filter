
#include "particle_filter/random.h"
#include "particle_filter/kernel_density_estimator.h"


int main(int argc, char **argv)
{


  // 1D case
  {
    std::vector<double> evaluation_data;

    for (uint i = 0; i < 200; ++i)
    {
      double x = Random::gaussian(0, 1);
      double y = Random::gaussian(3, 0.5);
      evaluation_data.push_back(x);
      evaluation_data.push_back(y);
    }

    KernelEstimator kernel_estimator;
    double result = 0;
    double increment = -5;
    double integral = 0;
    std::ofstream myfile;
    myfile.open ("kernel_test1D.txt");
    for (uint i = 0; i < 1000; ++i)
    {
      result = kernel_estimator.estimateKernelFunction1D(evaluation_data, "gaussian", 0.45, increment);
      myfile << increment << " " << result << "\n";
      integral += result * 0.01;
      increment += 0.01;
    }
    myfile.close();

    std::cout << "integral: " << integral << std::endl;
    std::cout << "result: " << result << std::endl;
  }


  //2D case
  {
//    std::vector<Eigen::VectorXd> evaluation_data2D;

    std::vector<WeightedDataPoint> evaluation_data2D;

    Eigen::VectorXd first_mean = Eigen::Vector2d(0,0);
    Eigen::VectorXd second_mean = Eigen::Vector2d(2,3);
    Eigen::MatrixXd first_cov = Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd second_cov = Eigen::MatrixXd::Identity(2,2);

    for (uint i = 0; i < 100; ++i)
    {
      Eigen::VectorXd first_gaussian = Random::multivariateGaussian(first_cov, &first_mean);
      Eigen::VectorXd second_gaussian = Random::multivariateGaussian(second_cov, &second_mean);
      WeightedDataPoint dp1;
      dp1.data_point = first_gaussian;
      dp1.weight = 2;
      evaluation_data2D.push_back(dp1);
      dp1.data_point = second_gaussian;
      dp1.weight = 1;
      evaluation_data2D.push_back(dp1);
    }

    KernelEstimator kernel_estimator;
    double result = 0;
    double result_09 = 0;
    double result_01 = 0;
    double result_05 = 0;

    double increment_y = -5;
    double integral = 0;
    std::ofstream myfile, my_file0_9, my_file0_1, my_file0_5;
    myfile.open ("kernel_test2D.txt");
    my_file0_9.open("kernel_2D_09.txt");
    my_file0_1.open("kernel_2D_01.txt");
    my_file0_5.open("kernel_2D_05.txt");

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2)* 0.25;


//    Eigen::MatrixXd H_Scotts = kernel_estimator.estimateH(evaluation_data2D);
    Eigen::MatrixXd H_Scotts;
    kernel_estimator.estimateWeightedH(evaluation_data2D, H_Scotts);
    ROS_INFO_STREAM("Scott's formula for H: " << H_Scotts);


    for (uint i = 0; i < 100; ++i)
    {
      double increment_x = -5;
      for (uint j = 0; j < 100; ++j)
      {
        Eigen::VectorXd sample = Eigen::Vector2d(increment_x, increment_y);
        result = kernel_estimator.estimateWeightedKernelFunctionND(evaluation_data2D, "gaussian", H_Scotts, sample);
        myfile << increment_x << " " << increment_y << " " << result << "\n";

        result_09 = kernel_estimator.estimateWeightedKernelFunctionND(evaluation_data2D, "gaussian", H_Scotts, sample, 0.9);
        my_file0_9 << increment_x << " " << increment_y << " " << result_09 << "\n";

        result_01 = kernel_estimator.estimateWeightedKernelFunctionND(evaluation_data2D, "gaussian", H_Scotts, sample, 0.1);
        my_file0_1 << increment_x << " " << increment_y << " " << result_01 << "\n";

        result_05 = kernel_estimator.estimateWeightedKernelFunctionND(evaluation_data2D, "gaussian", H_Scotts, sample, 0.5);
        my_file0_5 << increment_x << " " << increment_y << " " << result_05 << "\n";


        myfile.flush();
        my_file0_9.flush();
        my_file0_1.flush();
        my_file0_5.flush();

//      integral += result * 0.01;
        increment_x += 0.1;
      }
      increment_y += 0.1;
    }
    myfile.close();
    my_file0_9.close();
    my_file0_1.close();

    std::cout << "integral: " << integral << std::endl;
    std::cout << "result: " << result << std::endl;
    double entropy = kernel_estimator.estimateWeightedEntropyKernelND(evaluation_data2D,"gaussian",H_Scotts);

    double entropy_09 = kernel_estimator.estimateWeightedEntropyKernelND(evaluation_data2D,"gaussian",H_Scotts, 0.9);

    double entropy_01 = kernel_estimator.estimateWeightedEntropyKernelND(evaluation_data2D,"gaussian",H_Scotts, 0.1);

    double entropy_05 = kernel_estimator.estimateWeightedEntropyKernelND(evaluation_data2D,"gaussian",H_Scotts, 0.5);


    std::cout << "entropy 1 : " << entropy << std::endl;
    std::cout << "entropy 0.9 : " << entropy_09 << std::endl;
    std::cout << "entropy 0.1 : " << entropy_01 << std::endl;
    std::cout << "entropy 0.9 + 0.1 : " << entropy_09 + entropy_01 << std::endl;

    std::cout << "entropy 0.5 : " << entropy_05 << std::endl;
    std::cout << "entropy 0.5*2 : " << entropy_05*2 << std::endl;

  }
