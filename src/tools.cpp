#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  std::cout << "Tools::CalculateRMSE" << std::endl;
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size

  if(estimations.size() < 0)
  {
      std::cout<<"estimation.rows() < 0";
  }
  
  if(estimations.size() != ground_truth.size())
  {
      std::cout<<"estimations.size() != ground_truth.size()";
  }

  // TODO: accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd residual;
    residual = estimations[i] - ground_truth[i];
    residual = residual.array().square();
    rmse = rmse + residual;
  }

  // TODO: calculate the mean
  rmse = rmse / estimations.size();

  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  std::cout<< "Tools::CalculateJacobian" << std::endl;
   MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  if( px==0 && py==0 )
  {
      std::cout<<"Division by 0";
      return Hj;
  }
  
  // compute the Jacobian matrix
  float pxpy_2 = px*px + py*py;
  float sqrt_pxpy_2 = sqrt(pxpy_2);
  
  Hj(0,0) = px/sqrt_pxpy_2;
  Hj(0,1) = py/sqrt_pxpy_2;
  Hj(0,2) = 0.0;
  Hj(0,3) = 0.0;
  
  Hj(1,0) = -py/pxpy_2;
  Hj(1,1) = px/pxpy_2;
  Hj(1,2) = 0.0;
  Hj(1,3) = 0.0;

  Hj(2,0) = py*(vx*py-vy*px)/ pow(pxpy_2, 3.0/2.0);
  Hj(2,1) = px*(vy*px-vx*py)/ pow(pxpy_2, 3.0/2.0);
  Hj(2,2) = px/sqrt_pxpy_2;
  Hj(2,3) = py/sqrt_pxpy_2;

  std::cout<< "End Tools::CalculateJacobian" << std::endl;
  return Hj;
}
