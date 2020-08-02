#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  std::cout<<"Predict"<< std::endl; 
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  std::cout<<"End Predict"<< std::endl; 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  std::cout<<"KalmanFilter::Update"<< std::endl; 
  std::cout<<"KalmanFilter::Update x_"<< x_ << std::endl;
  std::cout<<"KalmanFilter::Update H_"<< H_ << std::endl;
  VectorXd z_pred = H_ * x_;
  std::cout<<"KalmanFilter::Update z_pred"<< z_pred << std::endl;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  std::cout<<"KalmanFilter::Update 1"<< std::endl; 
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  std::cout<<"KalmanFilter::Update 2"<< std::endl; 
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  std::cout<<"End KalmanFilter::Update"<< std::endl; 
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // recover state parameters
  VectorXd z_pred(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3); 

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  // float c3 = (c1*c2);    
  float c4 = (px*vx + py*vy) / c2;
    
  z_pred << c2, atan2(py,px), c4;
  
  std::cout<<"KalmanFilter::UpdateEKF z_pred"<< z_pred << std::endl;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  std::cout<<"KalmanFilter::UpdateEKF 1"<< std::endl; 
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  std::cout<<"KalmanFilter::UpdateEKF 2"<< std::endl; 
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  std::cout<<"End KalmanFilter::Update"<< std::endl; 
  
}
