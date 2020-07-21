#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  cout<< "FusionEKF::FusionEKF" << endl;
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;
   
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  
/* 
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  
  Hj_ << ; // radar
*/
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  cout << "FusionEKF::ProcessMeasurement" << endl;
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "Initializing EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {      
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      cout << "Initializing Radar " << endl;
      float p_x = cos(measurement_pack.raw_measurements_[0]) * measurement_pack.raw_measurements_[1];  
      float p_y = sin(measurement_pack.raw_measurements_[0]) * measurement_pack.raw_measurements_[1];  
      
      // v_x = ρ_dot * cos(φ)
	  // v_y = ρ_dot * sin(φ)
      float v_x = cos(measurement_pack.raw_measurements_[1]) * measurement_pack.raw_measurements_[2];  
      float v_y = sin(measurement_pack.raw_measurements_[1]) * measurement_pack.raw_measurements_[2];  
      ekf_.x_ << p_x, p_y, v_x, v_y; 
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      cout << "Initializing Laser " << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
      			measurement_pack.raw_measurements_[1], 
      			0, 
      			0;
    }
	previous_timestamp_ = measurement_pack.timestamp_;
    
    
    // state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "End Initializing" << endl;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */  
  MatrixXd G(4,2);
  MatrixXd Q_v(2,2);
  
  int noise_ax = 9; 
  int noise_ay = 9;
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt_2 = dt * dt;
  
  previous_timestamp_ = measurement_pack.timestamp_;
  
  G << dt_2/2, 0, 0, dt_2/2, dt, 0, 0, dt;
  Q_v << noise_ax, 0, 0, noise_ay;
  
  cout<<"ProcessMeasurement - 1 "<< endl; 
  ekf_.Q_ = G*Q_v*G.transpose();
  cout<<"ProcessMeasurement - 2 "<< endl; 	
  
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0, 
  			 0, 1, 0, dt,
  			 0, 0, 1,  0,
  			 0, 0, 0,  1;
  //ekf_.F_(0, 2) = dt;
 // ekf_.F_(1, 3) = dt;
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // TODO: Radar updates
    cout<<"Radar"<<endl;
    VectorXd measurement(3);      
    VectorXd y(3);      
    measurement << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2];
    
    // from cart. system to polar
    // y=z−h(x′)
    
    // recover state parameters
    float px = ekf_.x_(0);
    float py = ekf_.x_(1);
    float vx = ekf_.x_(2);
    float vy = ekf_.x_(3); 

    // pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    // float c3 = (c1*c2);    
    float c4 = (px*vx + py*vy) / c1;
    
    y << c2, atan2(py,px), c4;
    
    Hj_ = tools.CalculateJacobian(ekf_.x_);
/*
    // check division by zero
    if (fabs(c1) < 0.0001) 
    {
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    }
	else
    {
      // compute the Jacobian matrix
      Hj_ << (px/c2), (py/c2), 0, 0,
          -(py/c1), (px/c1), 0, 0,
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    }
        */
    VectorXd z(3);     
    z = y; 
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.Update(z);
    cout << "End Radar" << endl;
  }
  else 
  {
    // TODO: Laser updates
    cout<<"Laser"<<endl;
    VectorXd z(2);
    VectorXd measurement(2);  
    
    cout<<"Laser 0"<<endl;
    cout<<measurement_pack.raw_measurements_<<endl;
    measurement << 	measurement_pack.raw_measurements_[0], 
    				measurement_pack.raw_measurements_[1];
	cout<<"Laser 1"<<endl;
    ekf_.H_ = H_laser_;
    cout<<"Laser 2"<<endl;
    ekf_.R_ = R_laser_;
    cout<<"Laser 3"<<endl;
    z = measurement;
    cout<<"Laser 4"<<endl;
    ekf_.Update(z);
    cout<<"End Laser"<<endl;
  }
  
   
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
