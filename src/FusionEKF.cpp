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
//  cout<< "FusionEKF::FusionEKF" << endl;
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
 
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
//  cout << "FusionEKF::ProcessMeasurement" << endl;
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
//    cout << "Initializing EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {      
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
//      cout << "Initializing Radar " << endl;
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
//      cout << "Initializing Laser " << endl;
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
//    cout << "End Initializing" << endl;
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
//  cout<<"measurement_pack.timestamp_: " << measurement_pack.timestamp_ <<endl;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  
  G << dt_2/2, 0, 0, dt_2/2, dt, 0, 0, dt;
  Q_v << noise_ax, 0, 0, noise_ay;
  
  ekf_.Q_ = G*Q_v*G.transpose();	
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0, 
  			 0, 1, 0, dt,
  			 0, 0, 1,  0,
  			 0, 0, 0,  1;
  
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
//   cout<<"Radar"<<endl;
    VectorXd measurement(3);           
    measurement << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2];
    if(measurement(1) < 0) 
  		measurement(1) = measurement(1) + 2*PI;	// map from [-Pi, Pi] to [0, 2PI]
    
    Hj_ = tools.CalculateJacobian(ekf_.x_);
 
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement);
//    cout << "End Radar" << endl;
  }
  else 
  {
    // TODO: Laser updates
//    cout<<"Laser"<<endl;
    VectorXd z(2);
    VectorXd measurement(2);  
    
    cout<<measurement_pack.raw_measurements_<<endl;
    measurement << 	measurement_pack.raw_measurements_[0], 
    				measurement_pack.raw_measurements_[1];
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    z = measurement;
    ekf_.Update(z);
    cout<<"End Laser"<<endl;
  }
  
   
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
