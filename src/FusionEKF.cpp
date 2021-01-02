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

//   Eigen::MatrixXd H_laser_;
//   Eigen::MatrixXd Hj_;
//   MatrixXd x(）;
//   Hj_ = Tools::CalculateJacobian(x);
  //state vector
  //process state function
  //jacobian of observation function
    // the initial transition matrix F_
  
    // create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  
    // measurement matrix
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;
  
    // set the process covariance matrix Q
    // set the acceleration noise components
  noise_ax = 9.0;
  noise_ay = 9.0;
  

  
  ekf_.Q_ = MatrixXd(4, 4);
//   ekf_.Q_ <<   noise_ax, 0,  noise_ax, 0,
//          0,  noise_ay, 0, noise_ay,
//          noise_ax, 0,  noise_ax, 0,
//          0,  noise_ay, 0, noise_ay;
  
    //measurement covariance
  ekf_.R_ = MatrixXd(2, 2);
  ekf_.R_ << 0.0225, 0,
            0, 0.0225;
  
  
  //ekf_.init(X_in, P_in , F_in, H_in  ,R_in , Q_in);
             


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      //covjert first
      float phi = measurement_pack.raw_measurements_(1);
      float y = measurement_pack.raw_measurements_[0] * sin(phi);
      float x = measurement_pack.raw_measurements_[0] * cos(phi);
      float vx = measurement_pack.raw_measurements_(2) * cos(phi);
      float vy = measurement_pack.raw_measurements_(2) * sin(phi);
                                                             
      
      ekf_.x_<< x,
                y, 
      		   vx,
     			vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      //set first state to measurement
          // set the state with the initial location and zero velocity
    ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
 // std::cout<<" ekf initilaized......"<<std::endl;

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  //std::cout<<" deltaT  "<<dt<<std::endl;
  
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  noise_ax = 3;
  noise_ay = 3;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
//   std::cout<<"here ......." <<std::endl;

//   std::cout<<ekf_.x_<<std::endl;
//   std::cout<<ekf_.P_<<std::endl;
//   std::cout<<std::endl;
//   std::cout<<ekf_.F_<<std::endl;
//    std::cout<<std::endl;
//   std::cout<<ekf_.Q_<<std::endl;
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates  
    //convert
    // measurement_pack.raw_measurements_
    //compute jacobian
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj_);

  } 
  else
  {
    //TODO: Laser up
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
   }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
