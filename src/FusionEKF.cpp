#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  
  //initialize Tools
  Tools tools;
  
  is_initialized_ = false;

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
  
  // measurement matrix for laser
  H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;
  
  Hj_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
  
  //create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);
  
  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
  
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  
  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    // first timestamp needs to be the first in the measurement NOT 1970...
    previous_timestamp_ = measurement_pack.timestamp_;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // convert from radar measurements (polar) to cartesian and initialize state
      ekf_.x_ << measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]), measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]), 0.1, 0.1;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
      // initialize the state
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // update the transition matrix F
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // set the process covariance matrix Q
  MatrixXd G;
  G = MatrixXd(4,2);
  G << pow(dt,2)/2, 0,
      0, pow(dt,2)/2,
      dt, 0,
      0, dt;
  
  MatrixXd Q_v;
  Q_v = MatrixXd(2,2);
  // noise ax and ay predefined as 9
  Q_v << noise_ax, 0,
      0, noise_ay;
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ = G*Q_v*G.transpose();
  
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
    // make sure to use the right measurement covariance matrix and measurement function
    ekf_.R_ = R_radar_;

    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;

    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    
    // make sure to use the right measurement covariance matrix and measurement function
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
