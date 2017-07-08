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

  //laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0 , 0;

  //radar measurement jacobian matrix
  Hj_ << MatrixXd::Zero(3, 4);

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
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    float px = 0, py = 0, vx = 0, vy = 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      //recover measurement parameters
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);

      px = rho * cos(phi);
      py = - rho * sin(phi);
      vx = rho_dot * cos(phi);
      vy = - rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */

      //set the state with the initial location and zero velocity
      px = measurement_pack.raw_measurements_(0);
      py = measurement_pack.raw_measurements_(1);
      vx = 0;
      vy = 0;
    }

    ekf_.x_ << px, py, vx, vy;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

/*****************************************************************************
*  Prediction
****************************************************************************/

  /**
  TODO:
  * Update the state transition matrix F according to the new elapsed time.
  - Time is measured in seconds.
  * Update the process noise covariance matrix.
  * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;

  //ekf_.F_ = 

  ekf_.Predict();

/*****************************************************************************
*  Update
****************************************************************************/

  /**
  TODO:
  * Use the sensor type to perform the update step.
  * Update the state and covariance matrices.
  */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
