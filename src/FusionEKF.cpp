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

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //laser measurement matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0 , 0;

  //radar measurement jacobian matrix
  Hj_ = MatrixXd(3, 4);
  Hj_ << MatrixXd::Zero(3, 4);
  
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
    cout << "EKF: " << endl;
    
    /**
    State vector initialization
    */
    
    float px = 0, py = 0, vx = 0, vy = 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.

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
      //set the state with the initial location and zero velocity
      px = measurement_pack.raw_measurements_(0);
      py = measurement_pack.raw_measurements_(1);
      vx = 0;
      vy = 0;
    }

    VectorXd x_in = VectorXd(4);
    x_in << px, py, vx, vy;
    ekf_.x_ = x_in;

    /**
    State transition matrix initialization
    */
    
  	MatrixXd F_in = MatrixXd::Identity(4, 4);
  	ekf_.F_ = F_in;
  	
  	/**
    State covariance matrix initialization
    */
    MatrixXd P_in = MatrixXd(4, 4);
	  P_in << 1, 0, 0, 0,
			      0, 1, 0, 0,
			      0, 0, 1000, 0,
			      0, 0, 0, 1000;
		ekf_.P_ = P_in;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

/*****************************************************************************
*  Prediction
****************************************************************************/
  
  // elapsed time between current and prev measurement (in seconds)
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;
	previous_timestamp_ = measurement_pack.timestamp_;
	
	// update F matrix
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

  // update Q matrix
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			       0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			       dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			       0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
			       
  // perform prediction 
  ekf_.Predict();

/*****************************************************************************
*  Update
****************************************************************************/

  /*
  Laser updates
  */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // calculate the radar's Jacobian H and use this one to update state
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    // use radar measurement covariance matrix R
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
	/*
  Laser updates
  */
  else {
    // recover the laser's H and use this one to update state
    ekf_.H_ = H_laser_;
    // use laser measurement covariance matrix R
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
