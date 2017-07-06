#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    * Predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * Update the state with Standard Kalman Filter equations
    * given the measurment vector @param z
  */
  
  MatrixXd I = MatrixXd::Identity(z.size(), z.size());
  
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; 
  MatrixXd K = P_ * Ht * S.inverse();
  x_ += K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * Update the state with Extended Kalman Filter equations
    * given the measurement vector @param z
  */
  
  MatrixXd I = MatrixXd::Identity(z.size(), z.size());
  
  // components of state vector in polar coordinates
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
  VectorXd hx(3);
  hx << rho, phi, rho_dot;
  
  VectorXd y = z - hx;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; 
  MatrixXd K = P_ * Ht * S.inverse();
  x_ += K * y;
  P_ = (I - K * H_) * P_;
}
