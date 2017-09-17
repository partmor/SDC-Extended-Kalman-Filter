# **CarND: Extended Kalman Filter**  [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
[//]: # (Image References)
[sample_gif]: ./img/pf_sample.gif

The goal of this project is to develop a complete **sensor fusion model**, based on the **Extended Kalman Filter**, in order to esimate the state of a moving vehicle combining measurments from two different types of sensors: **lidar** and **radar**.

## Processing Flow Overview

The process begins with a lidar or radar measurement being pushed into the system.

### 1. Prediction

The state of the object is predicted by applying a **linear motion model** with **constant velocity**, taking into account the elapsed time `dt` between the current and previous observation.

The prediction step is implemented in the `KalmanFilter::predict` method.

`F` is the state transition matrix, and `Q` is the process covariance matrix. Because the state vector only tracks position and velocity assuming the latter is constant between time intervals, acceleration is modeled as a random noise. The `Q` matrix includes time `dt` to account for the fact that as more time passes, we become more uncertain about our position and velocity. 

### 2. Update

The measurement update step depends on sensor type. 

For **laser** measurments the measurement function is **linear** in `x`, hence the **Standard Kalman Filter** update equations are applied.See `KalmanFilter::Update` for more details.

However, **radar** measurements involve a **non-linear** measurement function (cartesian to polar coordinates). The **Extended Kalman Filter** approach consists in linearizing the non-linear measurement function, retaining its Taylor series expansion only up to the linear term. As a result, the `H` matrix in the Kalman filter is replaced by the Jacobian matrix when calculating `S`, `K` and `P`. To calculate the `y` error, the non-linear function is used directly.

This casuistry has ben resolved in `FusionEKF::processMeasurement` as follows:
```c++
/*
Radar updates
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
```

For more details see the `KalmanFilter::Update` and `KalmanFilter::UpdateEKF` methods.

*Note*: The only non-linearity present in the problem is the measurement function. Since a linear motion model was chosen for this project, the state transition function is linear and it is not necessary to apply the Extended Kalman Filter formulation in the prediction step. 


