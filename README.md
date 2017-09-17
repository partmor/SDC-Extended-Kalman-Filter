# **CarND: Extended Kalman Filter**  [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
[//]: # (Image References)
[process_overview]: ./img/process_overview.png

The goal of this project is to develop a complete **sensor fusion model**, based on the **Extended Kalman Filter**, in order to esimate the state of a moving vehicle combining measurments from two different types of sensors: **lidar** and **radar**.

## State vector

The state of the moving object is characterised by its 2D (x,y) position and velocity components: `[px, py, vx, vy]`.

## Processing Flow Overview

The process begins with a lidar or radar measurement being pushed into the system.

![process_overview]

### 0. Initialization

In the very first iteration of the algorithm, an initialization step replaces the prediction-update combo. The state of the vehicle is initialized with the help of the first set of incoming measurments. **Laser** measurements only provide information about `px` and `py`. Similarly, **radar** only determines completely the `px` and `py` positions, and only provides partial information about the initial velocity components `vx` and `vy` through the radial projection `rho_dot`.

In the laser case, the velocities where arbitrarily set to `vx = 5`, `vy = 0`, representing an initial moderate horizontal velocity. For initialization with radar measurements, `vx` and `vy` where set equal to the `x` and `y` projections of `rho_dot` respectively; in absence of any information on the tangential polar component, this is a reasonable option.

In any case, the state covariance matrix `P` was conveniently initialized to reflect the uncertainty of the initial estimates: position components given by the sensor's noise specs, and the velocity uncertainty of the same order of magnitude as the actual velocity.

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

## Performance evaluation

The ground truth positions for each instant are available. Thus, the performance of the tracking algorithm is evaluated with the **Root Mean Squared Error** *(RMSE)*, an accuracy metric that measures the deviation of the estimated state from the true state.

The initialization of the state vector and covariance matrix was tweaked to yield `[px, py, vx, vy]` RMSEs bellow `[.11, .11, 0.52, 0.52]` throughout most of the simulation.
