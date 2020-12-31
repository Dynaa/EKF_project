# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric : The px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].

The main program can be built and run by doing the following from the project top directory :

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF


**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Algorithm
I follow every steps described in the class :
    ![Sensor Fusion flow](https://github.com/Dynaa/EKF_project/blob/master/Docs/ekf_flow.jpg)

# Initialisation 
In this first step I initialized variables and matrices (x, F, H_laser, H_jacobian, P, etc.) :
```cpp
{
 // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // tools 
  tools = Tools();

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  
  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //measurement matrix - radar
  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;

  
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // initial state covariance matrix P_
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
  }
```

# Initializing the Kalman Filter
In this step the Kalman Filter is initialized, we have to take care regarding the first measurement we receive (Lidar or Radar).

# Predict and Update Steps
Kalman filters remained on two steps : 

![KF steps](https://github.com/Dynaa/EKF_project/blob/master/Docs/Steps.png)
```cpp
{
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
}
```



Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. The Update function will use the standard Kalman filter equations. The UpdateEKF will use the extended Kalman filter equations : 
```cpp
{
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
}
```

```cpp
{
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // Compute h function in order to determinate z_pred value 
  VectorXd z_pred(3) ;
  float rho = std::sqrt(x_(0)*x_(0)+x_(1)*x_(1)) ;
  float phi = std::atan2(x_(1),x_(0));
  float phi_dot = 0. ;
  
  if(rho>0.0001)
  {
    phi_dot = (x_(0)*x_(2)+x_(1)*x_(3))/(rho);
  }


  z_pred(0) = rho ;
  z_pred(1) = phi ;
  z_pred(2) = phi_dot ; 

  VectorXd y = z - z_pred;
  
  if (y(1) < -M_PI){
    y(1) += 2 * M_PI;
  }
  if (y(1) > M_PI){
    y(1) -= 2 * M_PI;
  }


  MatrixXd Ht = H_.transpose() ;
  MatrixXd S = H_ * P_ * Ht + R_ ;
  MatrixXd Si = S.inverse() ;
  MatrixXd PHt = P_ * Ht ;
  MatrixXd K = PHt * Si ;

  //new estimate
  x_ = x_ + (K * y) ;
  long x_size = x_.size() ;
  MatrixXd I = MatrixXd::Identity(x_size, x_size) ;
  P_ = (I - K * H_) * P_ ;
}
}
```

# Tools
Finally some tools were developed for Jacobian matrix computation and RMSE for accuracy assessment.

