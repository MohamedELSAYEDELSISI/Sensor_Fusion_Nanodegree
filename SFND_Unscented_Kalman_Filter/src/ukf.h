
#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 
 public: 
  
  UKF();

  
  virtual ~UKF();

  
  void ProcessMeasurement(MeasurementPackage meas_package);

  
  void Prediction(double delta_t);

  
  void UpdateLidar(MeasurementPackage meas_package);

 
  void UpdateRadar(MeasurementPackage meas_package);


  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // vector for predicted state 
  Eigen::VectorXd x_mean;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;


  // State dimension
  int n_x_, n_z;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
  
  // measurment prediction matrix 
  Eigen::MatrixXd H;
  
  // measurment noise covirance 
  
  Eigen::MatrixXd R, R_Z, S;
  
  Eigen::VectorXd y, Z_mean; 
  
  double r, phi, r_dot;
  
};
#endif  // UKF_H
