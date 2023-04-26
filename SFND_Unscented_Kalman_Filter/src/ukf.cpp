
#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#define std_ldr_V  .20
#define std_ldr_yaw .20
#define std_ldr_yaw_r .20
#define std_rdr_yaw_ra 0.25
#define std_rdr_yaw_rat 0.25
#define Nx 5

using Eigen::MatrixXd;
using Eigen::VectorXd;



UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.3;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  n_x_ = 5;
  
  n_aug_ = 7;
  
  lambda_ = 3 - n_aug_;
  
  n_z = 3 ;
      
    
  is_initialized_ = false;
  
  Xsig_pred_ =  MatrixXd(n_x_ , 2 * n_aug_ + 1);
  Xsig_pred_.setZero();
  
  weights_ = VectorXd(2 * n_aug_ + 1);
  
  
  for(int i = 0; i < 2 * n_aug_ + 1; i++)   
       i == 0 ? weights_(i) = lambda_ / (lambda_ + n_aug_) : weights_(i) = 0.5 / (lambda_ + n_aug_);
  
  
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
   // TODO: Complete this function! Make sure you switch between lidar and radar measurements
if ( is_initialized_ == false )
{
  if(meas_package.sensor_type_ == MeasurementPackage::LASER )
  {
    
   x_ << meas_package.raw_measurements_(0),  meas_package.raw_measurements_(1), 0 , 0 , 0;
        
    P_.fill(0.0);
    P_(0, 0) = std_laspx_ * std_laspx_ ;
    P_(1, 1) = std_laspy_ * std_laspy_ ;
    P_(2, 2) =  1;//std_laspy_ * std_laspy_;
    P_(3, 3) = std_laspy_ * std_laspy_;
    P_(4, 4) = std_laspy_ * std_laspy_;
 
  }
  
  else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
  
		const double rho = meas_package.raw_measurements_(0); 	// range
		const double phi = meas_package.raw_measurements_(1);		// bearing
		const double rho_dot = meas_package.raw_measurements_(2);	// range change
	
		const double px = rho * sin(phi); // x-pos
		const double py = rho * cos(phi); // y-pos
	
		const double vx = rho_dot * cos(phi);  // velocity x
	    const double vy = rho_dot * sin(phi);  // velocity y
        const double v = sqrt(vx * vx + vy * vy);
	
	    x_ << px, py, v, 0, 0;
		 
		P_.fill(0);
		P_(0, 0) = std_laspx_ * std_laspx_ ;
		P_(1, 1) = std_laspy_ * std_laspy_ ;
		P_(2, 2) = std_radrd_ * std_radrd_;
		P_(3, 3) = std_radphi_ * std_radphi_ ;
		P_(4, 4) = std_radphi_ * std_radphi_;
		  
  }
  
  time_us_ = meas_package.timestamp_;
  is_initialized_ = true;
  
}
  

   Prediction(( meas_package.timestamp_ - time_us_ ) / 1e6);
  
  if( meas_package.sensor_type_ == MeasurementPackage::LASER )     
    UpdateLidar(meas_package);
    
  else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)   
    UpdateRadar(meas_package);    
    time_us_ = meas_package.timestamp_;

}

void UKF::Prediction(double delta_t) {
     
    MatrixXd sig = MatrixXd(n_aug_ , 2 * n_aug_ +1);
     
    MatrixXd P_aug(n_aug_ , n_aug_);

    VectorXd x_aug(n_aug_);
  
    x_aug<< x_ , 0, 0;
  
    P_aug.fill(0.0); 
 
   for(int i = 0; i < n_aug_ ; i++)
    {
     for(int j = 0; j < n_aug_; j++)
      {
       
         if( i == 5 && j == 5 ) P_aug(i,j) =  pow(std_a_,2);
       
         else if( i == 6 && j == 6 ) P_aug(i,j) = pow(std_yawdd_,2);
       
         else if( i < 5  && j < 5 ) P_aug(i,j) = P_(i,j); 
       
         else P_aug(i,j) = 0;
      }
 }
    
    MatrixXd A = P_aug.llt().matrixL();
    sig.col(0) = x_aug ;
  
  for(int i = 0; i < n_aug_  ; i++)
  {
    sig.col(i+1) = x_aug + sqrt( lambda_ + n_aug_) * A.col(i);
    sig.col(i + n_aug_ +1 ) = x_aug - sqrt( lambda_ + n_aug_) * A.col(i);
  }
  
  
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    {
      
    double p_x = sig(0,i);
    double p_y = sig(1,i);
    double v =  sig(2,i);
    double yaw = sig(3,i);
    double yawd = sig(4,i);
    double nu_a = sig(5,i);
    double nu_yawdd = sig(6,i);
    double px_p, py_p;
      
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/ yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v/ yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
    } 
    
    else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }
      
    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
    }
  
  
  x_.fill(0.0);
  for(int i  = 0; i < 2 * n_aug_ + 1; i++)
    x_ += weights_(i) * Xsig_pred_.col(i);
  
  P_.fill(0.0);
  for(int i  = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd dif =  Xsig_pred_.col(i) - x_ ;
    while (dif(3)> M_PI) dif(3) -=2.*M_PI;
    while (dif(3)< -M_PI) dif(3) +=2.*M_PI;
    P_ += weights_(i) * dif * dif.transpose();
 
  }
  
} 

void UKF::UpdateLidar(MeasurementPackage meas_package) {
   
  MatrixXd H_ = MatrixXd(2, 5);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;
  MatrixXd R_ = MatrixXd(2, 2);
  R_ << (std_laspx_*std_laspx_), 0,
        0, (std_laspy_*std_laspy_);

  VectorXd z_pred = H_ * x_;
  VectorXd y = meas_package.raw_measurements_ - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_; 
    
  
}
  
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  
  MatrixXd Zsig(3 , 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  S = MatrixXd(3 , 3);
  MatrixXd Tc = MatrixXd(n_x_ , n_z);
  Tc.setZero();
  Z_mean = VectorXd(n_z);
 
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double rad = Xsig_pred_(3,i);
    double v1 = cos(rad)*v;
    double v2 = sin(rad)*v;
    
   Zsig(0 , i) = sqrt( px * px + py * py );
   Zsig(1 , i) = atan2( py , px );
   Zsig(2 , i) = (px * v1 + py * v2 ) / sqrt( px * px + py * py );
 }
  
  Z_mean.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
     Z_mean +=  weights_(i) * Zsig.col(i);
  
  S.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
       VectorXd dif = Zsig.col(i) -  Z_mean;
       while (dif(1)> M_PI) dif(1)-=2.*M_PI;
       while (dif(1)<-M_PI) dif(1)+=2.*M_PI;
       S += weights_(i) * dif * dif.transpose();
      
  }
   
  R_Z = MatrixXd(3 , 3);
  R_Z.fill(0.0);
  R_Z(0,0) = std_radr_ * std_radr_;
  R_Z(1,1) = std_radphi_ * std_radphi_;
  R_Z(2,2) = std_radrd_ * std_radrd_;
  
  S += R_Z;
  
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
      VectorXd dif_2 = Xsig_pred_.col(i) - x_; 
      while (dif_2(3)> M_PI) dif_2(3)-=2.*M_PI;
      while (dif_2(3)<-M_PI) dif_2(3)+=2.*M_PI;
    
      VectorXd dif_3 = Zsig.col(i) - Z_mean ;
      while (dif_3(1)> M_PI) dif_3(1)-=2.*M_PI;
      while (dif_3(1)<-M_PI) dif_3(1)+=2.*M_PI;
    
      Tc += weights_(i) * dif_2 * dif_3.transpose(); 
  }
  
  MatrixXd K = Tc * S.inverse(); 
  VectorXd dif_4 =  meas_package.raw_measurements_ - Z_mean;
  while (dif_4(1)> M_PI) dif_4(1)-=2.*M_PI;
  while (dif_4(1)<-M_PI) dif_4(1)+=2.*M_PI;
  
 x_ += K * dif_4 ;
 P_ -= K * S * K.transpose(); 
 

}


