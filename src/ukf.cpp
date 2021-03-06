#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
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
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  n_aug_ = 7;
  n_x_ = 5;

  Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);
  lambda_ = 3 - n_aug_;
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for(int i=1;i<=2*n_aug_;i++){
    weights_(i) = 0.5 / (lambda_+n_aug_);
  }
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if(!is_initialized_){
    
    if(meas_package.sensor_type_ ==  MeasurementPackage::LASER){
      x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
    }
    P_ = MatrixXd::Identity(5,5);
    previous_timestamp_  = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }
  
  double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = meas_package.timestamp_;

  Prediction(delta_t);
  if(meas_package.sensor_type_ ==  MeasurementPackage::LASER){
    //cout << "LASER" << endl;
    UpdateLidar(meas_package);
  }
  else{
    //cout << "RADAR" << endl;
    UpdateRadar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  
  // generate sigma points
  VectorXd x_aug(n_aug_);
  MatrixXd P_aug(n_aug_,n_aug_);
  MatrixXd Xsig_aug(n_aug_,2*n_aug_+1);
  P_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_ + 1) = 0;
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_ * std_yawdd_;
  MatrixXd P_aug_sqrt = P_aug.llt().matrixL();
 
  P_aug_sqrt = sqrt(lambda_ + n_aug_) * P_aug_sqrt;
  Xsig_aug.col(0) = x_aug;
  for(int i=0;i<n_aug_;i++){
    Xsig_aug.col(i+1) = x_aug + P_aug_sqrt.col(i);
    Xsig_aug.col(i+n_aug_+1) = x_aug - P_aug_sqrt.col(i);
  }
  // predict sigma points
  for(int i=0; i<2*n_aug_+1; i++){
    double px, py, v, phi, phi_dot;
    double new_px, new_py, new_v, new_phi, new_phi_dot;
    double nu_a,nu_phidd;
    px =  Xsig_aug(0,i);
    py =  Xsig_aug(1,i);
    v = Xsig_aug(2,i);
    phi = Xsig_aug(3,i);
    phi_dot = Xsig_aug(4,i);
    nu_a = Xsig_aug(5,i);
    nu_phidd = Xsig_aug(6,i);

    if(fabs(phi_dot)>0.001){
      new_px = px + v * (sin(phi + phi_dot * delta_t) - sin(phi))/phi_dot + 0.5 * delta_t * delta_t * nu_a * cos(phi);
      new_py = py + v * (-cos(phi + phi_dot * delta_t) + cos(phi))/phi_dot + 0.5 * delta_t * delta_t * nu_a * sin(phi);
    }
    else{
      new_px = px + v * cos(phi) * delta_t + 0.5 * delta_t * delta_t * nu_a * cos(phi);
      new_py = py + v * sin(phi) * delta_t + 0.5 * delta_t * delta_t * nu_a * sin(phi);
    }
    
    new_v = v + nu_a * delta_t;
    new_phi = phi + phi_dot * delta_t + 0.5 * delta_t * delta_t * nu_phidd;
    new_phi_dot = phi_dot + nu_phidd * delta_t;

    Xsig_pred_(0,i) = new_px;
    Xsig_pred_(1,i) = new_py;
    Xsig_pred_(2,i) = new_v;
    Xsig_pred_(3,i) = new_phi;
    Xsig_pred_(4,i) = new_phi_dot;
  }
  
  VectorXd mean(5);
  mean = Xsig_pred_ * weights_;
  
  MatrixXd covariance(5,5);
  covariance.fill(0.0);
  for(int i = 0;i < 2 * n_aug_ + 1;i++){
    covariance += weights_(i) * (Xsig_pred_.col(i) - mean) * (Xsig_pred_.col(i) - mean).transpose();
  }
  x_ = mean;
  P_ = covariance;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  
  MatrixXd R(2,2);
  R << std_laspx_ * std_laspx_,0,
       0, std_laspy_ * std_laspy_;
 
  MatrixXd Xsig_meas(2,2*n_aug_+1);
  for(int i=0;i<2*n_aug_+1;i++){
    Xsig_meas(0,i) = Xsig_pred_(0,i);
    Xsig_meas(1,i) = Xsig_pred_(1,i);
  }

  //calculate the mean and covariance
  VectorXd meas_mean;
  meas_mean = Xsig_meas * weights_;
  
  MatrixXd meas_corv (2, 2);;
  meas_corv.fill(0.0);
  for(int i=0;i<2*n_aug_+1;i++){
    meas_corv += weights_(i) * (Xsig_meas.col(i) - meas_mean) * (Xsig_meas.col(i) - meas_mean).transpose();
  }
  meas_corv += R;

  VectorXd y = meas_package.raw_measurements_ - meas_mean;
  
  MatrixXd Tc = MatrixXd(n_x_,2); 
  Tc.fill(0.0);
  for(int i=0;i<2*n_aug_+1;i++){
    Tc += weights_(i) * (Xsig_pred_.col(i)- x_) * (Xsig_meas.col(i) - meas_mean).transpose();
  }
  
  MatrixXd K = Tc * meas_corv.inverse();

  x_ = x_ + K * y;
  P_ = P_ - K * meas_corv * K.transpose();

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  
  MatrixXd R(3,3);
  R << std_radr_ * std_radr_ , 0, 0,
       0, std_radphi_ * std_radphi_,0,
       0, 0, std_radrd_ * std_radrd_;

  MatrixXd Xsig_meas(3,2*n_aug_+1);
  for(int i = 0; i < 2 * n_aug_ + 1; i++){
    double rho, angle, rho_dot;
    double px,py,v,phi,phi_dot;
    
    px = Xsig_pred_(0,i);
    py = Xsig_pred_(1,i);
    v = Xsig_pred_(2,i);
    phi = Xsig_pred_(3,i);
    phi_dot = Xsig_pred_(4,i);

    rho = sqrt(px*px + py*py);
    angle = atan2(py,px);
    rho_dot = (px*v*cos(phi) + py*v*sin(phi))/rho;
  
    Xsig_meas(0,i) = rho;
    Xsig_meas(1,i) = angle;
    Xsig_meas(2,i) = rho_dot;
  }
  

  VectorXd meas_mean(3);
  meas_mean = Xsig_meas * weights_;

  MatrixXd meas_corv = MatrixXd(3,3);
  meas_corv.fill(0.0);
  for(int i=0;i<2*n_aug_+1;i++){
    meas_corv += weights_(i)*(Xsig_meas.col(i) - meas_mean) * (Xsig_meas.col(i) - meas_mean).transpose();
  }
  meas_corv += R;
  
 
  VectorXd y = meas_package.raw_measurements_ - meas_mean;
  
  MatrixXd Tc = MatrixXd(n_x_,3);
  Tc.fill(0.0);
  for(int i=0;i<2*n_aug_+1;i++){
    Tc += weights_(i) * (Xsig_pred_.col(i) - x_) * (Xsig_meas.col(i) - meas_mean).transpose();
  }
 
  MatrixXd K =  Tc * meas_corv.inverse();

  x_ = x_ + K * y;
  P_ = P_ - K * meas_corv * K.transpose();
 

}
