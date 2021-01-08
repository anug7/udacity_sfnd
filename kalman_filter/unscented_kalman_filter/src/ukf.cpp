#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
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
  std_yawdd_ = 2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  n_x_ = 5;

  n_aug_ = 7;

  lambda_ = 3 - n_aug_;

  P_ << 10, 0, 0, 0, 0,
         0, 10, 0, 0, 0,
         0, 0, 10, 0, 0,
         0, 0, 0, 10, 0,
         0, 0, 0, 0, 10;

  Xsig_pred_ = Eigen::MatrixXd(n_x_, n_aug_ * 2 + 1);
  Xsig_pred_.fill(0);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (isFirstReading){
      if(meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER){
            x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0., 0, 0;
            P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
                  0, std_laspy_ * std_laspy_, 0, 0, 0,
                  0, 0, 1, 0, 0,
                  0, 0, 0, 1, 0,
                  0, 0, 0, 0, 1;
      }else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR){
            double range = meas_package.raw_measurements_(0),
                   theta = meas_package.raw_measurements_(1),
                   thetad = meas_package.raw_measurements_(2);

            double px = range * cos(theta),
                   py = range * sin(theta),
                   vx = thetad * cos(theta),
                   vy = thetad * sin(theta),
                   v = sqrt(vx * vx + vy * vy);


            x_ << px, py, v, 0, 0;
            P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
                    0, std_radr_ * std_radr_, 0, 0, 0,
                    0, 0, std_radrd_ * std_radrd_, 0, 0,
                    0, 0, 0, std_radphi_, 0,
                    0, 0, 0, 0, std_radphi_;
      }
      std::string sensor = (meas_package.sensor_type_  == MeasurementPackage::SensorType::LASER)? "Lidar": "Radar"; 
      std::cout << "UKF state initialized with sensor reading: " << sensor << "\n";
      prev_time = meas_package.timestamp_;
      isFirstReading = false;
  }else{
      double dt = (meas_package.timestamp_ - prev_time) / 1000000.0;
      prev_time = meas_package.timestamp_;
      Prediction(dt);
      if(meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER && use_laser_){
          UpdateLidar(meas_package);
      }else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && use_radar_){
          UpdateRadar(meas_package);
      }
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  // create augmented mean vector
  Eigen::VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  Eigen::MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  Eigen::MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // predict sigma points
  for (int i = 0; i< 2 * n_aug_ + 1; ++i) {
    // extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t) );
    } else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  weights.fill(0.5/(n_aug_ + lambda_));
  // set weights
  double weight_0 = lambda_/(lambda_ + n_aug_);
  weights(0) = weight_0;

  x_.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    x_ = x_ + weights(i) * Xsig_pred_.col(i);
  }

  P_.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) <- M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // set measurement dimension, px,py
  int n_z_ = 2;
  
  // set vector for weights
  Eigen::VectorXd weights = Eigen::VectorXd(2 * n_aug_+ 1);
  double weight_0 = lambda_/(lambda_ + n_aug_);
  double weight = 0.5/(lambda_ + n_aug_);
  weights.fill(weight);
  weights(0) = weight_0;

  VectorXd z = VectorXd(n_z_);
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1);

  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  Zsig.fill(0);

  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);

  MatrixXd S = MatrixXd(n_z_, n_z_);
  S.fill(0.0);
  
  MatrixXd R = MatrixXd(2, 2);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      double p_x = Xsig_pred_(0, i);
      double p_y = Xsig_pred_(1, i);

      Zsig(0, i) = p_x;
      Zsig(1, i) = p_y;
      z_pred += weights(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      VectorXd z_diff = Zsig.col(i) - z_pred;
      S += weights(i) * z_diff * z_diff.transpose();
  }

  S = S + R;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      VectorXd z_diff = Zsig.col(i) - z_pred;

      VectorXd x_diff = Xsig_pred_.col(i) - x_;

      while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
      while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;

      Tc += weights(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();

  VectorXd z_diff = z - z_pred;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  lidar_nis.push_back(z_diff.transpose() * S.inverse() * z_diff);
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  // set vector for weights
  Eigen::VectorXd weights = Eigen::VectorXd(2 * n_aug_+ 1);
  double weight_0 = lambda_/(lambda_ + n_aug_);
  double weight = 0.5/(lambda_ + n_aug_);
  weights.fill(weight);
  weights(0) = weight_0;
  
// Dimension of measurement
  int n_z_ = 3;

  Eigen::MatrixXd Zsig(n_z_, n_aug_ * 2 + 1);
  
  // mean predicted measurement
  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z_);
  z_pred.fill(0.0);

  // measurement covariance matrix S
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z_, n_z_);

  // transform sigma points into measurement space
  Zsig.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
      double px = Xsig_pred_(0, i),
             py = Xsig_pred_(1, i),
             v = Xsig_pred_(2, i),
             si = Xsig_pred_(3, i);
      double spxpy = sqrt(px * px + py * py);
      Zsig(0, i) = spxpy;
      Zsig(1, i) = atan2(py, px);
      Zsig(2, i) = ((px * cos(si) * v) + (py * sin(si) * v)) / spxpy;
      
      z_pred += weights(i) * Zsig.col(i);
  }
  
  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z_, n_z_);

  R <<  std_radr_ * std_radr_, 0, 0,
        0, std_radphi_ * std_radphi_, 0,
        0, 0, std_radrd_ * std_radrd_;
  S = S + R;

  // create matrix for cross correlation Tc
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z_);

  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    // state difference
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  VectorXd z(3);
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1),
       meas_package.raw_measurements_(2);
       

  // residual
  Eigen::VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
  radar_nis.push_back(z_diff.transpose() * S.inverse() * z_diff);
}
