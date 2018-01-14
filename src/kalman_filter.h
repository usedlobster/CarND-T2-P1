#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_lidar , H_radar ;

  // measurement covariance matrix
  Eigen::MatrixXd R_lidar , H_radar ;

  // C'tor
  KalmanFilter();

  // D'tor
  virtual ~KalmanFilter();

  void Predict( float dt );


};

#endif /* KALMAN_FILTER_H_ */
