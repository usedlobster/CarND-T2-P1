#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {

    // lidar observation noise fixed
    R_lidar_ = MatrixXd(2, 2);
    R_lidar_ << 0.0225, 0.0000,
                0.0000, 0.0225 ;

    // Lidar observation model
    H_lidar_ = MatrixXd( 2 , 4 ) ;
    H_lidar_ << 1, 0, 0, 0,
                0, 1, 0, 0 ;


    // radar observation model
    // to be computed at each radar update

    H_radar_  = MatrixXd( 3 , 4 ) ;
    H_radar_ << 0 , 0 , 0 , 0 ,
                0 , 0 , 0 , 0 ,
                0 , 0 , 0 , 0 ;

    // radar observation noise fixed
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0.0000, 0.00,
                0.00, 0.0009, 0.00,
                0.00, 0.0000, 0.09 ;

    // P = error covariance matrix
    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0    , 0 ,
          0, 1, 0    , 0 ,
          0, 0, 225  , 0 ,
          0, 0, 0    , 225 ;

    // State transition matrix - without [dt]
    F_ = MatrixXd(4, 4);
    F_ << 1, 0, 0 , 0 ,
          0, 1, 0 , 0 ,
          0, 0, 1 , 0 ,
          0, 0, 0 , 1 ;

    // process noise
    Q_ = MatrixXd(4, 4);

    // a 4x4 identity matrix
    I_ = MatrixXd::Identity( 4 , 4  ) ;


    prev_dt = -1.0  ;



}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict( double dt ) {



}

