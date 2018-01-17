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

    // measurement matrix for lidar/radar
    Eigen::MatrixXd H_lidar_, H_radar_  ;

    // measurement covariance matrix for lidar/radar
    Eigen::MatrixXd R_lidar_, R_radar_ ;

    // identity matrix ( 4 x 4 )
    Eigen::MatrixXd I_ ;

    // for caching some matrices
    float prev_dt ;

    // C'tor
    KalmanFilter();
    // D'tor
    virtual ~KalmanFilter();


    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     *
     * @param dt Time between k and k+1 sensor readings.
     */
    void Predict( float dt )  ;


    /**
     * Updates the state by using standard Kalman Filter equations
     *
     * @param z The measurement at k+1 from LIDAR sensor
     */
    void UpdateLIDAR(const Eigen::VectorXd &z);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1 from RADAR sensor
     */
    void UpdateRADAR(const Eigen::VectorXd &z);


};

#endif /* KALMAN_FILTER_H_ */
