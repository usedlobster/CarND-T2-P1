#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {


    is_initialized_ = false ;
    previous_timestamp_ = 0 ;



}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // first measurement
        cout << "EKF: " << endl;

        // setup
        ekf_.x_ = VectorXd(4)  ;
        ekf_.x_ << 0 , 0, 0, 0 ;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

            // get radar measurements
            float rho     = measurement_pack.raw_measurements_[0] ;
            float phi     = measurement_pack.raw_measurements_[1] ;
            float rho_dot = measurement_pack.raw_measurements_[2] ;
            // now convert from polar -> cartesian
            float px =  rho * cos( phi)  ;
            float py =  rho * sin( phi ) ;

            // set current state - we could use rho_dot * cos( phi ) ...
            ekf_.x_ << px, py, 0 , 0 ;


        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

            // laser measurements from lidar only have x,y - data
            float px = measurement_pack.raw_measurements_[0] ;
            float py = measurement_pack.raw_measurements_[1] ;

            // we don't know velocity , we can
            ekf_.x_ << px, py, 0.5 , 0.5  ;

        }


        // remember timestamp

        previous_timestamp_ = measurement_pack.timestamp_ ;

        // done initializing, no need to predict or update anything
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/


    float dt = ( measurement_pack.timestamp_ - previous_timestamp_ ) / 1.0e6 ;
    previous_timestamp_ = measurement_pack.timestamp_ ;


    ekf_.Predict( dt ) ;

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.updateRADAR() ;
    } else {
        ekf_.updateLIDAR() ;
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
