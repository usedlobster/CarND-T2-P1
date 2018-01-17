#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {


    // Lidar observation model

    H_lidar_ = MatrixXd( 2, 4 ) ;
    H_lidar_ << 1, 0, 0, 0,
             0, 1, 0, 0 ;


    // radar observation model
    // to be computed at each radar update

    H_radar_  = MatrixXd( 3, 4 ) ;
    H_radar_ << 0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0 ;

    // lidar observation noise fixed - by sensor type
    R_lidar_ = MatrixXd( 2, 2 );
    R_lidar_ << 0.0225, 0.0000,
             0.0000, 0.0225 ;

    // radar observation noise fixed - by sensor type
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0.0000, 0.00,
             0.00, 0.0009, 0.00,
             0.00, 0.0000, 0.09 ;

    // P = error covariance matrix
    P_ = MatrixXd( 4, 4 ) ;
    P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 225, 0,
    0, 0, 0, 225 ;

    // State transition matrix - without [dt]
    F_ = MatrixXd( 4, 4 );
    F_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1 ;

    // process noise
    Q_ = MatrixXd( 4, 4 );

    // setup a 4x4 identity matrix
    I_ = MatrixXd::Identity( 4, 4 ) ;

    //
    prev_dt = -1.0  ;



}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict( float dt ) {

    // the compiler will optimize these out
    // with any luck
    const float noise_ax = 9.0 ;
    const float noise_ay = 9.0 ;

    // quick and dirty , can't hurt
    // when dt - hasnt changed we don't need to recompute  F_ , Q_  matrix values
    if ( prev_dt != dt ) {


        prev_dt =  dt ;

        // setup F_
        F_( 0, 2 ) = dt ;
        F_( 1, 3 ) = dt ;
        //
        float dt2 =   dt*dt        ; // dt^2
        float dt3 =  (dt2*dt)/2.0  ; // => (dt^3)/2
        float dt4 =   dt3*(dt/2.0) ; // => (dt^4)/4


        // calculate Q_ based on dt
        Q_ <<   noise_ax * dt4, 0, noise_ax * dt3, 0,
        0, noise_ay * dt4, 0, noise_ay * dt3,
        noise_ax * dt3, 0, noise_ax * dt2, 0,
        0, noise_ay * dt3, 0, noise_ay * dt2 ;
    }

    // predict next state from previous
    x_ = F_ * x_;
    //  new covariance matrix ,
    P_ = F_ * P_ * ( F_.transpose()) + Q_;



}

// standard kalman filter update
void KalmanFilter::UpdateLIDAR(const VectorXd &z) {
    // calculate optimal? Kalman Gain - it
    MatrixXd PHt = P_ * H_lidar_.transpose() ;
    MatrixXd S = H_lidar_ *  PHt + R_lidar_  ;
    MatrixXd K = PHt * S.inverse();
    // calculate measurement residual
    VectorXd y = z - H_lidar_ * x_;
    // update new state based on current measurement.
    x_ = x_ + ( K * y ) ;
    // update estimate covaraince matrix
    P_ = ( I_ - K * H_lidar_ ) * P_;
}


// extended kalman filter update
void KalmanFilter::UpdateRADAR(const VectorXd &z) {

    // extract from x_ px,py,vx,vy variables -

    const float px = x_(0) ;
    const float py = x_(1) ;
    const float vx = x_(2) ;
    const float vy = x_(3) ;

    // we have already unpacked the current state vector x_
    // so we might as well calculate the jacaobian matrix here
    //

    float r1 = ( px*px ) + ( py * py ) ;
    if ( r1 < 1.0e-5 )
        r1 = 1.0e-5 ;

    float rho = sqrt( r1 )             ; // (px^2+py^2)^0.5
    float r3 = r1 * rho                ; // (px^2+py^2)^1.5
    float phi = atan2( py, px )       ;
    float rho_dot = ( px*vx + py*vy ) / rho ;

    float a, b ;  // values used twice

    a = px / rho ;
    b = py / rho ;

    H_radar_ <<    a, b, 0, 0,
             -py/r1, px/r1, 0, 0,
             py*(vx*py-vy*px) / r3, px*(vy*px - vx*py)/r3, a, b ;


    VectorXd h = VectorXd(3) ;
    h << rho, phi, rho_dot ;


    // y is residual between current ( h ) and new measurement ( z )
    VectorXd y = z - h ;


    // make sure y[1] is between [-pi,+pi]

    // this works , but must be slow !
    //y[1] = atan2( sin(y[1]) , cos(y[1]) ) ;


    while ( y[1] > M_PI )
        y[1] -= M_PI*2.0 ;

    while ( y[1] < -M_PI )
        y[1] += M_PI*2.0 ;

    // calculate kalman gain K
    MatrixXd PHt = P_ * H_radar_.transpose() ;
    MatrixXd S = H_radar_ *  PHt + R_radar_  ;
    MatrixXd K = PHt * S.inverse();

    // new state estimate
    x_ = x_ + ( K * y );
    // new
    P_ = ( I_ - K * H_radar_ ) * P_;

}

