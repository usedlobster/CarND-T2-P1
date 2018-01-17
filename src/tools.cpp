#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    // start with 0
    VectorXd rmse(4);
    rmse << 0,0,0,0 ;

    // check we have same estimate to ground
    if ( estimations.size() != ground_truth.size()  )
        std::cerr << "Estimation / Groud Truth Size Mismatch " ;
    else if ( estimations.size() < 1 )
        std::cerr  << "No Estimations " ;
    else {

        VectorXd xx(4) ; // to hold sum of squares element
        VectorXd xd    ;
        xx << 0,0,0,0  ;
        for(int i=0; i < estimations.size(); ++i) {
            xd =  estimations[i] - ground_truth[i] ;
            xx = xx + (VectorXd)xd.array().square() ;
        }


        rmse = xx / estimations.size()  ;
        // take sqrt element b7y element
        rmse = (VectorXd)rmse.array().sqrt() ;
    }

    return rmse ;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj( 3, 4 ) ;

    const float px = x_state(0) ;
    const float py = x_state(1) ;
    const float vx = x_state(2) ;
    const float vy = x_state(3) ;

    // we have already unpacked the current state vector
    // so we might as well calculate the jacaobian matrix here

    float d1 = ( px*px ) + ( py*py ) ;
    if ( d1 < 1e-5 )
        d1 = 1e-5 ;
    float d2  = sqrt( d1 )  ;
    float d3  = d2*d1      ;

    //

    Hj << ( px / d2  ), ( py / d2  ), 0, 0,
    -( py / d1 ), ( px / d1 ), 0, 0,
    py*(vx*py - vy*px ) / d3, px*( vy*px - vx*py ) / d3, px / d2, py/ d2 ;

    return Hj ;

}
