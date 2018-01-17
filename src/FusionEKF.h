#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
  public:
    /**
    * Constructor.
    */
    FusionEKF();

    /**
    * Destructor.
    */
    virtual ~FusionEKF();

    /**
    * Run the whole flow of the Kalman Filter from here.
    */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /*
    * Kalman Filter update and prediction math lives in here
    */
    KalmanFilter ekf_;

  private:

    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;

    // tool object used to compute  RMSE
    Tools tools;
};

#endif /* FusionEKF_H_ */
