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
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);
	
	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
        0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

	/**
	TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
	*/

	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 0.05, 0,
               0, 1, 0, 0.05,
               0, 0, 1, 0,
			   0, 0, 0, 1;
	
	
	ekf_.Q_ = MatrixXd(4, 4);

	H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
			   0, 0, 0, 1000;
	
	
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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
	cout << "xinit ok" << endl;
	
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	  // x = range * cos(bearing), y = range *sin(bearing)
	  // velocities computed with rho-dot 
	    cout << "Kalman Filter Initialization with Radar" << endl;

	    double x = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
		double y = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
	    double vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
		double vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);		
		if (fabs(x+y)<0.0001)
		{
			x=0.0001;
			y=0.0001;
			cout << "object stuck in sensor";
		}
		// set the state with converted radar data
		ekf_.x_ << x, y, vx, vy;
		previous_timestamp_ = measurement_pack.timestamp_;	  
		cout << "Init:" << ekf_.x_ << endl;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		cout << "Kalman Filter Initialization with LASER" << endl;

		//set the state with the initial location and zero velocity
		ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

		previous_timestamp_ = measurement_pack.timestamp_;
		cout << "Init:" << ekf_.x_ << endl;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
	}
	
	/*****************************************************************************
	*  Prediction
	****************************************************************************/

	/**
	TODO:
	* Update the state transition matrix F according to the new elapsed time.
	 - Time is measured in seconds.
	 * Update the process noise covariance matrix.
	 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/
	cout << "Predicting" << endl;
	//derive new timestamp
	double dt= (measurement_pack.timestamp_-previous_timestamp_)/ 1000000.0;
	double dt_2 = dt * dt;
	double dt_3 = dt_2 * dt;
	double dt_4 = dt_3 * dt;
	
	previous_timestamp_=measurement_pack.timestamp_;
	//insert elapsed time into F
	ekf_.F_ << 1, 0, dt, 0,
               0, 1, 0, dt,
               0, 0, 1, 0,
			   0, 0, 0, 1;
	cout << "F" << endl;
	cout << ekf_.F_ << endl;

	//update Q
	double noise_ax = 9.0;
	double noise_ay = 9.0;
	cout << "Q" << endl;
    ekf_.Q_ << dt_4/4*noise_ax, 0, 					dt_3/2*noise_ax, 	0,
			   0, 				dt_4/4*noise_ay,	0, 					dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, 					dt_2*noise_ax, 		0,
			   0, 				dt_3/2*noise_ay, 	0, 					dt_2*noise_ay;
	cout << ekf_.Q_ << endl;
	ekf_.Predict();
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	cout << "Radar" << endl;
    // Radar updates
	ekf_.R_ = R_radar_;	
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	//recalculate Jacobian 
	VectorXd z_ = VectorXd(3);
    z_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2];
	ekf_.UpdateEKF(z_);

  } else {
    // Laser updates
	cout << "Laser";
    ekf_.R_ = R_laser_;
	ekf_.H_ = H_laser_;
	VectorXd z_ = VectorXd(2);
    z_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
	ekf_.Update(z_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
