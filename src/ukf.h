#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {

private:
  
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;
  bool is_initialized() {
    return is_initialized_;
  }
  
  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;
  VectorXd& x() {
    return x_;
  }

  ///* state covariance matrix
  MatrixXd P_;
  MatrixXd& P() {
    return P_;
  }

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;
  MatrixXd& Xsig_pred() {
    return Xsig_pred_;
  }

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;
  VectorXd& weights() {
    return weights_;
  }
  double weights(int i) {
    return weights()(i);
  }

  ///* State dimension
  int n_x_;
  int n_x() {
    return n_x_;
  }

  ///* Augmented state dimension
  int n_aug_;
  int n_aug() {
    return n_aug_;
  }

  ///* Sigma point spreading parameter
  double lambda_;
  double lambda() {
    return lambda_;
  }
  
  //add measurement noise covariance matrix
  MatrixXd R_;
  MatrixXd& R() {
    return R_;
  }

  /**
   * Constructor
   */
  UKF();
  UKF(const int n_aug);

  /**
   * Destructor
   */
  virtual ~UKF();

  virtual void initialize(MeasurementPackage theMeasurementPackage);
  virtual void updateX(const VectorXd &theMeasurement);

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
  
  std::string toString();
  
private:
  
protected:
  static VectorXd calculateWeights(const int lambda, const int n_aug);
  static int calculateLambda(const int n_aug);
  virtual void predictRadarMeasurement(VectorXd& zMeasurment, MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out);

};

class RadarFilter: public UKF {
public:
  
  /**
   * Constructor
   */
  RadarFilter();
  
  void initialize(MeasurementPackage theRadarMeasurementPackage);
  void updateX(const VectorXd &theRadarMeasurement);
  void predictRadarMeasurement(VectorXd& zMeasurment, MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out);
  
};

class LidarFilter: public UKF {
public:
  
  /**
   * Constructor
   */
  LidarFilter();
  
  void initialize(MeasurementPackage theLidarMeasurementPackage);
  void updateX(const VectorXd &theLidarMeasurement);
  void predictRadarMeasurement(VectorXd& zMeasurment, MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out);

};

class UKFProcessor {
  
private:
  RadarFilter radarFilter;
  LidarFilter lidarFilter;
  
  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  
  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

public:
  /**
   * Constructor
   */
  UKFProcessor();
  
  /**
   * Destructor
   */
  virtual ~UKFProcessor();
  
  void ProcessMeasurement(MeasurementPackage theMeasurementPackage);
  VectorXd& x();
  MatrixXd& P();
  
  bool useLidar() {
    return use_laser_;
  }
  
  bool useRadar() {
    return use_radar_;
  }
  
};

#endif /* UKF_H */
