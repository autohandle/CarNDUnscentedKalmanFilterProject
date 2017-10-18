#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanState {
  
private:
  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;
  
  ///* state covariance matrix
  MatrixXd P_;
  
public:
  
  /**
   * Constructor
   */

  KalmanState(Eigen::VectorXd theStateVector, Eigen::MatrixXd theCovarianceMatrix);
  KalmanState(int numberOfStates);
  
  /**
   * Destructor
   */
  virtual ~KalmanState();
  
  VectorXd& x() {
    return x_;
  }
  
  MatrixXd& P() {
    return P_;
  }
  
  int n_x() {
    return x().size();
  }

};

class UKF {

private:
  
  KalmanState kalmanState;
  MatrixXd R_;
  int n_aug_;
  
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;
  bool is_initialized() {
    return is_initialized_;
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
  int n_x() {
    return kalmanState.n_x();
  }

  ///* Augmented state dimension
  int n_aug() {
    return n_aug_;
  }

  ///* Sigma point spreading parameter
  double lambda_;
  double lambda() {
    return lambda_;
  }
  
  //add measurement noise covariance matrix
  MatrixXd& R() {
    return R_;
  }

  /**
   * Constructor
   */
  UKF(const int n_aug, KalmanState theKalmanState, Eigen::MatrixXd theMeasurementNoiseCovariance);

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
  static Eigen::MatrixXd newR(int numberOfArguments, ...);
  
  VectorXd& x() {
    return kalmanState.x();
  }
  
  MatrixXd& P() {
    return kalmanState.P();
  }
  
  static void testGenerateSigmaPoints();
  static void testAugmentedSigmaPoints();

  
private:
  
protected:
  static VectorXd calculateWeights(const int lambda, const int n_aug);
  static int calculateLambda(const int n_aug);
  static double normalizeAngle(const double theAngle);
  virtual void generateSigmaPoints(MatrixXd* Xsig_out);
  virtual void augmentedSigmaPoints(MatrixXd* Xsig_out);
  virtual void predictZMeasurement(MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out);
  virtual void predictMeanAndCovariance(MatrixXd& xSigPredicted, VectorXd* x_out, MatrixXd* P_out);
  virtual void sigmaPointPrediction(double deltaT, MatrixXd& xSigAug, MatrixXd* Xsig_out);
};

class RadarFilter: public UKF {
public:
  
  /**
   * Constructor
   */
  RadarFilter(int theNumberofAugmentedStates, KalmanState theKalmanState, Eigen::MatrixXd theMeasurementNoiseCovariance);
  
  void initialize(MeasurementPackage theRadarMeasurementPackage);
  void updateX(const VectorXd &theRadarMeasurement);
  void predictZMeasurement(MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out);
  void predictMeanAndCovariance(MatrixXd& xSigPredicted, VectorXd* x_out, MatrixXd* P_out);
  void sigmaPointPrediction(double deltaT, MatrixXd& xSigAug, MatrixXd* Xsig_out);

  static void testPredictZMeasurement();
  static void testPredictMeanAndCovariance();
  static void testSigmaPointPrediction();
  static void testGenerateSigmaPoints();
  
};

class LidarFilter: public UKF {
public:
  
  /**
   * Constructor
   */
  LidarFilter(int theNumberofAugmentedStates, KalmanState theKalmanState, Eigen::MatrixXd theMeasurementNoiseCovariance);
  
  void initialize(MeasurementPackage theLidarMeasurementPackage);
  void updateX(const VectorXd &theLidarMeasurement);
  void predictZMeasurement(MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out);


};

class UKFProcessor {
  
private:
  
  KalmanState kalmanState;
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
  UKFProcessor(int theNumberOfStates, int theNumberOfAugmentedStates, MatrixXd& theRadarR, MatrixXd& theLidarR);
  
  /**
   * Destructor
   */
  virtual ~UKFProcessor();
  
  void ProcessMeasurement(MeasurementPackage theMeasurementPackage);
  
  bool useLidar() {
    return use_laser_;
  }
  
  bool useRadar() {
    return use_radar_;
  }
  
  VectorXd& x() {
    return kalmanState.x();
  }
  
  MatrixXd& P() {
    return kalmanState.P();
  }
  
  static void testRadar();
  
};

#endif /* UKF_H */
