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
  
  const VectorXd& x() {// process state vector
    return x_;
  }
  
  const MatrixXd& P() {// process covariance
    return P_;
  }
  
  const int n_x() {// size of state vector
    return x().size();
  }

};

class UKF {

private:
  
  KalmanState kalmanState;
  MatrixXd R_;// noise covariance matrix
  MatrixXd Q_;// process covariance matrix
  int n_aug_;// number of augmented states
  int n_z_;// number of measurements from sensor
  
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
  const int n_x() {
    return kalmanState.n_x();
  }

  ///* Augmented state dimension
  const int n_aug() {
    return n_aug_;
  }

  ///* Sigma point spreading parameter
  double lambda_;
  double lambda() {
    return lambda_;
  }
  
  //measurement noise covariance matrix
  const MatrixXd& R() {
    return R_;
  }
  
  //process noise covariance matrix
  const MatrixXd& Q() {
    return Q_;
  }

  //process noise covariance matrix
  const int n_z() {
    return n_z_;
  }

  /**
   * Constructor
   */
  UKF(const int n_aug, const int n_z, KalmanState theKalmanState,
      Eigen::MatrixXd theMeasurementNoiseCovariance/*R*/, Eigen::MatrixXd theProcessNoiseCovariance/*Q*/);

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
  KalmanState Prediction(const double delta_t);
  KalmanState Update(const MeasurementPackage meas_package, const Eigen::MatrixXd xSigPredicted);
  KalmanState Update(const Eigen::VectorXd z, const Eigen::MatrixXd xSigPredicted);


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
  static Eigen::MatrixXd newCovariance(int numberOfArguments, ...);
  
  const VectorXd& x() {
    return kalmanState.x();
  }
  
  const MatrixXd& P() {
    return kalmanState.P();
  }
  
  static void testGenerateSigmaPoints();
  static void testAugmentedSigmaPoints();
  static void testUpdateState();
  static void testSigmaPointPrediction();
  static void testPredictZMeasurement();
  static void testPredictMeanAndCovariance();
  static void testPrediction();

private:
  
protected:
  static VectorXd calculateWeights(const int lambda, const int n_aug);
  static int calculateLambda(const int theSizeOfTheStateVector);// maybe the actual vector n_x or augmented n_aug
  
  static double normalizeAngle(const double theAngle);
  virtual VectorXd normalizeStateVector(const VectorXd& theStateVector);
  virtual VectorXd normalizeMeasurementVector(const VectorXd& theMeasurementVector);

  virtual MatrixXd generateSigmaPoints();
  virtual MatrixXd augmentSigmaPoints();
  virtual MatrixXd sigmaPointPrediction(const double deltaT, const MatrixXd& xSigAug);
  virtual KalmanState predictMeanAndCovariance(const MatrixXd& xSigPredicted);

  virtual void predictZMeasurement(const MatrixXd& Zsig, const VectorXd& zPredicted, VectorXd* z_out, MatrixXd* S_out);
  virtual VectorXd predictZ(const MatrixXd& Zsig);
  virtual MatrixXd transformSigmaPointsToMeasurements(const MatrixXd& xSigPredicted);
  virtual MatrixXd measurementCovarianceMatrix(const MatrixXd& Zsig, const VectorXd& zPredicted);
  virtual VectorXd extractZMeasurement(const MeasurementPackage theMeasurementPackage);
  virtual VectorXd transformStateToMeasurement(const VectorXd theState);

  virtual KalmanState updateState(const Eigen::MatrixXd Zsig, const Eigen::MatrixXd& z_pred, const Eigen::MatrixXd& Xsig_pred, const Eigen::MatrixXd& S, const Eigen::VectorXd& z);

};

class RadarFilter: public UKF {
public:
  
  /**
   * Constructor
   */
  RadarFilter(int theNumberofAugmentedStates, KalmanState theKalmanState,
              Eigen::MatrixXd theMeasurementNoiseCovariance/*R*/, Eigen::MatrixXd theProcessNoiseCovariance/*Q*/);
  
  void initialize(MeasurementPackage theRadarMeasurementPackage);
  void updateX(const VectorXd &theRadarMeasurement);
  VectorXd transformStateToMeasurement(const VectorXd theState);

  VectorXd normalizeStateVector(const VectorXd& theStateVector);
  VectorXd normalizeMeasurementVector(const VectorXd& theMeasurementVector);
  
};

class LidarFilter: public UKF {
public:
  
  /**
   * Constructor
   */
  LidarFilter(const int theNumberofAugmentedStates, KalmanState theKalmanState,
              Eigen::MatrixXd theMeasurementNoiseCovariance/*R*/, Eigen::MatrixXd theProcessNoiseCovariance/*Q*/);
  
  void initialize(MeasurementPackage theLidarMeasurementPackage);
  void updateX(const VectorXd &theLidarMeasurement);
  VectorXd transformStateToMeasurement(const VectorXd theState);

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
  UKFProcessor(int theNumberOfStates, int theNumberOfAugmentedStates,
               MatrixXd& theRadarR, MatrixXd& theLidarR, MatrixXd& theProcessNoiseQ);
  
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
  
  const VectorXd& x() {
    return kalmanState.x();
  }
  
  const MatrixXd& P() {
    return kalmanState.P();
  }
  
  static void testRadar();
  
};

#endif /* UKF_H */
