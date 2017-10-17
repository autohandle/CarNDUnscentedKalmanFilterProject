#include "ukf.h"
#include "tools.h"
#include "measurement_package.h"
#include "Eigen/Dense"
#include <iostream>
#include <stdexcept>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  n_x_=5;// size f the state vector
  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

}

UKF::UKF(const int n_aug) : UKF() {
  n_aug_=n_aug;// initialize size of augmented states
  
  lambda_ = calculateLambda(n_aug_);
  weights_ = calculateWeights(lambda(), n_aug_);
}


UKF::~UKF() {}

void UKF::initialize(MeasurementPackage theMeasurementPackage) {
  if (!is_initialized_) {
    /**
     TODO:
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    
    //previous_timestamp_= theMeasurementPackage.timestamp_; // initial dt == 0.
    
    // first measurement
    
    assert(n_x()>0);
    x_=Eigen::VectorXd(n_x());
    x_.setOnes();
    
    is_initialized_ = true;
    if (Tools::TESTING) {
      std::cout << "UKF::initialize:" << toString() << std::endl;
    }
  }
}

void UKF::updateX(const VectorXd &theMeasurement) {
  
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage theMeasurementPackage) {
  initialize(theMeasurementPackage);



  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

std::string UKF::toString() {
  std::ostringstream oss;
  oss << "UKF::toString:" << std::endl
  << "x:" <<Tools::toString(x_) << std::endl
  << "P:" <<Tools::toString(P_) << std::endl;
  return oss.str();
}

VectorXd UKF::calculateWeights(const int lambda, const int n_aug) {
  assert (lambda>-10 && lambda < 10);
  assert (n_aug>0 && n_aug < 20);
  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = double(lambda)/double(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {
    double weight = 0.5/double(n_aug+lambda);
    weights(i) = weight;
  }
  return weights;
}

int UKF::calculateLambda(const int n_aug) {
  assert(n_aug>0 && n_aug<20);
  return 3 - n_aug;
}


void UKF::predictRadarMeasurement(VectorXd& zMeasurment, MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out) {

  //set state dimension
  int n_x = 5;
  
  //set augmented dimension
  int n_aug = 7;
  
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  //define spreading parameter
  double lambda = 3 - n_aug;
  
  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }
  
  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;
  
  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;
  
  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;
  
  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
  5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
  1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
  2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
  0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
  0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  
  /*******************************************************************************
   * Student part begin
   ******************************************************************************/
  
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    
    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);
    
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    
    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    S = S + weights(i) * z_diff * z_diff.transpose();
  }
  
  //add measurement noise covariance matrix
  //MatrixXd R = MatrixXd(n_z,n_z);
  //R <<    std_radr*std_radr, 0, 0,
  //0, std_radphi*std_radphi, 0,
  //0, 0,std_radrd*std_radrd;
  //S = S + R;
  
  
  /*******************************************************************************
   * Student part end
   ******************************************************************************/
  
  //print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;
  
  //write result
  *z_out = z_pred;
  *S_out = S;
}

RadarFilter::RadarFilter() : UKF(7) {
  assert(n_aug()==7);
  //set augmented dimension
  // initial covariance matrix
  R_ = MatrixXd(3, 3);
  R_ <<   std_radr_*std_radr_,  0,                        0,
          0,                    std_radphi_*std_radphi_,  0,
          0,                    0,                        std_radrd_*std_radrd_;
  assert(n_x()>0);
}

void RadarFilter::initialize(MeasurementPackage theRadarMeasurementPackage) {
  UKF::initialize(theRadarMeasurementPackage);
  updateX(theRadarMeasurementPackage.raw_measurements_);
}

void RadarFilter::updateX(const VectorXd &theRadarMeasurementPackage) {
}

void RadarFilter::predictRadarMeasurement(VectorXd& zMeasurement, MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out) {
  
  int n_z =zMeasurement.size();
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug() + 1);
  
  /*******************************************************************************
   * Student part begin
   ******************************************************************************/
  
  //transform sigma points into measurement space
  assert(xSigPredicted.rows()==4 && xSigPredicted.cols()==2 * n_aug() + 1);
  for (int i = 0; i < 2 * n_aug() + 1; i++) {  //2n+1 simga points
    
    // extract values for better readibility
    double p_x = xSigPredicted(0,i);
    double p_y = xSigPredicted(1,i);
    double v  = xSigPredicted(2,i);
    double yaw = xSigPredicted(3,i);
    
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    
    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
  
  //mean predicted measurement
  VectorXd zPredicted = VectorXd(n_z);
  zPredicted.fill(0.0);
  for (int i=0; i < 2*n_aug()+1; i++) {
    zPredicted = zPredicted + weights(i) * Zsig.col(i);
  }
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug() + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - zPredicted;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    S = S + weights(i) * z_diff * z_diff.transpose();
  }
  
  //add measurement noise covariance matrix
  //MatrixXd R = MatrixXd(n_z,n_z);
  //R <<    std_radr*std_radr, 0, 0,
  //0, std_radphi*std_radphi, 0,
  //0, 0,std_radrd*std_radrd;
  S = S + R();
  
  std::cout << "zPredicted: " << std::endl << zPredicted << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;
  
  //write result
  *z_out = zPredicted;
  *S_out = S;
  
}

LidarFilter::LidarFilter() : UKF(3) {
  assert(n_aug()==3);
}

void LidarFilter::initialize(MeasurementPackage theLidarMeasurementPackage) {
  UKF::initialize(theLidarMeasurementPackage);
  updateX(theLidarMeasurementPackage.raw_measurements_);
}

void LidarFilter::updateX(const VectorXd &theLidarMeasurementPackage) {
}

void LidarFilter::predictRadarMeasurement(VectorXd& zMeasurment, MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out) {
}

UKFProcessor::UKFProcessor() : radarFilter(RadarFilter()), lidarFilter(LidarFilter()) {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;
  
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
}

UKFProcessor::~UKFProcessor() {}

void UKFProcessor::ProcessMeasurement(MeasurementPackage theMeasurementPackage) {

  UKF& filter = useRadar()?((UKF&)radarFilter):((UKF&)lidarFilter);
  
  switch(theMeasurementPackage.sensor_type_) {
    case MeasurementPackage::RADAR :
      std::cout << "UKFProcessor::RADAR" << std::endl;
      if (useRadar()) {
        filter=radarFilter;
      }
      break;
    case MeasurementPackage::LASER :
      std::cout << "UKFProcessor::LIDAR" << std::endl;
      filter=lidarFilter;
      if (useLidar()) {
        filter=lidarFilter;
      }
      break;
    default :
      throw std::invalid_argument("");
  }
  filter.ProcessMeasurement(theMeasurementPackage);
}

VectorXd& UKFProcessor::x() {
  return lidarFilter.x_;
}

MatrixXd& UKFProcessor::P() {
  return lidarFilter.P_;
}
