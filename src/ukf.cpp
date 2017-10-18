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

KalmanState::KalmanState(Eigen::VectorXd theStateVector, Eigen::MatrixXd theCovarianceMatrix) :
  x_(theStateVector), P_(theCovarianceMatrix) {
  assert(x().size()==P().rows());
}

KalmanState::KalmanState(int numberOfStates) :
KalmanState(Eigen::VectorXd(numberOfStates), Eigen::MatrixXd(numberOfStates,numberOfStates)) {
  
  x().setOnes(); // initialize state vector
  P().Identity(numberOfStates,numberOfStates); // initialize covariance
  
};

KalmanState::~KalmanState() {}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(const int theNumberOfStatesAugmented, KalmanState theKalmanState, Eigen::MatrixXd theMeasurementNoiseCovariance) :
  kalmanState(theKalmanState), R_(theMeasurementNoiseCovariance), n_aug_(theNumberOfStatesAugmented) {

  //n_x_=5;// size f the state vector

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

  //n_aug_=theNumberOfStatesAugmented;// initialize size of augmented states
  R_=theMeasurementNoiseCovariance;
  
  lambda_ = calculateLambda(n_aug());
  assert(lambda() < n_x());
  weights_ = calculateWeights(lambda(), n_aug());
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
  << "x:" <<Tools::toString(x()) << std::endl
  << "P:" <<Tools::toString(P()) << std::endl;
  return oss.str();
}

Eigen::MatrixXd UKF::newR(int numberOfSigmas, ...) {
  
  va_list listOfSigmas;
  
  Eigen::MatrixXd R = Eigen::MatrixXd(numberOfSigmas, numberOfSigmas);
  R.setZero();
  
  va_start ( listOfSigmas, numberOfSigmas );  // Initializing arguments to store all values after num
  for ( int rowcol = 0; rowcol < numberOfSigmas; rowcol++ ){
    double sigma=va_arg (listOfSigmas, double );
    R(rowcol,rowcol)=sigma*sigma;
    va_end ( listOfSigmas );
  }
  return R;
}


VectorXd UKF::calculateWeights(const int lambda, const int n_aug) {
  assert (lambda>-10 && lambda < 10);
  assert (n_aug>=0 && n_aug < 20);
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
  assert(n_aug>=0 && n_aug<20);
  return 3 - n_aug;
}

double normalizeAngle(const double theAngle) {
  double normalizedAngle=theAngle;
  while (normalizedAngle> M_PI) normalizedAngle-=2.*M_PI;
  while (normalizedAngle<-M_PI) normalizedAngle+=2.*M_PI;
  return normalizedAngle;
}

void UKF::generateSigmaPoints(MatrixXd* Xsig_out) {

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x(), 2 * n_x() + 1);
  
  //calculate square root of P
  MatrixXd A = P().llt().matrixL();
   
  /*******************************************************************************
   * Student part begin
   ******************************************************************************/
  
  //set first column of sigma point matrix
  Xsig.col(0)  = x();
  
  //set remaining sigma points
  for (int i = 0; i < n_x(); i++)
  {
    Xsig.col(i+1)     = x() + sqrt(lambda()+n_x()) * A.col(i);
    Xsig.col(i+1+n_x()) = x() - sqrt(lambda()+n_x()) * A.col(i);
  }
  
  /*******************************************************************************
   * Student part end
   ******************************************************************************/
  
  //print result
  std::cout << "UKF::generateSigmaPoints-Xsig = " << std::endl << Xsig << std::endl;
  
  //write result
  *Xsig_out = Xsig;
  
  /* expected result:
   Xsig =
   5.7441  5.85768   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441
   1.38  1.34566  1.52806     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38
   2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049
   0.5015  0.44339 0.631886 0.516923 0.595227   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015
   0.3528 0.299973 0.462123 0.376339  0.48417 0.418721 0.405627 0.243477 0.329261  0.22143 0.286879
   */
  
}

void UKF::testGenerateSigmaPoints() {
  //set state dimension
  int n_x = 5;
  
  //define spreading parameter
  //double lambda = 3 - n_x;
  
  //set augmented dimension
  //int n_aug = 7;
  
  //set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
  1.3800,
  2.2049,
  0.5015,
  0.3528;
  
  //set example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
  -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
  0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
  -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
  -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
  
  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;
  
  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;
  
  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;
  
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  KalmanState kalmanState = KalmanState(x,P);
  RadarFilter radarFiter = RadarFilter(n_x, kalmanState, UKF::newR(n_z, std_radr, std_radphi, std_radrd));
  assert(radarFiter.n_x()==n_x);
  //assert(radarFiter.n_aug()==n_aug);
  
  MatrixXd xSigOut = MatrixXd(n_x, 2 * n_x + 1);
  
  radarFiter.generateSigmaPoints(&xSigOut);
  
  std::cout << "UKF::testGenerateSigmaPoints-xSigOut: " << std::endl << xSigOut << std::endl;
  
  MatrixXd xSig = MatrixXd(n_x, 2 * n_x + 1);
   xSig <<
   5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441,
   1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38,
   2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,
   0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015,
  0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879;
  
  assert(Tools::areSame(xSigOut, xSig));

}

void UKF::augmentedSigmaPoints(MatrixXd* Xsig_out) {
 
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug(), 2 * n_aug() + 1);
  
  /*******************************************************************************
   * Student part begin
   ******************************************************************************/
  
  //create augmented mean state
  x_aug.head(5) = x();
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P();
  //P_aug(5,5) = std_a*std_a;
  //P_aug(6,6) = std_yawdd*std_yawdd;
  
  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug(); i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda()+n_aug()) * L.col(i);
    Xsig_aug.col(i+1+n_aug()) = x_aug - sqrt(lambda()+n_aug()) * L.col(i);
  }
  
  /*******************************************************************************
   * Student part end
   ******************************************************************************/
  
  //print result
  std::cout << "UKF::augmentedSigmaPoint-Xsig_aug = " << std::endl << Xsig_aug << std::endl;
  
  //write result
  *Xsig_out = Xsig_aug;
  
  /* expected result:
   Xsig_aug =
   5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
   1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
   2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
   0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
   0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
   0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
   0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641
   */
  
}

void testAugmentedSigmaPoints() {
  
  //set state dimension
  int n_x = 5;
  
  //set augmented dimension
  int n_aug = 7;
  
  //Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;
  
  //Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;
  
  //define spreading parameter
  double lambda = 3 - n_aug;
  
  //set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
  1.3800,
  2.2049,
  0.5015,
  0.3528;
  
  //create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
  -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
  0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
  -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
  -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
  
  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;
  
  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;
  
  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;
  
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  KalmanState kalmanState = KalmanState(n_x);
  RadarFilter radarFiter = RadarFilter(n_aug, kalmanState, UKF::newR(n_z, std_radr, std_radphi, std_radrd));
  assert(radarFiter.n_x()==n_x);
  assert(radarFiter.n_aug()==n_aug);
}

void UKF::predictZMeasurement(MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out) {
  throw std::logic_error("Not Implemented");
}

void UKF::predictMeanAndCovariance(MatrixXd& xSigPredicted, VectorXd* x_out, MatrixXd* P_out) {
  throw std::logic_error("Not Implemented");
}

void UKF::sigmaPointPrediction(double deltaT, MatrixXd& xSigAug, MatrixXd* Xsig_out) {
  throw std::logic_error("Not Implemented");
}

RadarFilter::RadarFilter(int theNumberofAugmentedStates, KalmanState theKalmanState, Eigen::MatrixXd theMeasurementNoiseCovariance) :
    UKF(theNumberofAugmentedStates, theKalmanState, theMeasurementNoiseCovariance) {
  assert(n_aug()==theNumberofAugmentedStates);
  //set augmented dimension
  // initial covariance matrix
  //R_ = UKF::newR(3, std_radr_, std_radphi_, std_radrd_);
  //R_ <<   std_radr_*std_radr_,  0,                        0,
  //        0,                    std_radphi_*std_radphi_,  0,
  //        0,                    0,                        std_radrd_*std_radrd_;
  assert(n_x()>0);
}

void RadarFilter::initialize(MeasurementPackage theRadarMeasurementPackage) {
  UKF::initialize(theRadarMeasurementPackage);
  updateX(theRadarMeasurementPackage.raw_measurements_);
}

void RadarFilter::updateX(const VectorXd &theRadarMeasurementPackage) {
}

void RadarFilter::sigmaPointPrediction(double deltaT, MatrixXd& xSigAug, MatrixXd* Xsig_out) {
  
  //create matrix with predicted sigma points as columns
  MatrixXd xSigPredicted = MatrixXd(n_x(), 2 * n_aug() + 1);

  //predict sigma points
  for (int i = 0; i< 2*n_aug()+1; i++)
  {
    //extract values for better readability
    double p_x = xSigAug(0,i);
    double p_y = xSigAug(1,i);
    double v = xSigAug(2,i);
    double yaw = xSigAug(3,i);
    double yawd = xSigAug(4,i);
    double nu_a = xSigAug(5,i);
    double nu_yawdd = xSigAug(6,i);
    
    //predicted state values
    double px_p, py_p;
    
    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*deltaT) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*deltaT) );
    }
    else {
      px_p = p_x + v*deltaT*cos(yaw);
      py_p = p_y + v*deltaT*sin(yaw);
    }
    
    double v_p = v;
    double yaw_p = yaw + yawd*deltaT;
    double yawd_p = yawd;
    
    //add noise
    px_p = px_p + 0.5*nu_a*deltaT*deltaT * cos(yaw);
    py_p = py_p + 0.5*nu_a*deltaT*deltaT * sin(yaw);
    v_p = v_p + nu_a*deltaT;
    
    yaw_p = yaw_p + 0.5*nu_yawdd*deltaT*deltaT;
    yawd_p = yawd_p + nu_yawdd*deltaT;
    
    
    //write predicted sigma point into right column
    xSigPredicted(0,i) = px_p;
    xSigPredicted(1,i) = py_p;
    xSigPredicted(2,i) = v_p;
    xSigPredicted(3,i) = yaw_p;
    xSigPredicted(4,i) = yawd_p;
  }
  
  /*******************************************************************************
   * Student part end
   ******************************************************************************/
  
  //print result
  std::cout << "RadarFilter::sigmaPointPrediction-xSigPredicted = " << std::endl << xSigPredicted << std::endl;
  
  //write result
  *Xsig_out = xSigPredicted;
}

void RadarFilter::testGenerateSigmaPoints() {
  UKF::testGenerateSigmaPoints();
}

void RadarFilter::testSigmaPointPrediction() {
  
  //set state dimension
  int n_x = 5;
  
  //set augmented dimension
  int n_aug = 7;
  
  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
  5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
  1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
  2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
  0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
  0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
  0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
  0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;
  
  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;
  
  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;
  
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  KalmanState kalmanState = KalmanState(n_x);
  RadarFilter radarFiter = RadarFilter(n_aug, kalmanState, UKF::newR(n_z, std_radr, std_radphi, std_radrd));
  assert(radarFiter.n_x()==n_x);
  assert(radarFiter.n_aug()==n_aug);
  
  MatrixXd xSigOut = MatrixXd(n_x, 2 * n_aug + 1);
  double deltaT=0.1;
  
  radarFiter.sigmaPointPrediction(deltaT, Xsig_aug, &xSigOut);
  
  std::cout << "RadarFilter::testSigmaPointPrediction-xSigOut: " << std::endl << xSigOut << std::endl;

  MatrixXd xSigPred = MatrixXd(n_x, 2 * n_aug + 1);
  xSigPred <<
  5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553,
  1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939,
  2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049,
  0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
  0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;
  
  assert(Tools::areSame(xSigOut, xSigPred));
}

void RadarFilter::predictZMeasurement(MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out) {
  
  const int n_z =3; // radar: r, phi, r_dot
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug() + 1);
  
  /*******************************************************************************
   * Student part begin
   ******************************************************************************/
  
  //transform sigma points into measurement space
  assert(xSigPredicted.rows()==n_x() && xSigPredicted.cols()==2 * n_aug() + 1);
  for (int i = 0; i < 2 * n_aug() + 1; i++) {  //2n+1 simga points
    
    // extract values for better readibility
    const double p_x = xSigPredicted(0,i);
    const double p_y = xSigPredicted(1,i);
    const double v  = xSigPredicted(2,i);
    const double yaw = xSigPredicted(3,i);
    
    const double v1 = cos(yaw)*v;
    const double v2 = sin(yaw)*v;
    
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
  
  //std::cout << "zPredicted: " << std::endl << zPredicted << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;
  
  //write result
  *z_out = zPredicted;
  *S_out = S;
  
}

void RadarFilter::testPredictZMeasurement() {
  //set state dimension
  int n_x = 5;
  
  //set augmented dimension
  int n_aug = 7;
  
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  //define spreading parameter
  //double lambda = 3 - n_aug;
  
  //set vector for weights
  //VectorXd weights = VectorXd(2*n_aug+1);
  //double weight_0 = lambda/(lambda+n_aug);
  //weights(0) = weight_0;
  //for (int i=1; i<2*n_aug+1; i++) {
  //  double weight = 0.5/(n_aug+lambda);
  //  weights(i) = weight;
  //}

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
  
  KalmanState kalmanState = KalmanState(n_x);
  RadarFilter radarFiter = RadarFilter(n_aug, kalmanState, UKF::newR(n_z, std_radr, std_radphi, std_radrd));
  
  VectorXd z_out = VectorXd(3);
  MatrixXd S_out = MatrixXd(3, 3);
  
  radarFiter.predictZMeasurement(Xsig_pred, &z_out, &S_out);
  
  std::cout << "RadarFilter::testPredictZMeasurement-z_out: " << std::endl << z_out << std::endl;
  std::cout << "RadarFilter::testPredictZMeasurement-S_out: " << std::endl << S_out << std::endl;
  
  VectorXd z_pred = VectorXd(3);
  z_pred << 6.12155, 0.245993, 2.10313;
  
  MatrixXd S = MatrixXd(3, 3);
  S <<  0.0946171,    -0.000139448,  0.00407016,
        -0.000139448, 0.000617548,  -0.000770652,
        0.00407016,   -0.000770652,  0.0180917;
  
  assert(Tools::areSame(z_pred, z_out));
  assert(Tools::areSame(S, S_out));

}

void RadarFilter::predictMeanAndCovariance(MatrixXd& xSigPredicted, VectorXd* x_out, MatrixXd* P_out) {
  std::cout << "RadarFilter::predictMeanAndCovariance-weights: " << std::endl << weights() << std::endl;

  //create vector for predicted state
  VectorXd x = VectorXd(n_x());
  
  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x(), n_x());
  
  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug() + 1; i++) {  //iterate over sigma points
    x = x+ weights(i) * xSigPredicted.col(i);
  }
  
  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug() + 1; i++) {  //iterate over sigma points
    
    // state difference
    VectorXd x_diff = xSigPredicted.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }
  
  //std::cout << "x: " << std::endl << x << std::endl;
  //std::cout << "P: " << std::endl << P << std::endl;
  
  *x_out=x;
  *P_out=P;
  
}

void RadarFilter::testPredictMeanAndCovariance() {
  //set state dimension
  int n_x = 5;
  
  //set augmented dimension
  int n_aug = 7;
  
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
  5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
  1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
  2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
  0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
  0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  
  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;
  
  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;
  
  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;
  
  KalmanState kalmanState = KalmanState(n_x);
  RadarFilter radarFilter = RadarFilter(n_aug, kalmanState, UKF::newR(n_z, std_radr, std_radphi, std_radrd));
  assert(radarFilter.n_x()==n_x);
  assert(radarFilter.n_aug()==n_aug);
  
  VectorXd x_out = VectorXd(n_x);
  MatrixXd P_out = MatrixXd(n_x,n_x);
  
  radarFilter.predictMeanAndCovariance(Xsig_pred, &x_out, &P_out);
  std::cout << "RadarFilter::testPredictMeanAndCovariance-x_out: " << std::endl << x_out << std::endl;
  std::cout << "RadarFilter::testPredictMeanAndCovariance-P_out: " << std::endl << P_out << std::endl;
  
  VectorXd x = VectorXd(n_x);
  x << 5.93637, 1.49035, 2.20528, 0.536853, 0.353577;
  
  MatrixXd P = MatrixXd(n_x, n_x);
  P << 0.00543425, -0.0024053, 0.00341576, -0.00348196, -0.00299378,
      -0.0024053, 0.010845, 0.0014923, 0.00980182, 0.00791091,
      0.00341576, 0.0014923, 0.00580129, 0.000778632, 0.000792973,
      -0.00348196, 0.00980182, 0.000778632, 0.0119238, 0.0112491,
      -0.00299378, 0.00791091, 0.000792973, 0.0112491, 0.0126972;
  
  assert(Tools::areSame(x, x_out));
  assert(Tools::areSame(P, P_out));
  
}

LidarFilter::LidarFilter(int theNumberofAugmentedStates, KalmanState theKalmanState, Eigen::MatrixXd theMeasurementNoiseCovariance) :
    UKF(theNumberofAugmentedStates, theKalmanState,theMeasurementNoiseCovariance) {
  assert(n_aug()==7);
}

void LidarFilter::initialize(MeasurementPackage theLidarMeasurementPackage) {
  UKF::initialize(theLidarMeasurementPackage);
  updateX(theLidarMeasurementPackage.raw_measurements_);
}

void LidarFilter::updateX(const VectorXd &theLidarMeasurementPackage) {
}

void LidarFilter::predictZMeasurement(MatrixXd& xSigPredicted, VectorXd* z_out, MatrixXd* S_out) {
}

UKFProcessor::UKFProcessor(int theNumberOfStates, int theNumberOfAugmentedStates, MatrixXd& theRadarR, MatrixXd& theLidarR) :
  kalmanState(KalmanState(theNumberOfStates)),
  radarFilter(RadarFilter(theNumberOfAugmentedStates, kalmanState, theRadarR)),
  lidarFilter(LidarFilter(theNumberOfAugmentedStates, kalmanState, theLidarR)) {
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
        filter.ProcessMeasurement(theMeasurementPackage);
      }
      break;
    case MeasurementPackage::LASER :
      std::cout << "UKFProcessor::LIDAR" << std::endl;
      filter=lidarFilter;
      if (useLidar()) {
        filter=lidarFilter;
        filter.ProcessMeasurement(theMeasurementPackage);
      }
      break;
    default :
      throw std::invalid_argument("");
  }

}

void UKFProcessor::testRadar() {
  RadarFilter::testGenerateSigmaPoints();
  RadarFilter::testSigmaPointPrediction();
  RadarFilter::testPredictZMeasurement();
  RadarFilter::testPredictMeanAndCovariance();

}
