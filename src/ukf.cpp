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

int KalmanState::id=0;

KalmanState::KalmanState(Eigen::VectorXd theStateVector, Eigen::MatrixXd theCovarianceMatrix) :
  x_(theStateVector), P_(theCovarianceMatrix), objectId(++id) {
  assert(x().size()==P().rows());
  assert(P().rows()==P().cols());
  if (Tools::TESTING) std::cout << "KalmanState::x&P constructor:" << toString() << std::endl;
}

KalmanState::KalmanState(int numberOfStates) :
KalmanState(Eigen::VectorXd(numberOfStates), Eigen::MatrixXd(numberOfStates,numberOfStates)) {
  
  x_.setOnes(); // initialize state vector
  P_.setIdentity(numberOfStates,numberOfStates); // initialize covariance

  if (Tools::TESTING) {
    P_ << 1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, .5, 0, 0,
    0, 0, 0, .01, 0,
    0, 0, 0, 0, .01;
    std::cout << "KalmanState::numberOfStates constructor:" << toString() << std::endl;
  }
};

std::string KalmanState::toString() {
  std::ostringstream oss;
  oss << "KalmanState::toString:" << std::endl
  << "x:" <<Tools::toString(x()) << std::endl
  << "P:" <<Tools::toString(P()) << std::endl;
  return oss.str();
}

void KalmanState::update(KalmanState theKalmanState) {
  assert(x().size()==theKalmanState.x().size());
  assert(P().cols()==theKalmanState.P().cols());
  assert(P().rows()==theKalmanState.P().rows());
  x_=theKalmanState.x();
  P_=theKalmanState.P();
  if (Tools::TESTING) std::cout << "KalmanState::update x:" << std::endl << x() << std::endl;
  if (Tools::TESTING) std::cout << "KalmanState::update P:" << std::endl << P() << std::endl;
}


KalmanState::~KalmanState() {}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(const int theNumberOfStatesAugmented,
         const int theNumberOfSensorMeasurements, MeasurementPackage::SensorType theSensorType,
         KalmanState& theKalmanState,
         Eigen::MatrixXd theMeasurementNoiseCovariance/*R*/, Eigen::MatrixXd theProcessNoiseCovariance/*Q*/) :
  kalmanState(theKalmanState), R_(theMeasurementNoiseCovariance), Q_(theProcessNoiseCovariance), n_aug_(theNumberOfStatesAugmented), n_z_(theNumberOfSensorMeasurements), sensorType(theSensorType){

  //n_x_=5;// size f the state vector

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  //std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  //std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  //std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  //std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  //std_radrd_ = 0.3;

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
  
  assert(sensorType>=MeasurementPackage::LASER && sensorType<=MeasurementPackage::RADAR);
  if (Tools::TESTING) std::cout << "UKF::constructor:" << toString() << std::endl;

}


UKF::~UKF() {}

void UKF::updateKalmanState(KalmanState& theKalmanState) {
  if (Tools::TESTING) std::cout << "UKF::updateKalmanState-current:" << (kalmanState.toString()) << std::endl;
  kalmanState.update(theKalmanState);
  if (Tools::TESTING) std::cout << "UKF::updateKalmanState-update:" << (kalmanState.toString()) << std::endl;

}

void UKF::initializeStateVector(const VectorXd &theMeasurement) {
  throw std::logic_error("Not Implemented");
}

VectorXd UKF::transformMeasurementToState(const VectorXd& theMeasurement) {
  throw std::logic_error("Not Implemented");
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
double UKF::ProcessMeasurement(const MeasurementPackage& theMeasurementPackage, const double theDeltaT) {

  MatrixXd sigmaPointsPrediction = Prediction(theDeltaT);
  double nis = Update(theMeasurementPackage, sigmaPointsPrediction);// updates KalmanState
  return nis;
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
MatrixXd UKF::Prediction(double deltaT) {// updates KalmanState, returns Xsig_pred
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  const MatrixXd sigmaPoints = generateSigmaPoints();
  const MatrixXd augmentedSigmaPoints = augmentSigmaPoints();
  const MatrixXd sigmaPointsPrediction = sigmaPointPrediction(deltaT, augmentedSigmaPoints);
  KalmanState predictedKalmanState=predictMeanAndCovariance(sigmaPointsPrediction);
  updateKalmanState(predictedKalmanState);
  return sigmaPointsPrediction;
}

double UKF::Update(const MeasurementPackage& theMeasurementPackage, const Eigen::MatrixXd& xSigPredicted) {
  return Update(extractZMeasurement(theMeasurementPackage),xSigPredicted);
}

double UKF::Update(const Eigen::VectorXd& z, const Eigen::MatrixXd& xSigPredicted) {
  if (Tools::TESTING) std::cout << "UKF::Update-z = " << std::endl << z << std::endl;
  if (Tools::TESTING) std::cout << "UKF::Update-xSigPredicted = " << std::endl << xSigPredicted << std::endl;
  MatrixXd zSigmaPoints = transformSigmaPointsToMeasurements(xSigPredicted);
  VectorXd zPredicted = predictZ(zSigmaPoints);
  MatrixXd S = measurementCovarianceMatrix(zSigmaPoints, zPredicted);
  double nis = updateState(zSigmaPoints, zPredicted, xSigPredicted, S, z);
  return nis;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage& meas_package) {
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
void UKF::UpdateRadar(const MeasurementPackage& meas_package) {
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
  << "sensor:" << sensorType << std::endl
  << "x:" <<Tools::toString(x()) << std::endl
  << "n_z:" << n_z_ << std::endl
  << "P:" <<Tools::toString(P()) << std::endl;
  return oss.str();
}

Eigen::MatrixXd UKF::newCovariance(int numberOfSigmas, ...) {
  
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

int UKF::calculateLambda(const int theSizeOfTheStateVector) {
  assert(theSizeOfTheStateVector>=0 && theSizeOfTheStateVector<20);
  return 3 - theSizeOfTheStateVector;
}

double UKF::normalizeAngle(const double theAngle) {
  assert(theAngle < 4.*M_PI);
  double normalizedAngle=theAngle;
  while (normalizedAngle> M_PI) normalizedAngle-=2.*M_PI;
  while (normalizedAngle<-M_PI) normalizedAngle+=2.*M_PI;
  return normalizedAngle;
}

VectorXd UKF::normalizeStateVector(const VectorXd& theStateVector) {
  return Eigen::VectorXd(theStateVector);
}

VectorXd UKF::normalizeMeasurementVector(const VectorXd& theMeasurementVector) {
  return Eigen::VectorXd(theMeasurementVector);
}

MatrixXd UKF::generateSigmaPoints() {

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x(), 2 * n_x() + 1);
  
  //calculate square root of P
  MatrixXd A = P().llt().matrixL();
   
  /*******************************************************************************
   * Student part begin
   ******************************************************************************/
  
  //set first column of sigma point matrix
  Xsig.col(0)  = x();
  
  double lambda = 3-n_x();
  
  //set remaining sigma points
  for (int i = 0; i < n_x(); i++)
  {
    Xsig.col(i+1)     = x() + sqrt(lambda+n_x()) * A.col(i);
    Xsig.col(i+1+n_x()) = x() - sqrt(lambda+n_x()) * A.col(i);
  }
  
  /*******************************************************************************
   * Student part end
   ******************************************************************************/
  
  //print result
  if (Tools::TESTING) std::cout << "UKF::generateSigmaPoints-Xsig = " << std::endl << Xsig << std::endl;
  
  //write result
  return Xsig;
}

MatrixXd UKF::augmentSigmaPoints() {
 
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug());
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug(), n_aug());
  
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
  P_aug.bottomRightCorner(Q().rows(), Q().cols()) = Q();
  
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
  if (Tools::TESTING) std::cout << "UKF::augmentedSigmaPoint-lambda = " << lambda() << ", n_aug = " << n_aug() << std::endl;
  if (Tools::TESTING) std::cout << "UKF::augmentedSigmaPoint-Xsig_aug = " << std::endl << Xsig_aug << std::endl;
  
  //write result
  return Xsig_aug;
}

MatrixXd UKF::sigmaPointPrediction(const double deltaT, const MatrixXd& xSigAug) {
  
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
  if (Tools::TESTING) std::cout << "UKF::sigmaPointPrediction-xSigPredicted = " << std::endl << xSigPredicted << std::endl;
  
  //write result
  return xSigPredicted;
}

KalmanState UKF::predictMeanAndCovariance(const MatrixXd& xSigPredicted) {
  //std::cout << "UKF::predictMeanAndCovariance-weights: " << std::endl << weights() << std::endl;
  if (Tools::TESTING) std::cout << "UKF::predictMeanAndCovariance-xSigPredicted: " << std::endl << xSigPredicted << std::endl;
  
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
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    if (Tools::TESTING) std::cout << "UKF::predictMeanAndCovariance-x_diff: " << std::endl << x_diff << std::endl;
    x_diff=normalizeStateVector(x_diff);
    
    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }
  
  if (Tools::TESTING) std::cout << "UKF::predictMeanAndCovariance-x: " << std::endl << x << std::endl;
  if (Tools::TESTING) std::cout << "UKF::predictMeanAndCovariance-P: " << std::endl << P << std::endl;
  
  //*x_out=x;
  //*P_out=P;
  return KalmanState::KalmanState(x, P);
}

MatrixXd UKF::transformSigmaPointsToMeasurements(const MatrixXd& xSigPredicted) {
  //const int n_z =3; // radar: r, phi, r_dot
  MatrixXd Zsig = MatrixXd(n_z(), 2 * n_aug() + 1);
  
  /*******************************************************************************
   * Student part begin
   ******************************************************************************/
  
  //transform sigma points into measurement space
  assert(xSigPredicted.rows()==n_x() && xSigPredicted.cols()==2 * n_aug() + 1);
  for (int i = 0; i < 2 * n_aug() + 1; i++) {  //2n+1 simga points
    /*
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
     */
    Zsig.col(i)=transformStateToMeasurement(xSigPredicted.col(i));
  }
  if (Tools::TESTING) std::cout << "UKF::transformSigmaPointsToMeasurements-Zsig: " << std::endl << Zsig << std::endl;
  return Zsig;
}

MatrixXd UKF::measurementCovarianceMatrix(const MatrixXd& Zsig, const VectorXd& zPredicted) {
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z(),n_z());
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug() + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - zPredicted;
    
    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    z_diff=normalizeMeasurementVector(z_diff);
    
    S = S + weights(i) * z_diff * z_diff.transpose();
  }
  if (Tools::TESTING) std::cout << "UKF::measurementCovarianceMatrix-S: " << std::endl << S << std::endl;

  //add measurement noise covariance matrix
  //MatrixXd R = MatrixXd(n_z,n_z);
  //R <<    std_radr*std_radr, 0, 0,
  //0, std_radphi*std_radphi, 0,
  //0, 0,std_radrd*std_radrd;
  S = S + R();
  
  if (Tools::TESTING) std::cout << "UKF::measurementCovarianceMatrix-R: " << std::endl << R() << std::endl;
  if (Tools::TESTING) std::cout << "UKF::measurementCovarianceMatrix-S=S+R: " << std::endl << S << std::endl;
  return S;
}

VectorXd UKF::predictZ(const MatrixXd& Zsig) {
  //mean predicted measurement
  VectorXd zPredicted = VectorXd(n_z());
  zPredicted.fill(0.0);
  for (int i=0; i < 2*n_aug()+1; i++) {
    zPredicted = zPredicted + weights(i) * Zsig.col(i);
  }
  if (Tools::TESTING) std::cout << "UKF::predictZ-zPredicted: " << std::endl << zPredicted << std::endl;
  return zPredicted;
}

void UKF::predictZMeasurement(const MatrixXd& Zsig, const VectorXd& zPredicted, VectorXd* z_out, MatrixXd* S_out) {
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z(),n_z());
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug() + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - zPredicted;
    
    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    z_diff=normalizeMeasurementVector(z_diff);
    
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

VectorXd UKF::extractZMeasurement(const MeasurementPackage theMeasurementPackage) {
  return theMeasurementPackage.raw_measurements_;
}

VectorXd UKF::transformStateToMeasurement(const VectorXd& theState) {
  throw std::logic_error("Not Implemented");
}

double UKF::updateState(const Eigen::MatrixXd& Zsig, const Eigen::MatrixXd& z_pred,
                             const Eigen::MatrixXd& Xsig_pred, const Eigen::MatrixXd& S, const Eigen::VectorXd& z) {
  
  if (Tools::TESTING) std::cout << "UKF::updateState-Zsig: " << std::endl << Zsig << std::endl;
  if (Tools::TESTING) std::cout << "UKF::updateState-z_pred: " << std::endl << z_pred << std::endl;
  if (Tools::TESTING) std::cout << "UKF::updateState-Xsig_pred: " << std::endl << Xsig_pred << std::endl;
  
  int n_z=Zsig.rows();
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x(), n_z);
  
  /*******************************************************************************
   * Student part begin
   ******************************************************************************/
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug() + 1; i++) {  //2n+1 simga points
    
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    z_diff=normalizeMeasurementVector(z_diff);
    
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x();
    //angle normalization
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    x_diff=normalizeMeasurementVector(x_diff);

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }
  if (Tools::TESTING) std::cout << "UKF::updateState-Tc: " << std::endl << Tc << std::endl;
  if (Tools::TESTING) std::cout << "UKF::updateState-S: " << std::endl << S << std::endl;

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //residual
  VectorXd z_diff = z - z_pred;
  
  //angle normalization
  //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  z_diff=normalizeMeasurementVector(z_diff);

  //update state mean and covariance matrix
  VectorXd newX = VectorXd(n_x());
  newX = x() + K * z_diff;
  MatrixXd newP = MatrixXd(n_x(), n_x());
  newP = P() - K*S*K.transpose();
  
  const double nis = z_diff.transpose()*S.inverse()*z_diff;
  /*******************************************************************************
   * Student part end
   ******************************************************************************/
  
  //print result
  if (Tools::TESTING) std::cout << "UKF::updateState-Updated state x: " << std::endl << newX << std::endl;
  if (Tools::TESTING) std::cout << "UKF::updateState-Updated state covariance P: " << std::endl << newP << std::endl;
  
  //write result
  //*x_out = newX;
  //*P_out = newP;
  KalmanState newKalmanState = KalmanState::KalmanState(newX, newP);
  updateKalmanState(newKalmanState);
  return nis;
}


RadarFilter::RadarFilter(const int theNumberofAugmentedStates, KalmanState& theKalmanState,
                         Eigen::MatrixXd theMeasurementNoiseCovariance/*R*/, Eigen::MatrixXd theProcessNoiseCovariance/*Q*/) :
    UKF(theNumberofAugmentedStates,
        3/* theNumberOfRadarMeasurements */, MeasurementPackage::RADAR,
        theKalmanState,
        theMeasurementNoiseCovariance, theProcessNoiseCovariance) {
  assert(n_aug()==theNumberofAugmentedStates);
  //set augmented dimension
  // initial covariance matrix
  //R_ = UKF::newR(3, std_radr_, std_radphi_, std_radrd_);
  //R_ <<   std_radr_*std_radr_,  0,                        0,
  //        0,                    std_radphi_*std_radphi_,  0,
  //        0,                    0,                        std_radrd_*std_radrd_;
  assert(n_x()>0);
}

VectorXd RadarFilter::transformMeasurementToState(const VectorXd& theRawMeasurements) {
  assert(n_z()==3);
  assert(theRawMeasurements.size()==n_z());
  double rho = theRawMeasurements(0);
  double phi = theRawMeasurements(1);
  double rhodot = theRawMeasurements(2);
  
  double vx = rho * cos(phi);
  double vy = rho * sin(phi);
  
  assert(n_x()==5);
  VectorXd state = VectorXd(n_x());
  state(0) = rho*cos(phi);
  state(1) = rho*sin(phi);
  state(2) = sqrt(vx*vx+vy*vy);
  state(3) = 0.;
  state(4) = 0.;
  return state;
}


VectorXd RadarFilter::transformStateToMeasurement(const VectorXd& theState) {
  assert(n_x()==5);
  // extract values for better readibility
  const double p_x = theState(0);
  const double p_y = theState(1);
  const double v  = theState(2);
  const double yaw = theState(3);
  const double yawd = theState(4);

  const double v1 = cos(yaw)*v;
  const double v2 = sin(yaw)*v;
  
  assert(n_z()==3);
  VectorXd z = VectorXd(n_z());
  // measurement model
  z(0) = sqrt(p_x*p_x + p_y*p_y);                        //r
  z(1) = atan2(p_y,p_x);                                 //phi
  z(2) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot

  return z;
}

VectorXd RadarFilter::normalizeStateVector(const VectorXd& theStateVector) {// px,py,v,yaw,yawd,v_aug,v_augdd
  VectorXd normalizedStateVector=UKF::normalizeStateVector(theStateVector);// make a copy
  normalizedStateVector(3)=normalizeAngle(normalizedStateVector(3));// normalize yaw
  return normalizedStateVector;
}


VectorXd RadarFilter::normalizeMeasurementVector(const VectorXd& theMeasurementVector) {// rho, phi, rho_dot
  VectorXd normalizedMeasurementVector=UKF::normalizeMeasurementVector(theMeasurementVector);// make a copy
  normalizedMeasurementVector(1)=normalizeAngle(normalizedMeasurementVector(1));// normalize phi
  return normalizedMeasurementVector;
}

LidarFilter::LidarFilter(const int theNumberofAugmentedStates, KalmanState& theKalmanState,
                         MatrixXd theMeasurementNoiseCovariance/*R*/, MatrixXd theProcessNoiseCovariance/*Q*/) :
UKF(theNumberofAugmentedStates,
    2 /* theNumberOfLidarMeasurements */, MeasurementPackage::LASER,
    theKalmanState, theMeasurementNoiseCovariance, theProcessNoiseCovariance) {
  assert(n_aug()==7);
}

VectorXd LidarFilter::transformMeasurementToState(const VectorXd& theRawMeasurements) {
  assert(n_z()==2);
  assert(theRawMeasurements.size()==n_z());
  double px = theRawMeasurements(0);
  double py = theRawMeasurements(1);
  assert(n_x()==5);
  VectorXd state = VectorXd(n_x());
  state(0) = px;
  state(1) = py;
  state(2) = 0.;
  state(3) = 0.;
  state(4) = 0.;
  return state;
}
VectorXd LidarFilter::transformStateToMeasurement (const VectorXd& theState) {
  assert(n_x()==5);
  // extract values for better readibility
  const double p_x = theState(0);
  const double p_y = theState(1);
  const double v  = theState(2);
  const double yaw = theState(3);
  const double yawd = theState(4);
  

  assert(n_z()==2);
  VectorXd z = VectorXd(n_z());
  // measurement model
  z(0) = p_x;
  z(1) = p_y;
  
  return z;
}

UKFProcessor::UKFProcessor(KalmanState& theKalmanState, int theNumberOfAugmentedStates, MatrixXd& theRadarR, MatrixXd& theLidarR, MatrixXd& theProcessNoiseQ) :
  kalmanState_(theKalmanState),
  radarFilter(RadarFilter(theNumberOfAugmentedStates, kalmanState_, theRadarR, theProcessNoiseQ)),
  lidarFilter(LidarFilter(theNumberOfAugmentedStates, kalmanState_, theLidarR, theProcessNoiseQ)) {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  if (Tools::TESTING) std::cout << "UKFProcessor::constructor-radarFilter:" << radarFilter.toString() << std::endl;
  if (Tools::TESTING) std::cout << "UKFProcessor::constructor-lidarFilter:" << lidarFilter.toString() << std::endl;
}

UKFProcessor::~UKFProcessor() {}

double UKFProcessor::ProcessMeasurement(const MeasurementPackage& theMeasurementPackage) {
  
  switch(theMeasurementPackage.sensor_type_) {
    case MeasurementPackage::RADAR :
      if (Tools::TESTING) std::cout << "UKFProcessor::RADAR" << std::endl;
      if (useRadar()) {
        return processMeasurement(theMeasurementPackage, &radarFilter);
      }
      break;
    case MeasurementPackage::LASER :
      if (Tools::TESTING) std::cout << "UKFProcessor::LIDAR" << std::endl;
      if (useLidar()) {
        return processMeasurement(theMeasurementPackage, &lidarFilter);
      }
      break;
    default :
      throw std::invalid_argument("");
  }
  throw std::logic_error("Not Implemented");
}

double UKFProcessor::processMeasurement(const MeasurementPackage& theMeasurementPackage,
                                              UKF *theFilter) {
  
  if (!is_initialized()) {
    initialize(theMeasurementPackage, theFilter, kalmanState());
    return 0.;
  }

  double deltaT=(theMeasurementPackage.timestamp_-time_us_)/1000000.;//in seconds
  time_us_= theMeasurementPackage.timestamp_;
  return (*theFilter).ProcessMeasurement(theMeasurementPackage, deltaT);
}

void UKFProcessor::initialize(const MeasurementPackage& theMeasurementPackage,
                              UKF *theFilter,
                              KalmanState& theKalmanState) {
  if (!is_initialized()) {
    initialize(theMeasurementPackage,
               (*theFilter).transformMeasurementToState(theMeasurementPackage.raw_measurements_),
               theKalmanState);
  }
}

void UKFProcessor::initialize(const MeasurementPackage& theMeasurementPackage,
                              const VectorXd theInitialState,
                              KalmanState& theKalmanState) {
  if (!is_initialized()) {
    /**
     TODO:
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    
    //previous_timestamp_= theMeasurementPackage.timestamp_; // initial dt == 0.
    time_us_= theMeasurementPackage.timestamp_;
    
    // first measurement
    
    assert(theKalmanState.n_x()>0);
    
    KalmanState newKalmanState=
    KalmanState::KalmanState(theInitialState, theKalmanState.P());
    theKalmanState.update(newKalmanState);
    is_initialized_ = true;
    if (Tools::TESTING) {
      std::cout << "UKF::initialize:" << theKalmanState.toString() << std::endl;
    }
  }
}


