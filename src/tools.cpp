#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth){
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
     || estimations.size() == 0){
    cout << "Tools::CalculateRMSE-Invalid estimation or ground_truth data" << endl;
    return rmse;
  }
  
  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    
    VectorXd residual = estimations[i] - ground_truth[i];
    
    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  
  //calculate the mean
  rmse = rmse/estimations.size();
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
  
  //return the result
  return rmse;
}

bool Tools::areSame(VectorXd a, VectorXd b) {
  for (int r=0; r<a.rows(); r++) {
    if (!Tools::areSame(a(r),b(r))) {
      std::cout << std::endl
      << "a(" << r << "):" << a(r) << " != "
      << "b(" << r << "):" << b(r) << " == "
      << "fabs(a-b):" << fabs(a(r) - b(r))
      << std::endl;
      return false;
    }
  }
  return true;
}

bool Tools::areSame(MatrixXd a, MatrixXd b) {
  for (int r=0; r<a.rows(); r++) {
    for (int c=0; c<a.cols(); c++) {
      if (!Tools::areSame(a(r,c),b(r,c))) {
        std::cout << std::endl
        << "a(" << r << "," << c << "):" << a(r,c) << " != "
        << "b(" << r << "," << c << "):" << b(r,c) << " == "
        << "fabs(a-b):" << fabs(a(r,c) - b(r,c))
        << std::endl;
        return false;
      }
    }
  }
  return true;
}

static const double EPSILON=1.e-5;

bool Tools::areSame(double a, double b) {
  return fabs(a - b) < EPSILON;
}

bool Tools::isZero(double a) {
  return fabs(a) < EPSILON;
}

bool Tools::isNotZero(double a) {
  return !isZero(a);
}

std::string Tools::toString(VectorXd vector) {
  std::ostringstream ss;
  ss << "[" << vector.rows() << "x_]:";
  for (int r=0; r<vector.rows(); r++ ) {
    if ((r == 0) && (vector.rows() > 1)) {
      ss << std::endl;
    }
    if (r == 0) {
      ss << "[";
    } else {
      ss << "," << std::endl;
    }
    ss << vector(r);
  }
  ss << "]";
  //ss << "[" << toStringSize(vector.rows(), vector.cols()) << "]=" << vector;
  return ss.str();
}

std::string Tools::toString(MatrixXd matrix) {
  //std::cout << typeid(matrix(0)).name() << '\n';
  //std::cout << decltype(matrix) << '\n';
  std::ostringstream ss;
  ss << "[" << Tools::toStringSize(matrix.rows(), matrix.cols()) << "]=";
  for (int r=0; r<matrix.rows(); r++ ) {
    if ((r == 0) && (matrix.rows() > 1)) {
      ss << std::endl;
    }
    if (r != 0 ) ss << "," << std::endl;
    ss << "[" << matrix.row(r) << "]";
  }
  return ss.str();
}

std::string Tools::toStringSize(int rows, int columns) {
  std::ostringstream ss;
  ss << rows << "x" << columns;
  return ss.str();
}
