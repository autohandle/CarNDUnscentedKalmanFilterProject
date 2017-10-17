#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  static VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
  
  static bool areSame(VectorXd a, VectorXd b);
  static bool areSame(MatrixXd a, MatrixXd b);
  static bool areSame(double a, double b);
  static bool isZero(double a);
  static bool isNotZero(double a);
  
  static double normalizeAngle(double angle) {
    double a = fmod(angle + M_PI, 2 * M_PI);
    return a >= 0 ? (a - M_PI) : (a + M_PI);
  }
  
  static const bool TESTING=true;
  
  static std::string toString(VectorXd vector);
  static std::string toString(MatrixXd matrix);
  static std::string toStringSize(int rows, int columns);
  
};

#endif /* TOOLS_H_ */
