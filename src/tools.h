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
  
  static bool areSame(VectorXd a, VectorXd b, double epsilon);
  static bool areSame(MatrixXd a, MatrixXd b, double epsilon);
  static bool areSame(double a, double b, double epsilon);
  static bool isZero(double a, double epsilon);
  static bool isNotZero(double a, double epsilon);
  
  static void testGenerateSigmaPoints();
  static void testAugmentedSigmaPoints();
  static void testSigmaPointPrediction();
  static void testPredictZMeasurement();
  static void testPredictMeanAndCovariance();
  static void testPrediction();
  
  static void testUpdateState();
  static void testUpdate();
  
  static void testRadar();
  
  static double normalizeAngle(double angle) {
    double a = fmod(angle + M_PI, 2 * M_PI);
    return a >= 0 ? (a - M_PI) : (a + M_PI);
  }
  
  static const bool TESTING=false;
  static const bool FILETESTING=false;

  static std::string toString(VectorXd vector);
  static std::string toString(MatrixXd matrix);
  static std::string toStringSize(int rows, int columns);

  static const bool SEARCHING=false;

};

#endif /* TOOLS_H_ */
