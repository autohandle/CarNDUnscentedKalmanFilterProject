#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"
#include <string>

class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
    RADAR,
    INVALID
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

  static const std::string SensorTypeToString;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
