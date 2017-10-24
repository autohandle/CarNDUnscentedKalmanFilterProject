#include <uWS/uWS.h>
//#include "uWS/uWS.h"
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

VectorXd createEstimateVector(const VectorXd& x) {
  double p_x = x(0);
  double p_y = x(1);
  double v1  = x(2);
  double v2 = x(3);
  
  VectorXd estimate(4);
  
  estimate(0) = p_x;
  estimate(1) = p_y;
  estimate(2) = v1;
  estimate(3) = v2;
  
  return estimate;
}

VectorXd createGroundTruthVector(std::string theSensorMeasurement) {
  
  MeasurementPackage measurementPackage;
  
  istringstream iss(theSensorMeasurement);
  
  // reads first element from the current line
  string sensorType;
  iss >> sensorType;
  
  if (sensorType.compare("L") == 0) {
    for (int toss=0; toss<3; toss++) {
      string measurement;
      iss >> measurement;
    }
    
  } else if (sensorType.compare("R") == 0) {
    for (int toss=0; toss<4; toss++) {
      string measurement;
      iss >> measurement;
    }
  }
  
  float x_gt;
  float y_gt;
  float vx_gt;
  float vy_gt;
  iss >> x_gt;
  iss >> y_gt;
  iss >> vx_gt;
  iss >> vy_gt;
  VectorXd gt_values(4);
  gt_values(0) = x_gt;
  gt_values(1) = y_gt;
  gt_values(2) = vx_gt;
  gt_values(3) = vy_gt;
  
  return gt_values;
}

MeasurementPackage createMeasurementPackage(std::string theSensorMeasurement) {
  
  MeasurementPackage measurementPackage;
  
  istringstream iss(theSensorMeasurement);
  long long timestamp;
  
  // reads first element from the current line
  string sensorType;
  iss >> sensorType;
  
  if (sensorType.compare("L") == 0) {
    
    measurementPackage.sensor_type_ = MeasurementPackage::LASER;
    measurementPackage.raw_measurements_ = VectorXd(2);
    float px;
    float py;
    iss >> px;
    iss >> py;
    measurementPackage.raw_measurements_ << px, py;
    iss >> timestamp;
    measurementPackage.timestamp_ = timestamp;
    
  } else if (sensorType.compare("R") == 0) {
    
    measurementPackage.sensor_type_ = MeasurementPackage::RADAR;
    measurementPackage.raw_measurements_ = VectorXd(3);
    float ro;
    float theta;
    float ro_dot;
    iss >> ro;
    iss >> theta;
    iss >> ro_dot;
    measurementPackage.raw_measurements_ << ro,theta, ro_dot;
    iss >> timestamp;
    measurementPackage.timestamp_ = timestamp;
  } else {
    measurementPackage.sensor_type_ = MeasurementPackage::INVALID;
  }
  
  return measurementPackage;
}

VectorXd compareRSME(VectorXd thePreviousRSME, VectorXd theCurrentRSME) {
  assert (thePreviousRSME.size()==theCurrentRSME.size());
  VectorXd comparison = VectorXd(4);
  for (int row=0; row<thePreviousRSME.size(); row++) {
    comparison(row)=theCurrentRSME(row)-thePreviousRSME(row);
  }
  return comparison;
}

bool isRadar(const MeasurementPackage theMeasurementPackage) {
  return theMeasurementPackage.sensor_type_==MeasurementPackage::RADAR;
}

bool isLidar(const MeasurementPackage theMeasurementPackage) {
  return theMeasurementPackage.sensor_type_==MeasurementPackage::LASER;
}

bool isProcessingSensorMeasurement(const MeasurementPackage theMeasurementPackage, UKFProcessor theUKFProcessor) {
  return (isRadar(theMeasurementPackage) && theUKFProcessor.useRadar()) || (isLidar(theMeasurementPackage) && theUKFProcessor.useLidar());
}

VectorXd transformKalmanStateToGroundTruthState(const VectorXd& theKalmanState) {
  assert(theKalmanState.size()==5);
  // extract values for better readibility
  const double px = theKalmanState(0);
  const double py = theKalmanState(1);
  const double v  = theKalmanState(2);
  const double yaw = theKalmanState(3);
  const double yawd = theKalmanState(4);
  
  const double vx = v * cos(yaw);
  const double vy = v * sin(yaw);
  
  VectorXd groundTruthState = VectorXd(4);// px, py, vx, vy
  // measurement model
  groundTruthState(0) = px;
  groundTruthState(1) = py;
  groundTruthState(2) = vx;
  groundTruthState(3) = vy;
  
  return groundTruthState;
}

VectorXd transformKalmanStateToGroundTruthState(KalmanState& theKalmanState) {
  return transformKalmanStateToGroundTruthState(theKalmanState.x());
}

VectorXd runAsFileProcessor(UKFProcessor theUKFProcessor, std::string theFileName) {
  
  ifstream measurementFile;
  measurementFile.open(theFileName, ios::in);
  if (Tools::TESTING) cout<<"runAsFileProcessor-theFileName: <"<< theFileName << ">, is_open? " << measurementFile.is_open() << "\n";
  
  int lineNumber = 0;
  
  if (measurementFile.is_open()) {
    vector<VectorXd> estimates;
    vector<VectorXd> groundTruthVector;
    if (Tools::TESTING) cout<<"runAsFileProcessor-theFileName: "<< theFileName << "\n";
    string measurementLine;
    VectorXd rsme;
    while ( getline (measurementFile, measurementLine) ) {
      lineNumber++;
      if (Tools::TESTING) {
        cout << "l-------------------------------------------" << "\n"
        << lineNumber << ": <" << measurementLine << ">\n"
        << "l-------------------------------------------" << "\n";
      }
      MeasurementPackage measurementPackage = createMeasurementPackage(measurementLine);

      if (isProcessingSensorMeasurement(measurementPackage, theUKFProcessor)) {
        double nis = theUKFProcessor.ProcessMeasurement(measurementPackage);
        KalmanState kalmanState = theUKFProcessor.kalmanState();

        VectorXd groundTruthValues = createGroundTruthVector(measurementLine);
        if (Tools::TESTING) cout<<"runAsFileProcessor-groundTruthValues: <"<< Tools::toString(groundTruthValues) << "\n";
        groundTruthVector.push_back(groundTruthValues);
        VectorXd estimate = transformKalmanStateToGroundTruthState(kalmanState);
        if (Tools::TESTING) cout<<"runAsFileProcessor-estimate: <"<< Tools::toString(estimate) << "\n";
        estimates.push_back(estimate);
        rsme = Tools::CalculateRMSE(estimates, groundTruthVector);
        if (Tools::FILETESTING || Tools::TESTING) {
          cout <<"runAsFileProcessor-rsme: <"<< Tools::toString(rsme) << "\n";
          cout << "nis: " << nis << ">\n";
        }
      }
    }
    measurementFile.close();
    return rsme;
  } else {
    cout<<"runAsFileProcessor-theFileName: <"<< theFileName << "> failed to open\n";
    throw std::logic_error("file error");
  }
  measurementFile.close();
  throw std::logic_error("? error");
}

int main(int argc, char *argv[])
{
  uWS::Hub h;
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  const double std_a_ = 0.6;//0.5;//0.7;
  
  
  // Process noise standard deviation yaw acceleration in rad/s^2
  const double std_yawdd_ = 0.6;//0.8;//0.6;
  
  MatrixXd processNoiseQ = UKF::newCovariance2(std_a_, std_yawdd_);


  // Laser measurement noise standard deviation position1 in m
  const double std_laspx_ = 0.15;
  
  // Laser measurement noise standard deviation position2 in m
  const double std_laspy_ = 0.15;
  
  // Radar measurement noise standard deviation radius in m
  const double std_radr_ = 0.3;
  
  // Radar measurement noise standard deviation angle in rad
  const double std_radphi_ = 0.03;
  
  // Radar measurement noise standard deviation radius change in m/s
  const double std_radrd_ = 0.3;
  
  MatrixXd radarR = UKF::newCovariance3(std_radr_, std_radphi_, std_radrd_);
  MatrixXd lidarR = UKF::newCovariance2(std_laspx_, std_laspy_);

  // Create a Kalman Filter instance
  KalmanState kalmanState = KalmanState(5);
  UKFProcessor ukfProcessor=UKFProcessor(kalmanState,7/*numberof augmented states*/,
                                         radarR, lidarR, processNoiseQ);
  //UKF ukf;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
    
  int status=0;
  
  if (Tools::TESTING) cout<<"argc: "<< argc <<"\n";
  if (Tools::FILETESTING && argc>1) {
    if (Tools::TESTING) {
      cout << "argv[0]: " << argv[0] << ", \nargv[1]: " << argv[1] << "\n";
    }
    //Tools::testRadar();
    VectorXd rsme=runAsFileProcessor(ukfProcessor, argv[1]);
  } else {
    //status=runAsServer(fusionEKF);
  }
  if (Tools::FILETESTING) {
    cout<< "status: " << status <<"\n";
    return(status);
  }
  
  if (Tools::SEARCHING && (argc>1)) {
    cout<<"SEARCHING argc: "<< argc <<"\n";
    for (int standardDeviation=1; standardDeviation<10; standardDeviation++) {
      estimations.clear();
      ground_truth.clear();
      MatrixXd processNoiseQ = UKF::newCovariance2(standardDeviation*0.1, 0.6);
      KalmanState kalmanState = KalmanState(5);
      cout <<"SEARCHING-standardDeviation: "<< standardDeviation*0.1 << "\n";
      cout <<"SEARCHING-processNoiseQ: <"<< Tools::toString(processNoiseQ) << "\n";
      UKFProcessor ukfProcessor = UKFProcessor(kalmanState,7/*numberof augmented states*/,
                                               radarR, lidarR, processNoiseQ);
      VectorXd rsme = runAsFileProcessor(ukfProcessor, argv[1]);
      cout <<"SEARCHING-rsme: <"<< Tools::toString(rsme) << "\n";
    }
    return(status);
  }


//  h.onMessage([&ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  h.onMessage([&ukfProcessor,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
    	  long long timestamp;

    	  // reads first element from the current line
    	  string sensor_type;
    	  iss >> sensor_type;

    	  if (sensor_type.compare("L") == 0) {
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          }
          float x_gt;
    	  float y_gt;
    	  float vx_gt;
    	  float vy_gt;
    	  iss >> x_gt;
    	  iss >> y_gt;
    	  iss >> vx_gt;
    	  iss >> vy_gt;
    	  VectorXd gt_values(4);
    	  gt_values(0) = x_gt;
    	  gt_values(1) = y_gt; 
    	  gt_values(2) = vx_gt;
    	  gt_values(3) = vy_gt;
    	  ground_truth.push_back(gt_values);
          
          //Call ProcessMeasurment(meas_package) for Kalman filter
//    	  ukf.ProcessMeasurement(meas_package);
          ukfProcessor.ProcessMeasurement(meas_package);

    	  //Push the current estimated x,y positon from the Kalman filter's state vector

    	  VectorXd estimate(4);

    	  //double p_x = ukf.x_(0);
    	  //double p_y = ukf.x_(1);
    	  //double v  = ukf.x_(2);
    	  //double yaw = ukf.x_(3);
          double p_x = ukfProcessor.x()(0);
          double p_y = ukfProcessor.x()(1);
          double v  = ukfProcessor.x()(2);
          double yaw = ukfProcessor.x()(3);

    	  double v1 = cos(yaw)*v;
    	  double v2 = sin(yaw)*v;

    	  estimate(0) = p_x;
    	  estimate(1) = p_y;
    	  estimate(2) = v1;
    	  estimate(3) = v2;
    	  
    	  estimations.push_back(estimate);

    	  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}























































































