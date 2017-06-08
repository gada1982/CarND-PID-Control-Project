/*
 * main.cpp
 *
 * Created on: June 02, 2017
 * Author: Daniel Gattringer
 * Mail: daniel@gattringer.biz
 */

#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, const char *argv[])
{
  uWS::Hub h;

  PID pid_steer;
  
  // Init PID Controller with parameters for steering control
  double Kp_steer;
  double Kd_steer;
  double Ki_steer;
  
  if(argc == 1) {
    // Set default values if no parameter is given
    Kp_steer = 0.11;
    Ki_steer = 0.0008;
    Kd_steer = 2.5;
  }
  else if (argc == 4) {
    // Set parameter values from terminal input
    Kp_steer = strtod(argv[1], NULL);
    Ki_steer = strtod(argv[2], NULL);
    Kd_steer = strtod(argv[3], NULL);
  } else {
    std::cout << "Usage: ./pid kp ki kd" << std::endl;
    return -1 ;
  }
  
  // Init the PID controller for steering
  pid_steer.Init(Kp_steer, Ki_steer, Kd_steer);
  h.onMessage([&pid_steer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double trottle_value = 0.5;
          double cte_min = pid_steer.ReturnCteMin();
          double cte_avg = pid_steer.ReturnCteAvg();
          double cte_max = pid_steer.ReturnCteMax();
          double steer_min = pid_steer.ReturnSteerMin();
          double steer_max = pid_steer.ReturnSteerMax();
          
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();
          
          // Print out some information
          std::cout << "\nCTE: " << cte << " CTE_min: " << cte_min << " CTE_avg: " << cte_avg << " CTE_max: " << cte_max << std::endl;
          std::cout << "Steering Value: " << steer_value << " Steer_min: " << steer_min << " Steer_max: " << steer_max << std::endl;
          std::cout << "Trottle Value: " << trottle_value << std::endl;
          std::cout << "Speed: " << speed << std::endl;
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = trottle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
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
