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
  PID pid_throttle;
  
  // Init PID Controller with parameters for steering control
  double Kp_steer;
  double Kd_steer;
  double Ki_steer;
  
  // Init PID Controller with parameters for trottle/brake control
  double Kp_trottle = -0.45;
  double Kp_offset_trottle = 1.6;
  
  if(argc == 1) {
    // Set default values if no parameter is given
    Kp_steer = 0.2;
    Ki_steer = 0.001;
    Kd_steer = 3.0;
  }
  else if (argc == 4) {
    // Set parameter values from terminal input
    Kp_steer = strtod(argv[1], NULL);
    Ki_steer = strtod(argv[2], NULL);
    Kd_steer = strtod(argv[3], NULL);
  } else {
    std::cout << "Usage: ./pid kp ki kd " << std::endl;
    return -1 ;
  }
  
  // Init the PID controller for steering
  pid_steer.Init(Kp_steer, Ki_steer, Kd_steer, 0.0);
  
  // Init the PID controller for throttle
  pid_throttle.Init(Kp_trottle, 0.0, 0.0, Kp_offset_trottle);

  h.onMessage([&pid_steer, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double trottle_value = 1.0;
          int speed_max;
          bool brake_used = false;
          double cte_min = pid_steer.ReturnCteMin();
          double cte_max = pid_steer.ReturnCteMax();
          
          /***************************
           * Configuration of safety-mode which sets a max-speed of 50 miles/h
           * Set safe-mode = true   ---> for a smooth ride
           * Set safe-mode = false  ---> to see the car racing and see the limits of the solution
           *                            This default-value is not useable in all cases.
           *                            It depands, how much computing power is available.
           *                            (use safe-mode = true in this cases).
           ***************************/
          bool safe_mode = true;
          
          if(safe_mode == true) {
            speed_max = 40;
          }
          else {
            speed_max = 80;
          }
          
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();
          //trottle_value = pid.ReturnTrottleValue();
          
          if(trottle_value < 0) {
            brake_used = true;
          }
          else {
            brake_used = false;
          }
          
          // If the speed limit is reached decrease the trottle to hold the speed
          if(speed > speed_max){
            trottle_value = 0.35;
          }
          
          // Print out some information
          std::cout << "CTE: " << cte << " CTE_min: " << cte_min << " CTE_max: " << cte_max << std::endl;
          std::cout << "Speed: " << speed << std::endl;
          std::cout << "Steering Value: " << steer_value << " Trottle Value: " << trottle_value << " Brake used: " << brake_used << std::endl;

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
