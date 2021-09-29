#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// helper function for increment iterator
int next(int iter) { return (iter + 1 == 3) ? 0 : iter + 1;}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argC, char* argV[]) {
  uWS::Hub h;

  // some constants for twiddling
  const double threshold = 0.001;
  const int max_steps = 500;

  // pid and twiddling variables
  PID pid;
  bool twiddle = false;
  bool initialized = false;
  int step = 0;
  double sum_cte = 0.0;
  double best_cte;
  double p[3] = {0.07, 0.0001, 1.9};
  double dp[3] = {0.005, 0.000001, 0.1};
  int p_iter = 0; // dp iterator
  bool p_was_increased = false;

  if (argC == 4) {
    double Kp, Ki, Kd;
    Kp = atof(argV[1]);
    Ki = atof(argV[2]);
    Kd = atof(argV[3]);
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;

  } else {
    if (argC == 2) {
      twiddle = true;
      std::cout << "twiddle is set to true" << std::endl;
    }
  }

  pid.Init(p[0], p[1], p[2]);

  h.onMessage([&pid, &twiddle, &initialized, &step, &sum_cte, &best_cte, &dp, &p, &p_iter, &p_was_increased, &threshold, &max_steps](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          json msgJson;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           */
 
          if (!twiddle) { // no twiddling
            pid.UpdateError(cte);
            steer_value = -1.0 * pid.TotalError();
            if (steer_value > 1) steer_value = 1;
            else if (steer_value < -1) steer_value = -1;
            
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                      << std::endl;
            
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          } else {  // twiddling
            if (step == 0) {  // a new simulation begins, need to use new pid param
              pid.Init(p[0], p[1], p[2]);
            }
            pid.UpdateError(cte);
            steer_value = -1.0 * pid.TotalError();

            step += 1;
            sum_cte += cte * cte;

            if (step < max_steps) {  // haven't completed 1 simulation cycle
              // normal operations
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              // std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
            } else { // step == max_steps, completed 1 simulation cycle
              step = 0; // reset
              
              double avg_cte = sum_cte / max_steps;
              sum_cte = 0;  // reset
              
              if (!initialized) {
                // setting up the initial best_cte
                best_cte = avg_cte;
                initialized = true;
                p[p_iter] += dp[p_iter];
                p_was_increased = true;
              } else {
                // compare to the current best_cte
                if (avg_cte < best_cte) {
                  best_cte = avg_cte;
                  if (p_was_increased) {
                    // initial changing direction is the right direction
                    dp[p_iter] *= 1.1;
                  } else {  // p was decreased
                    dp[p_iter] *= 1.05;
                  }
                  p_iter = next(p_iter);
                  
                  // update p[i] for next simulation cycle
                  p[p_iter] += dp[p_iter];
                  p_was_increased = true;
                }
                else {  // haven't found the right direction of changing dp
                  if (p_was_increased) {  
                    // change p in the decreasing direction
                    p[p_iter] -= 2 * dp[p_iter];
                    p_was_increased = false;
                  } else {  // p was decreased
                    // there was no improvements in both directions
                    p[p_iter] += dp[p_iter];  // restore p
                    dp[p_iter] *= 0.95;
                    p_iter = next(p_iter);

                    // update p[i] for next simulation cycle
                    p[p_iter] += dp[p_iter];
                    p_was_increased = true;
                  }
                }
              }

              std::cout << "avg cte: " << avg_cte << std::endl;
              // std::cout << "iterator: " << p_iter << std::endl;
              // std::cout << "p[i]: " << p[p_iter] << std::endl;
              // std::cout << "dp[i]: " << dp[p_iter] << std::endl;

              if (dp[0] + dp[1] + dp[2] < threshold) {
                // stopping criterion met
                std::cout << "Best Kp Ki Kd: " << p[0] << p[1] << p[2] << std::endl;
              } else {
                // restart simulation
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            }
          }         
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}