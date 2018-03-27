#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(int argc, char *argv[]) {
  
  uWS::Hub h;
  
  // Use the command line arguments for cost function
  std::vector<long> cost_value (7,1);
  
  // Use the default values when no command line values
  cost_value[4] = 50;  
  cost_value[5] = 250000;
  cost_value[6] = 1;
  
  if((argc > 1) && (argc <= 8))
  {
    for(unsigned int i = 0; i < (argc - 1); i++)
    {
      cost_value[i] = atol(argv[i+1]);
    }
  }

  // MPC is initialized here!
  MPC mpc;
  mpc.UpdateCost(cost_value);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          
          double steer_value_i = j[1]["steering_angle"];
          double throttle_value_i = j[1]["throttle"];




          // Add in the latency time for processing
          double delay_t = .1;
          
          // Lf is the distance of the steering axle from COG
          const double Lf = 2.67;
          
          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // loop through all observations
          for (unsigned int j = 0; j < ptsx.size(); j++)
          {
      
            double veh_x, veh_y;
            
            // Convert vehicle co-ordinates to map co-ordinates
            veh_x = (ptsx[j] - px) * cos(-psi) - (ptsy[j] - py) * sin(-psi);
            veh_y = (ptsy[j] - py) * cos(-psi) + (ptsx[j] - px) * sin(-psi);
            
            next_x_vals.push_back(veh_x);
            next_y_vals.push_back(veh_y);            
          }
          
          // Convert the vectors to Eigen VectorXd before using in polyfit
          Eigen::Map<Eigen::VectorXd> next_x_vals_eig(next_x_vals.data(), next_x_vals.size());
          Eigen::Map<Eigen::VectorXd> next_y_vals_eig(next_y_vals.data(), next_y_vals.size());
          
          // Find the coefficients of the best fit polynomial from the given waypoints
          auto coeffs = polyfit(next_x_vals_eig, next_y_vals_eig, 3);
          
          // Calculate the error at x=0, the position just in front of car for CTE
          double cte = polyeval(coeffs,0);
          
          // Calculation for Euclidean distance of last way point //Not used in this code
          double euc_d = fabs(sqrt((next_x_vals[-1] * next_x_vals[-1]) 
                  + ( polyeval(coeffs,next_x_vals[-1]) *  polyeval(coeffs,next_x_vals[-1]))));
                  
          //std::cout << euc_d << std::endl;
          
          // Calculate the error in orientation of the vehicle
          double epsi = -atan(coeffs[1]);
          
          
          // State after delay.          
          //transition from t to t+1 (time step is delta_t = 0.1)
          // using the general kinematic equations
          
          double delay_x = v * cos(0) * delay_t; // Assuming initial x = 0 in car co-ordinate
          double delay_y = v * sin(0) * delay_t; // Assuming initial y = 0 in car co-ordinate
          double delay_psi = -v * steer_value_i * delay_t / Lf ; // Assuming initial psi = 0 in car co-ordinate
          double delay_v = v + throttle_value_i * delay_t; 
          double delay_cte = cte + v * sin(epsi) * delay_t;
          double delay_epsi = epsi-(v * steer_value_i * delay_t / Lf);
          
          // Update the states to be used in ipopt solver
          Eigen::VectorXd state(6);
          state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;
          //state <<  0.,  0., 0., v, cte, epsi;
          
          // Solve for the set constraints and get the target variable values
          std::vector<double> vars = mpc.Solve(state, coeffs);

          // Update the control values to be sent to the simulator
          double steer_value = vars[0] /(deg2rad(25));
          double throttle_value = vars[1];
          
          json msgJson;
          

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          
          //Display the MPC predicted trajectory 
          
          std::vector<double> x_val;
          std::vector<double> y_val;
          
          for (unsigned int i = 2; i < vars.size(); i++)
          {
            // Alternate values for x and y on the trajectory
            if ( i % 2 == 0 ) {
              x_val.push_back( vars[i] );
            } else {
              y_val.push_back( vars[i] );
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = x_val;
          msgJson["mpc_y"] = y_val;
          

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
