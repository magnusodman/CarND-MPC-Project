#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Geometry"
#include "MPC.h"
#include "json.hpp"

// for convenience
double LATENCY = 0.1;
double ACCELERATION_THROTTLE_RATIO = 1.0;
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

tuple<double, double > transformCoordinate(double x, double y, double psi, double x_o, double y_o) {
  double x_t = x-x_o;
  double y_t = y-y_o;

  double x_r = x_t * cos(-psi) - y_t * sin(-psi);
  double y_r = x_t * sin(-psi) + y_t * cos(-psi);

  return std::make_tuple(x_r, y_r);
};

tuple<Eigen::VectorXd, Eigen::VectorXd> transformToVehicleCoordinates(vector<double, allocator<double>> ptsx, vector<double, allocator<double>> ptsy,
                                                                      double psi, double x_o, double y_o) {

  std::vector<double> transformedX;
  std::vector<double> transformedY;

  for(int index = 0; index < ptsx.size(); index++) {
    auto coordinates = transformCoordinate(ptsx[index], ptsy[index], psi, x_o, y_o);
    double x_t = std::get<0>(coordinates);
    double y_t = std::get<1>(coordinates);
    transformedX.push_back(x_t);
    transformedY.push_back(y_t);

  }

  return std::forward_as_tuple(Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(transformedX.data(), transformedX.size()), Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(transformedY.data(), transformedY.size()));

};

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

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
          double mph2ms = 0.447;
          double v = j[1]["speed"]; //* mph2ms;
          v = v * mph2ms;
          double delta = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];
          double Lf = 2.67;


          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          std::tuple<Eigen::VectorXd, Eigen::VectorXd> transformedCoordinates = transformToVehicleCoordinates(ptsx, ptsy,psi, px, py);


          Eigen::VectorXd ptsxVector = std::get<0>(transformedCoordinates);
          Eigen::VectorXd ptsyVector = std::get<1>(transformedCoordinates);

          std::cout << "Transformed xs: " << ptsxVector << std::endl;
          std::cout << "Transformed ys: " << ptsyVector << std::endl;



          auto coeffs = polyfit(ptsxVector, ptsyVector, 3);
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          std::cout << "CTE: " << cte << std::endl;

          Eigen::VectorXd  state(6);

          /**
           * To compensate for latency we calculate the state at time now + latency
           */
          state(0) = v * LATENCY;
          state(1) = 0;
          state(2) = 0 - v * delta * LATENCY /  Lf;
          state(3) = v + throttle * ACCELERATION_THROTTLE_RATIO * LATENCY; //* 0,44704;
          state(4) = cte + v * sin(epsi) * LATENCY;
          state(5) = epsi - v * delta * LATENCY / Lf;

          auto solution = mpc.Solve(state, coeffs);
          std::cout << "Solution: " << solution[0] << ", " << solution[1] << ", " << solution[2] << ", " << solution[3] << ", " << solution[4] << ", " << solution[5] << ", " << solution[6] << ", " <<solution[7] << ", " << std::endl;
          double steer_value = solution[6];
          double throttle_value = solution[7];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          mpc_x_vals = mpc.GetTrajectoryX();
          mpc_y_vals = mpc.GetTrajectoryY();
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line


          for(int i = 1; i < 20; i++) {
            double next_y = polyeval(coeffs, 3*i);
            next_x_vals.push_back(double(3*i));
            next_y_vals.push_back(next_y);
          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
