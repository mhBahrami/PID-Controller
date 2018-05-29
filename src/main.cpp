#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include <cmath>

// for convenience
using json = nlohmann::json;
using namespace std;

// Some useful constants
const double final_speed = 25.0;
long iteration;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_last_of(']');
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;


//  std::vector<double> p = {0.1, 0.00, 0.0};
//  std::vector<double> p = {0.3, 0.00, 0.0};
//  std::vector<double> p = {0.05, 0.00, 0.0};
//  std::vector<double> p = {0.05, 0.00, 3.0};
//  std::vector<double> p = {0.05, 0.002, 3.0};
//  std::vector<double> p = {0.05, 0.005, 3.0};


//  std::vector<double> p = {0.1, 0.00, 1.0};
//  std::vector<double> p = {0.1, 0.00, 2.0};
//  std::vector<double> p = {0.1, 0.00, 5.0};
//  std::vector<double> p = {0.1, 0.00, 0.5};
//  std::vector<double> p = {0.1, 0.001, 0.5};
  std::vector<double> p = {0.1, 0.001, 1.0}; // best params so far found with twiddle on continuous loop
  PID pid_steer, pid_speed;
  ///* Initialize the pid variable.
  pid_steer.init(p[0], p[1], p[2]);
  pid_speed.init(0.20, 0.001, 2.0);

//  Twiddle twiddle;
//  iteration = 0;
//  std::vector<double> dp = {0.001, 0.001, 0.001};
//  twiddle.init(p, dp);

  h.onMessage([&pid_steer, &pid_speed/*, &twiddle*/](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));
      if (!s.empty()) {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          const double cte = stod(j[1]["cte"].get<string>());
          const double speed = stod(j[1]["speed"].get<string>());
          //const double angle = stod(j[1]["steering_angle"].get<string>());
          double steer_value, speed_value;
          /*
          * TODO: Calculate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          ///**********************************************************************************
          pid_steer.updateError(cte);
          steer_value = pid_steer.totalError();
          if (steer_value > 1.0) steer_value = 1.0;
          if (steer_value < -1.0) steer_value = -1.0;

          // se: speed error
          const double se = speed - final_speed;
          pid_speed.updateError(se);
          speed_value = pid_speed.totalError();

          iteration++;
          //cout << iteration << endl;
//          if(iteration%10==0) {
//            if(iteration == 10) {
//              cout << "****************[Start Twiddle]****************" << endl;
//            }
//            twiddle.tune(pid_steer);
//          }
//          twiddle.print();
          cout << "--> steer_value: " << steer_value << endl;
          ///**********************************************************************************

          // DEBUG
//          cout << " >> CTE: " << setw(10) << cte;
//          cout << " Steering Value: " << setw(10) << steer_value;
//          cout << " Error: " << setw(10) << pid_steer.totalError() << endl;
//          cout << " >> CTE: " << setw(10) << cte;
//          cout << " Throttle Value: " << setw(10) << steer_value;
//          cout << " Error: " << setw(10) << pid_steer.totalError() << endl << endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;//0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //cout << msg << endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
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
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    cout << "Listening to port " << port << endl;
  }
  else
  {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
