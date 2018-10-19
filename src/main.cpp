#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>


// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int reset_time = 0;
int cte_r = 0; // aux cte variable

// Resetting the Simulator
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


class Twiddle {
public:
  vector<double> best_params, p, dp;
  int num_params, idx;
  double best_err, avg_err, err;
  bool is_using, is_initialized;
  bool flag_fw, flag_bw;

  Twiddle();
  virtual ~Twiddle();
  void Init(double kp_, double ki_, double kd_);
  void Update();
};

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double kp_, double ki_, double kd_) {
  is_using = true;

  flag_fw = false;
  flag_bw = false;

  num_params = 3;

  best_err = 999;
  err = 0;
  avg_err = 0;
  idx = 0;

  p = {kp_, ki_, kd_};
  dp = {1.0, 1.0, 1.0};

  is_initialized = false;
}

void Twiddle::Update() {

  // initialize
  if (!is_initialized) {
    best_err = err;
    flag_fw = true;
    flag_bw = false;

    // the first step: update kp gain
    p[idx] += dp[idx];
    is_initialized = true;
  }
  else {
    if (err < best_err) {
      best_err = err;
      dp[idx] *= 1.1;
      // switch into the next pid param
      idx = (idx + 1) % 3;

      flag_fw = true;
      flag_bw = false;
    }
    else {
      if (flag_fw) {

        p[idx] -= 2 * dp[idx];
        flag_fw = false;
        flag_bw = true;
      }
      else {
        p[idx] += dp[idx];
        dp[idx] *= 0.9;
        idx = (idx + 1) % 3;

        flag_fw = true;
        flag_bw = false;
      }
    }
    if (flag_fw) {
      p[idx] += dp[idx];
    }
  }
}


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


int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(0.1, 0.01, 1);

  Twiddle tw;
  tw.Init(0.1, 0.01, 1);

  h.onMessage([&pid, &tw](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid.UpdateError(cte);
          steer_value = pid.Controller();

          // limit steer value to -1,1 range
          steer_value = steer_value < -1 ? -1 : steer_value;
          steer_value = steer_value > 1 ? 1 : steer_value;

          //remove high disturbance at the very beginning
          if (pid.steps > 5) {
            pid.TotalError();
            cte_r = cte;
          }

          // DEBUG
          // std::cout << "CTE: " << cte << "\tSteering Value: " << steer_value << " \tstep counter: " << pid.steps << std::endl;
          // cout << "pid steps: " << pid.steps << endl;

          //reset simulator for twiddle params tuning
          if ((pid.steps > 2000 || fabs(cte_r) > 3 ) & tw.is_using) {

            //assign large error if the car out of road
            tw.avg_err = pid.total_err / pid.steps;
            if (fabs(cte) > 3) {tw.err = tw.avg_err + 100.0;}
            else {tw.err = tw.avg_err;}

            //unactive twiddle if performance is good enough
            if (tw.err < 0.001) {tw.is_using = false;}

            //update pid params using twiddle
            tw.Update();

            pid.Kp = tw.p[0];
            pid.Ki = tw.p[1];
            pid.Kd = tw.p[2];
            cout << "---------------------------" << endl;
            cout << "Reset Simulator. Times: " << (reset_time += 1) << endl;
            cout << "---------------------------" << endl;
            cout << "kp: " << tw.p[0] << "\tki: " << tw.p[1] << "\tkd: " << tw.p[2];
            cout << "\tdkp: " << tw.dp[0] << "\tdki: " << tw.dp[1] << "\tdkd: " << tw.dp[2] << endl;
            cout << "Best err: " << tw.best_err << "\tcurrent err: " << tw.err << "\tcurrent idx: " << tw.idx << endl;

            pid.total_err = 0;
            pid.steps = 0;

            reset_simulator(ws);
            cte = 0;
            cte_r = 0;
            sleep(0.5); // wait for reseting sim
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
  h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data, size_t, size_t) {
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
