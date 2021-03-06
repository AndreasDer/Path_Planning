#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;

  Vehicle ego = Vehicle();
  ego.lane = lane;
  ego.ref_vel = 0.0;

  //start lane of the car
  //int lane = 1;

  //reference velocity
 // double ref_vel = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ego]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          std::map<int, Vehicle> vehicles;
          // j[1] is the data JSON object
          // Main car's localization Data
          ego.x = j[1]["x"];
          ego.y = j[1]["y"];
          ego.s = j[1]["s"];
          ego.d = j[1]["d"];
          ego.yaw = j[1]["yaw"];
          ego.speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_path_size = previous_path_x.size();
          
          for (int i = 0; i < sensor_fusion.size(); i++) {
              double obj_d = sensor_fusion[i][6];
              int lane = (int)obj_d % 4;
              int id = sensor_fusion[i][0];
              Vehicle vehicle = Vehicle(lane, sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], sensor_fusion[i][5], sensor_fusion[i][6]);
              vehicles.insert(std::pair<int, Vehicle>(id, vehicle));
          }
          
          std::map<int, vector<vector<double>>> predictions = predictTraffic(vehicles, prev_path_size);
          
          if (prev_path_size > 0) {
              ego.s = end_path_s;
          }
          
          // Choose best target state for ego vehicle
          ego.choose_next_state(predictions);
          
          json msgJson;

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = ego.x;
          double ref_y = ego.y;
          double ref_yaw = deg2rad(ego.yaw);

          // If no previous path, project current position into the past
          if (prev_path_size < 2) {
              double prev_car_x = ego.x - cos(ego.yaw);
              double prev_car_y = ego.y - sin(ego.yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(ego.x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(ego.y);
          }
          // If previous path exists, take last position and  project back from last position
          else {
              ref_x = previous_path_x[prev_path_size - 1];
              ref_y = previous_path_y[prev_path_size - 1];

              double ref_x_prev = previous_path_x[prev_path_size - 2];
              double ref_y_prev = previous_path_y[prev_path_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }
          // Add 3 more points in 30 meters distance to the the trajectory
          for (int i = 1; i < 4; i++) {
              vector<double> xy = getXY(ego.s + i * 30, (2 + 4 * ego.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(xy[0]);
              ptsy.push_back(xy[1]);
          }

          //transform to car coordinates
          for(int i = 0; i < ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // Calculate spline that uses all points in the trajectory
          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Add what is left from the previous path to the current path
          for (int i = 0; i < prev_path_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }
          // Project another point in the forward direction along the spline
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;
          // Calculate points along the spline until target point is met
          for (int i = 0; i <= 50 - prev_path_size; i++) {
              double N = (target_dist / (.02 * ego.ref_vel / 2.24));
              double x_point = x_add_on + (target_x / N);
              double y_point = s(x_point);
              x_add_on = x_point;

              // Transform back to world coordinate system
              double x_ref = x_point;
              double y_ref = y_point;

              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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