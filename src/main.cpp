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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

template<typename T>
void printVec(vector<T> v) {
  for(T &x: v) {
    std::cout << x << ", " ;
  }
  std::cout << std::endl;
}

bool isValidLane(int laneId) {
  return (laneId >= 0) && (laneId < 3);
}

int getLane(double d) {
  return static_cast<int>(floor(d/4));
}

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
  constexpr double speed_limit = 21; // meters per second
  constexpr double frame_period = 0.02; // seconds
  constexpr double disp_frame = speed_limit * frame_period;
  constexpr double speed_quantum{0.15};
  double ego_vel {0.0};
  int ego_lane{1};
  bool too_close{false};

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s, speed_limit, frame_period, disp_frame, speed_quantum,
               &map_waypoints_dx,&map_waypoints_dy,&ego_vel, &ego_lane, &too_close]
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
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          auto path_size = previous_path_x.size();
          auto next_s = (path_size > 0) ? end_path_s : car_s; // starting s value for the trajectory planner
          auto next_d = car_d; // // starting d value for the trajectory planner
          vector<double> way_points_x; // widely spaced way-points to determine path (x co-ordinates)
          vector<double> way_points_y; // widely spaced way-points to determine path (y co-ordinates)
          
          // reference point (for determining intermediate points)
          auto ref_x = car_x;
          auto ref_y = car_y;
          auto ref_yaw = deg2rad(car_yaw);
          auto prev_car_x = car_x - cos(car_yaw);
          auto prev_car_y = car_y - sin(car_yaw);

          // triggers for lane change and collision avoidance
          too_close = false;
          bool left_lane_available = isValidLane(ego_lane-1);
          bool right_lane_available = isValidLane(ego_lane+1);
          for(auto &tv: sensor_fusion) {
            double tv_d = tv[6];
            double tv_s = tv[5];
            double speed = distance(0, 0, tv[3], tv[4]); 
            // project target vehicle location
            double proj = tv_s + speed * frame_period * path_size; 
            if( (tv_d > (4*ego_lane)) && (tv_d < (4*(ego_lane + 1)))) {
              // Target vehicle is in the same lane as ego vehicle
              if( (proj > next_s) && ((proj - next_s) < 30) ) {
                // Collision detected
                too_close = true;
              }
            } else {
              if(isValidLane(ego_lane-1) && (getLane(tv_d) == ego_lane-1) && (abs(proj-next_s) < 30) ) {
                // target vehicle blocks left lane. maybe checking forward direction alone is sufficient ?
                left_lane_available = false;
              }
              if(isValidLane(ego_lane+1) && (getLane(tv_d) == ego_lane+1) && (abs(proj-next_s) < 30) ) {
                // target vehicle blocks right lane. maybe checking forward direction alone is sufficient ?
                right_lane_available = false;
              }
            }
          }
          
          if(too_close) {
            ego_vel = std::max(speed_quantum, ego_vel-speed_quantum);
            if(ego_vel < 0.75*speed_limit) { // avoid jerk from turns... 
              ego_lane = left_lane_available ? ego_lane-1 : (right_lane_available ? ego_lane+1 : ego_lane);
            }
          } else {
            ego_vel = std::min(speed_limit, ego_vel+speed_quantum);
          }
          next_d = ego_lane * 4 + 2;
          
          // if not many points left from previous path...
          if(path_size < 2) {
            // ... start with car location as reference.
            way_points_x.push_back(prev_car_x);
            way_points_x.push_back(car_x);
            
            way_points_y.push_back(prev_car_y);
            way_points_y.push_back(car_y);
          } else {
            // ... else, use the end of path as reference.
            prev_car_x = previous_path_x[path_size - 2];
            prev_car_y = previous_path_y[path_size - 2];
            
            ref_x = previous_path_x[path_size-1];
            ref_y = previous_path_y[path_size-1];
            ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
            
            way_points_x.push_back(prev_car_x);
            way_points_x.push_back(ref_x);
            
            way_points_y.push_back(prev_car_y);
            way_points_y.push_back(ref_y);
          }
          
          // create anchor points at s=30, 60, 90 on the desired lane
          auto p1 = getXY(next_s+30, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          auto p2 = getXY(next_s+60, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          auto p3 = getXY(next_s+90, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          way_points_x.push_back(p1[0]);
          way_points_x.push_back(p2[0]);
          way_points_x.push_back(p3[0]);
          
          way_points_y.push_back(p1[1]);
          way_points_y.push_back(p2[1]);
          way_points_y.push_back(p3[1]);
          // calculate x, y for these anchor points in car-local co-ordinates. 
          for(int i = 0; i < way_points_x.size(); ++i) {
            auto sx = way_points_x[i] - ref_x;
            auto sy = way_points_y[i] - ref_y;
            
            way_points_x[i] = (sx*cos(0-ref_yaw) - sy*sin(0-ref_yaw));
            way_points_y[i] = (sx*sin(0-ref_yaw) + sy*cos(0-ref_yaw));
          }
          
          // use intermediate points to make motion smooth. Use spline to calculate trajectory.
          tk::spline s;
          s.set_points(way_points_x, way_points_y);
          
          // we use a triangle approximation of the spline to calculate the step-size in the 
          // x-direction for each point in the trajectory.
          double target_x{30};
          double target_y = s(target_x);
          double target = distance(0, 0, target_x, target_y);
          double num_steps = target/(frame_period*ego_vel);
          double delta_x = 0.0;
          // Populate next_points.          
          if(path_size > 0) {
            // ... beginning with points left over from previous plan
            next_x_vals.assign(std::begin(previous_path_x), std::end(previous_path_x));
            next_y_vals.assign(std::begin(previous_path_y), std::end(previous_path_y));
          }
          // transform from car co-ordinates to world co-ordinates and add to next_x...
          for(int i = path_size; i<50; ++i) {
            // upto a maximum of 50 points.
            delta_x += target_x/num_steps;
            auto newy = s(delta_x);
            // rotate and shift co-ordinates.
            double x_point = delta_x*cos(ref_yaw) - newy*sin(ref_yaw) + ref_x;
            double y_point = delta_x*sin(ref_yaw) + newy*cos(ref_yaw) + ref_y;
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