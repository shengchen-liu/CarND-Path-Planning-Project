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

  //Reference velocity.
  double ref_vel = 0; // mph

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          	//Car's lane. Stating at middle lane.
          int lane = 1;


          double speed_diff = .224;
          const double max_accel = 49.5;

          int prev_size = previous_path_x.size();

          //PREDICTION

          /***
          The prediction component estimates what actions other objects might take in the future. For example, if another vehicle were identified, the prediction component would estimate its future trajectory.
          ***/

          /*
            In this example prediction module we use to find out following
            car ahead is too close, car on the left is too close, and car on the right is too close.
            As explained actual prediction module will be implementd using the apparoach mentioned above, but this highway project
            doesnt need to predict the trajectory of each vehicle as those vehicles trajectory will be on the straight lane.
          */

          if(prev_size > 0) {
            car_s = end_path_s;
          }

          bool car_left= false;
          bool car_right = false;
          bool car_ahead = false;
          for(int i=0; i < sensor_fusion.size(); i++) {
            
            float d = sensor_fusion[i][6];

            int check_car_lane;

            /*Currently we assume that we have only three lanes and each lane has 4 meter width. In actual scenarion, 
            number of lanes an ddistance between the lanes and total lanes distance can be detected using computer vision 
            technologies. We slightly touched in advanced lane findings in term1. 
            */
            if(d > 0 && d < 4) {
              check_car_lane = 0;
            } else if(d > 4 && d < 8) {
              check_car_lane = 1;
            } else if(d > 8 and d < 12) {
              check_car_lane = 2;
            } 	
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];	

            //This will help to predict the where the vehicle will be in future
            check_car_s += ((double)prev_size*0.02*check_speed);
            if(check_car_lane == lane) {
              //A vehicle is on the same line and check the car is in front of the ego car
              car_ahead |= check_car_s > car_s && (check_car_s - car_s) < 30;										

            } else if((check_car_lane - lane) == -1) {
              //A vehicle is on the left lane and check that is in 30 meter range
              car_left |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;

            } else if((check_car_lane - lane) == 1) {
              //A vehicle is on the right lane and check that is in 30 meter range
              car_right |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;
            
            }
          }

          // //As we said, actual prediction module gives the possible trajectories from the current timeline to the future of each vehicle. 
          // //In this highway exmaple, we will have only one possible trajectory for each vehicle and that is why we are using simple approach as above.
          // //In complex situation we may need to use model, data, or hybrid approach for prdiction module
          
          // //BEHAVIOUR
          // /***
          // The behavioral planning component determines what behavior the vehicle should exhibit at any point in time. 
          // For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component.
          // ***/
          if(car_ahead) {
            // std::cout<< "car ahead!" << std::endl;
            if(!car_left && lane > 0) { // if there is no car in the left and we are not on the left lane
              lane--;
              std::cout << "change lane left!"<< std::endl;;
            } else if(!car_right && lane !=2) { // if there is no car in the right and we are not in the rightmost lane
              lane++;
              std::cout << "change lane right!"<< std::endl;;;
            }else {
              // cannot pass, we have to slow down
              ref_vel -= speed_diff;
              std::cout << "slow down!"<< std::endl;;
            }
          } else {
            // no car ahead
            if ( lane != 1 ) { // if we are not on the center lane.
              if ( ( lane == 0 && !car_right ) || ( lane == 2 && !car_left ) ) {
                lane = 1; // Back to center.
                std::cout << "Back to Center!"<< std::endl;
              }
            }
            if(ref_vel < max_accel){
              ref_vel += speed_diff;
              std::cout<<"accelrate!"<< std::endl;
            }
          }
          std::cout << ""<< std::endl;;
          

          // create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous path is almost zero, use the car as the starting reference
          if ( prev_size < 2 ) {
              //Use two points thats makes path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          }
          // use previous path's end points as the staring reference
          else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];

            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of starting reference
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // transfomation to set the car's reference frame
          for (int i = 0; i < ptsx.size(); ++i) {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x; // 
            double shift_y = ptsy[i] - ref_y; 

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // create a spline
          tk::spline s;

          // set x,y points to the spline
          s.set_points(ptsx, ptsy); // anchor points

          // start with all of the previous points from last time
          for (int i = 0; i < prev_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so that we travel at our desired referenced velocity
          double target_x = 30.0;
          double target_y = s(target_x); 
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          // fill out the rest of our path planner after filling it with previous poionts, here we have 50 points
          for( int i = 1; i < 50 - prev_size; i++ ) {
                        
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);
              x_add_on = x_point;
              double x_ref = x_point;
              double y_ref = y_point;
              //Rotate back to normal after rotating it earlier
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
              x_point += ref_x;
              y_point += ref_y;
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }


          //  end
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