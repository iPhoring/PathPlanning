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

int main(void) {
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
  while (getline(in_map_, line)){
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

  // start in lane 1
  int lane = 1;
  //Have a reference velocity to target
  double ref_vel=0.0;//49.5; //mph

  h.onMessage([&ref_vel, &lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode){

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = hasData(data);
        if (s != "") {
            auto j = json::parse(s);
            string event = j[0].get<string>();
            if (event == "telemetry"){
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
                // of the road.
                auto sensor_fusion = j[1]["sensor_fusion"];

                /**
                * define a path made up of (x,y) points that the car will visit
                * sequentially every .02 seconds
                */
                int prev_size=previous_path_x.size(); //last path that car was following before runing thru below path calculation. simulator will pass this as s
                
                //******************************************************Prediction and Behavior*****************************************
               if(prev_size>0){car_s=end_path_s;}
               bool too_close_ahead=false;
               bool too_close_left=false;
               bool too_close_right=false;
               for(int i=0;i<sensor_fusion.size();i++){
                 float d=sensor_fusion[i][6];
                 
                 bool car_to_left = false, car_to_right = false, car_just_ahead = false;
                 if(d<(2+4*lane+2) && d>(2+4*lane-2)){
                   car_just_ahead = true;
                 } else if (d<(2+4*(lane-1)+2) && d>(2+4*(lane-1)-2)){
                   car_to_left = true;
                 } else if (d<(2+4*(lane+1)+2) && d>(2+4*(lane+1)-2)){
                   car_to_right = true;
                 }
                 

                 // find car speed
                 double vx =sensor_fusion[i][3]; 
                 double vy= sensor_fusion[i][4];
                 double check_speed=sqrt(vx*vx+vy*vy); //calculate the speed of the car in front of us
                 double check_car_s=sensor_fusion[i][5]; //find its s value

                 check_car_s +=((double)prev_size*0.02*check_speed); //if using previous points can project the s value outwards in time
                 //car is in front of as with higher s and gap between us is less than 30 meters
                 if(car_just_ahead){
                 //if(d<(2+4*lane+2) && d>(2+4*lane-2)){
                   if((check_car_s>car_s) &&((check_car_s-car_s)<30)) too_close_ahead=true;
                 } else if(car_to_left) {
                   if ((car_s - 30 < check_car_s) && (car_s + 30 > check_car_s)) too_close_left=true;
                 } else if(car_to_right){
                   if ((car_s - 30 < check_car_s) && (car_s + 30 > check_car_s)) too_close_right=true;
                 }

                 
                 
               } // if sensor_fusion
              
               //Behavior module
               if (too_close_ahead){
                  if ((!too_close_left) && (lane > 0)){
                    lane--;
                  } else if ((!too_close_right) && (lane !=2)){
                    lane++;
                  } else ref_vel -=.224;
               } else if(ref_vel <49.5){
                 ref_vel +=.224;
               } //(too_close_ahead)
                //******************************************************Prediction and Behavior module ends*****************************
              
                //Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
                vector<double> pointsx;
                vector<double> pointsy;
                // reference x,y,yaw states
                // either it will be the starting point of the car or at the previous path end point
                double ref_x=car_x;
                double ref_y=car_y;
                double ref_yaw=deg2rad(car_yaw);

                if(prev_size<2){
                    double prev_car_x=car_x -cos(car_yaw);
                    double prev_car_y=car_y -sin(car_yaw);
                    
                    pointsx.push_back(prev_car_x);
                    pointsx.push_back(car_x);
                    
                    pointsy.push_back(prev_car_y);
                    pointsy.push_back(car_y);
                } else{
                    //Last couple of points in the previous path that the car was following 
                    //and use that to calculate what angle the car was heading using last couple of points
                    //then pusing these newly calculated points to previous point vector
                    
                    // Redefine reference state as previous path end points
                    ref_x=previous_path_x[prev_size-1];
                    ref_y=previous_path_y[prev_size-1];
                    
                    double ref_x_prev=previous_path_x[prev_size-2];
                    double ref_y_prev=previous_path_y[prev_size-2];
                    ref_yaw=atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
                    
                    // Use two points that make the path tangent to the previous path's end point
                    pointsx.push_back(ref_x_prev);
                    pointsx.push_back(ref_x);
                    
                    pointsy.push_back(ref_y_prev);
                    pointsy.push_back(ref_y);
                } // (prev_size<2) adding two points

                // find more points using a technique as stated in classroom
                // In frenet odd even,y 30m spaced points ahead of the starting reference
                vector<double> next_wp0=getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                vector<double> next_wp1=getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                vector<double> next_wp2=getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                
                // adding three more points spaced at 0.5 meters
                pointsx.push_back(next_wp0[0]);
                pointsx.push_back(next_wp1[0]);
                pointsx.push_back(next_wp2[0]);
                
                pointsy.push_back(next_wp0[1]);
                pointsy.push_back(next_wp1[1]);
                pointsy.push_back(next_wp2[1]);
                
                // now we have two previous points and location of car at 30 meters, 60 and 90 meters
                // chaging the reference angle to car angle so that it is zero degrees
                // this is down to make the math and viewing logic eazy
                // tranformation to local car;s coordinates
                for (int i=0;i<pointsx.size();i++){
                    double shift_x=pointsx[i]-ref_x;
                    double shift_y=pointsy[i]-ref_y;
                    pointsx[i]=(shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw)); //pls note the change in angle
                    pointsy[i]=(shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw)); //pls note the change in angle
                }
                
                //create a spline lib reference
                tk::spline s;

                // set (x,y) points to the spline
                s.set_points(pointsx,pointsy); // adding anchor points to the spline
          
                //define the actual (x,y) points that car will be using for the path planning
                vector<double> next_x_vals; //future points
                vector<double> next_y_vals; // future points 
                
                //starting from our good known previous path that car was following
                // so far we have pushed 5 points to previous path vector
                for (int i=0;i<previous_path_x.size();i++){
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                }

                //calculate how to break up spline points
                // so that we travel at our desired reference  velocity
                double target_x=30.0; // our horizon going out 30m
                double target_y=s(target_x); //asking the spline 
                double target_dist=sqrt((target_x)*(target_x)+(target_y)*(target_y));
                double x_add_on=0; //starting at the origin for transformtion

                //fill up the rest of our path planner after filling it with 
                // previous points, here we will always output 50 points
                for (int i=0;i< 50-previous_path_x.size();i++){
                    double N=(target_dist/(0.02*ref_vel/2.24)); // 2.24 is for MPH; to covert to meters per second
                    double x_point=x_add_on+(target_x)/N;
                    double y_point=s(x_point);
                    
                    x_add_on=x_point;
                    
                    double x_ref=x_point;
                    double y_ref=y_point;
                    
                    // rotate back to global coordinates as we are car local 
                    x_point=(x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
                    y_point=(x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
                    
                    x_point += ref_x;
                    y_point += ref_y;
                    
                    next_x_vals.push_back(x_point);
                    next_y_vals.push_back(y_point);
                } // 50-previous_path_x.size()

                json msgJson;
                msgJson["next_x"] = next_x_vals;
                msgJson["next_y"] = next_y_vals;
                auto msg = "42[\"control\","+ msgJson.dump()+"]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            } // end "telemetry" if
        }
    } // if (length && length) -- end websocket if
  });// end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {std::cout << "Connected!!!" << std::endl;});

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,char *message, size_t length) {ws.close();std::cout << "Disconnected" << std::endl;});

  int port = 4567;

  if (h.listen(port)){
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

} // int main(void) closure