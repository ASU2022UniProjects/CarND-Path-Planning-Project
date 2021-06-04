#include <math.h>
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "float.h"

#include "planner.h"

using namespace std;

using json = nlohmann::json;


double predictPosOfCarInLane(double vx, double vy, double obj_s, int prev_path_size){
  double obj_speed = sqrt(vx * vx + vy * vy);

  return obj_s + (0.02 * obj_speed * prev_path_size);
}

bool checkLaneSafety(SensorFusion *car_behind, SensorFusion *car_ahead,int lane_diff,int prev_path_size, double car_s){
  double car_ahead_pos =  predictPosOfCarInLane(car_ahead->vx , car_ahead->vy, car_ahead->s, prev_path_size);
  double car_behind_pos = predictPosOfCarInLane(car_behind->vx, car_behind->vy, car_behind->s, prev_path_size);

  return ((car_ahead_pos > car_s) && ((car_ahead_pos - car_s) > 15.0) && (car_behind_pos < car_s) && (abs(car_behind_pos - car_s) > 15.0));
}

vector<double> checkAheadBehind(int lane_to_check, vector<vector<SensorFusion>> cars_ahead, vector<vector<SensorFusion>> cars_behind, double car_s, int curr_lane, int prev_path_size)
{
  double min_dist_ahead = 10000;
  double max_dist_behind = 10000;
  SensorFusion *car_behind;
  SensorFusion *car_ahead;

  int new_lane = lane_to_check;
  for (int j = 0; j < cars_ahead[lane_to_check].size(); j++)
  {
    SensorFusion *car = (SensorFusion *)&cars_ahead[lane_to_check][j];
    int d = car->d;
    if ((d < (4 * new_lane + 4)) && (d > (4 * new_lane)))
    {
      double obj_dst = car->s;

      // printf("Car ahead in center at distance = %f\n", (obj_dst - car_s));
      if ((obj_dst - car_s) < min_dist_ahead)
      {
        min_dist_ahead = (obj_dst - car_s);
        car_ahead = car;
      }
    }
  }

  for (int j = 0; j < cars_behind[lane_to_check].size(); j++)
  {
    SensorFusion *car = (SensorFusion *)&cars_behind[lane_to_check][j];
    int d = car->d;

    if ((d < (4 * new_lane + 4)) && (d > (4 * new_lane)))
    {
      double obj_dst = car->s;

      // printf("Car behind in center at distance = %f\n", (car_s - obj_dst));
      if ((car_s - obj_dst) < max_dist_behind)
      {
        max_dist_behind = (car_s - obj_dst);
        car_behind = car;
      }
    }
  }
  return {cars_ahead[lane_to_check].size(), cars_behind[lane_to_check].size(), min_dist_ahead, max_dist_behind, (double)checkLaneSafety(car_behind,car_ahead,abs(curr_lane-lane_to_check),prev_path_size,car_s)};
}




int costFunction(vector<vector<double>> lanes_data, int curr_lane){
  int best_lane = curr_lane;
  double lowest_cost = DBL_MAX;
  for(int i = 0; i < lanes_data.size(); ++i){
    int no_of_cars_ahead = lanes_data[i][0];
    int no_of_cars_behind = lanes_data[i][1];
    double min_dist_ahead = lanes_data[i][2];
    double max_dist_behind = lanes_data[i][3];
    bool lane_safe = (bool)(int)lanes_data[i][4];

    double curr_lane_cost = no_of_cars_ahead*60 + no_of_cars_behind*15 - 8*min_dist_ahead - max_dist_behind*(i!=curr_lane) - 50*(i==1);

    cout << "Lane: " << i << "    Cost: "<< curr_lane_cost << "      Safety: "<< lane_safe << endl;
    cout <<  "Max Distance: " << max_dist_behind<<endl;
    cout <<  "Min Distance: " << min_dist_ahead<<endl;
    if(curr_lane_cost < lowest_cost && lane_safe){
      best_lane = i;
      lowest_cost = curr_lane_cost;
    }
  }
  cout << "Current lane: "<< curr_lane << endl;
  cout << "Best lane: "<< best_lane << endl;
  cout << "Middle lane safety: "<< (bool)(int)lanes_data[1][4] << endl;
  cout << "Best lane safety: "<< (bool)(int)lanes_data[best_lane][4] << endl;
  if(abs(curr_lane-best_lane)>1){
    if((bool)(int)lanes_data[1][4] == true){
      best_lane = 1;
    }
    else{
      best_lane = curr_lane;
    }
  }

  return best_lane;

}
int lane = 1;              //0-left lane, 1-middle lane, 2-right lane
double ref_velocity = 0.0; //MPH

int main()
{
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    istringstream iss(line);
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode)
              {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2')
                {

                  auto s = hasData(data);

                  if (s != "")
                  {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                      // j[1] is the data JSON object

                      // Main car's localization Data
                      double car_x = j[1]["x"];
                      double car_y = j[1]["y"];
                      double car_s = j[1]["s"];
                      double car_d = j[1]["d"];
                      double car_yaw = j[1]["yaw"];
                      double car_speed = j[1]["speed"];

                      // Previous path data given to the Planner
                      vector<double> previous_path_x = j[1]["previous_path_x"];
                      vector<double> previous_path_y = j[1]["previous_path_y"];
                      // Previous path's end s and d values
                      double end_path_s = j[1]["end_path_s"];
                      double end_path_d = j[1]["end_path_d"];

                      // Sensor Fusion Data, a list of all other cars on the same side
                      //   of the road.
                      auto sensor_fusion = j[1]["sensor_fusion"];

                      int prev_path_size = previous_path_x.size();

                      if (prev_path_size > 0)
                      {
                        car_s = end_path_s;
                      }

                      bool lane_change = false;
                      double max_velocity = 49.5;

                      vector<vector<SensorFusion>> cars_ahead(3);

                      vector<vector<SensorFusion>> cars_behind(3);

                      for (int i = 0; i < sensor_fusion.size(); i++)
                      {
                        SensorFusion car;

                        car.id = sensor_fusion[i][0];
                        car.x = sensor_fusion[i][1];
                        car.y = sensor_fusion[i][2];
                        car.vx = sensor_fusion[i][3];
                        car.vy = sensor_fusion[i][4];
                        car.s = sensor_fusion[i][5];
                        car.d = sensor_fusion[i][6];

                        double obj_speed = sqrt(car.vx * car.vx + car.vy * car.vy);
                        double sensor_range = 400.0;

                        if (((car.s - car_s) >= 0.0) && ((car.s - car_s) < sensor_range))
                        {
                          cars_ahead[car.d / 4].push_back(car);
                        }
                        else if (((car.s - car_s) < 0.0) && ((car.s - car_s) > -sensor_range))
                        {
                          cars_behind[car.d / 4].push_back(car);
                        }

                        if ((car.d < (4 * lane + 4)) && (car.d > (4 * lane)))
                        {

                          double obj_s_future = predictPosOfCarInLane(sensor_fusion[i][3],sensor_fusion[i][4],sensor_fusion[i][5],prev_path_size);

                          if ((obj_s_future > car_s) && (abs(obj_s_future - car_s) < 17.0))
                          {
                            lane_change = true;
                            max_velocity = obj_speed;
                          }
                        }
                      }

                      //printf("\033c");
                      // printf("lane  - %d ", lane);
                      // printf("speed - %f ", ref_velocity);
                      // printf("Ahead - ");
                      // printf("|%3d|%3d|%3d| ", cars_ahead[0].size(), cars_ahead[1].size(), cars_ahead[2].size());
                      // printf("Behind - ");
                      // printf("|%3d|%3d|%3d| ", cars_behind[0].size(), cars_behind[1].size(), cars_behind[2].size());
                      // printf("Max vel - %5.2f", max_velocity);
                      // printf("\n");

                      //TODO Stay on max car ahead velocity if decelerating
                      if (ref_velocity < max_velocity)
                      {
                        ref_velocity += MAX_SPEED_INC;
                      }
                      else if (ref_velocity > max_velocity)
                      {
                        ref_velocity -= MAX_SPEED_DEC;
                      }
                      int lane_change_diff;
                      if (lane_change == true)
                      {
                        vector<vector<double>> lanes_data;
                        printf("Try Lane Change!\n");

                        //Safe distance change to constant
                        // printf("SAFE_DIST = %f\n", SAFE_DIST);

                        //Get all lanes data
                        for(int i = 0; i < 3; ++i){
                          lanes_data.push_back(checkAheadBehind(i,cars_ahead,cars_behind,car_s,lane, prev_path_size));
                        }
                        //Check if adjacent lane is empty
                        lane = costFunction(lanes_data, lane);

                        printf("Chosen lane %d\n", lane);
                      }

                      vector<double> pts_x;
                      vector<double> pts_y;

                      vector<double> refs = getInitialPoints(pts_x, pts_y,
                                                             previous_path_x, previous_path_y,
                                                             prev_path_size,
                                                             car_x, car_y, car_yaw);
                      double ref_x = refs[0];
                      double ref_y = refs[1];
                      double ref_yaw = refs[2];

                      setNextWayPoints(pts_x, pts_y,
                                       car_s, lane,
                                       map_waypoints_s, map_waypoints_x, map_waypoints_y);

                      shiftPlane(pts_x, pts_y,
                                 ref_x, ref_y, ref_yaw);

                      vector<double> next_x_vals;
                      vector<double> next_y_vals;

                      for (int i = 0; i < prev_path_size; i++)
                      {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                      }

                      pushFuturePoints(pts_x, pts_y,
                                       next_x_vals, next_y_vals,
                                       prev_path_size, ref_velocity,
                                       ref_x, ref_y, ref_yaw);

                      json msgJson;
                      msgJson["next_x"] = next_x_vals;
                      msgJson["next_y"] = next_y_vals;

                      auto msg = "42[\"control\"," + msgJson.dump() + "]";

                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    } // end "telemetry" if
                  }
                  else
                  {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                } // end websocket if
              }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                 { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
                    {
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