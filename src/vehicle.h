#ifndef VEHICLE_H
#define VEHICLE_H
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

#define MIN_LANES 0
#define MAX_LANES 2

class vehicle {
public:
  vehicle();
  virtual ~vehicle();

  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
  int prev_size;

  vector<double> sensor_fusion;
  vector<double> ptsx;
  vector<double> ptsy;
  // reference x, y, yaw states
  double ref_x;
  double ref_y;
  double ref_yaw;
  vector<double> next_x_vals;
  vector<double> next_y_vals; 

  string state;

  void Init(double car_x, double car_y, double car_s,
          double car_d, double car_yaw, double car_speed,
          vector<double> previous_path_x, vector<double> previous_path_y,
          double end_path_s, double end_path_d, int prev_size,
          double ref_x, double ref_y, double ref_yaw);
  //void choose_next_state(vector<double> predictions, int lane,
  void choose_next_state(int &lane, double &ref_vel, bool too_close, vector<vector<double>> sensor_fusion,
                         vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s);
  vector<string> successor_states(int lane, bool too_close);
  void generate_trajectory(int lane, double ref_vel, vector<double> map_waypoints_x,
                           vector<double> map_waypoints_y, vector<double> map_waypoints_s); 
  vector<double> generate_predictions(vector<double> sensor_fusion);
  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
};

#endif
