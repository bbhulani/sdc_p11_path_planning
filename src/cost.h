#ifndef COST_H
#define COST_H
#include <vector>

using namespace std;

#define VEHICLE_RADIUS 30
#define SPEED_LIMIT 49.5

double distance_to_nearest_vehicle(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size);
double lane_speed(vector<vector<double>> sensor_fusion, int intended_lane);

double inefficiency_cost(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size, double target_speed);
double collision_cost(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size, double target_speed);
double buffer_cost(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size, double target_speed);
double calculate_cost(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size, double target_speed);

#endif
