#include "cost.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include <iostream>

const double COLLISION_COST = 1000;
const double BUFFER_COST = 100;
const double EFFICIENCY = 10;

double inefficiency_cost(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size, double target_speed) 
{
  /*
  Cost becomes higher for trajectories with intended lane that has traffic slower than vehicle's target speed. 
  */
  double ln_speed = lane_speed(sensor_fusion, intended_lane);
  if (ln_speed < 0) {
      ln_speed = target_speed;
  }
  double cost = (target_speed - ln_speed)/target_speed;
  return cost;
}

double lane_speed(vector<vector<double>> sensor_fusion, int intended_lane)
{ 
  double lane_speed;
  int k; 
  for(k = 0; k < sensor_fusion.size(); k++)
  {
    float d1 = sensor_fusion[k][6];
    double vx1 = sensor_fusion[k][3];
    double vy1 = sensor_fusion[k][4];
    double check_speed1 = sqrt(pow(vx1,2)+pow(vy1,2));
    if (d1 < (2+4*intended_lane+2) && d1 > (2+4*intended_lane-2))
    {
      lane_speed += check_speed1;
    }
  }
  lane_speed = lane_speed/k;
  return lane_speed;
}

// Find the distance to the nearest vehicle in the intended_lane
double distance_to_nearest_vehicle(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size) 
{
  vector<double> car_distances;

  for(int k = 0; k < sensor_fusion.size(); k++)
  {
    float d1 = sensor_fusion[k][6];
    double vx1 = sensor_fusion[k][3];
    double vy1 = sensor_fusion[k][4];
    double check_speed1 = sqrt(pow(vx1,2)+pow(vy1,2));
    double check_car_s1 = sensor_fusion[k][5];
    check_car_s1 += ((double)prev_size * 0.02 * check_speed1);
    if (d1 < (2+4*intended_lane+2) && d1 > (2+4*intended_lane-2))
    {
      car_distances.push_back(abs(check_car_s1 - car_s));
      //cout << "Car distance = " << car_distances[k] << endl;
    }
  }
  if(car_distances.size()) {
    vector<double>::iterator closest_car = min_element(begin(car_distances), end(car_distances));
    int closest_idx = distance(begin(car_distances), closest_car);
    //cout << "Car distance = " << car_distances[closest_idx] << endl;
    return car_distances[closest_idx];
  } else {
    return 0;
  }
}


double collision_cost(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size, double target_speed)
{
    // Binary cost function which penalizes collisions.
    double nearest = distance_to_nearest_vehicle(sensor_fusion, car_s, intended_lane, prev_size);
    if ((nearest < VEHICLE_RADIUS) && nearest)
      return 1.0;
    else
      return 0.0;
}

double buffer_cost(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size, double target_speed)
{
    // Penalizes getting close to other vehicles.
    double nearest = distance_to_nearest_vehicle(sensor_fusion, car_s, intended_lane, prev_size);
    if(nearest)
      return 1 - 2*exp(-(2*VEHICLE_RADIUS/nearest));
    else
      return 0;
}


double calculate_cost(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, int prev_size, double target_speed) 
{
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    double cost = 0.0;

    //Add additional cost functions here.
    vector<function<double(vector<vector<double>>, double, int, int, double)>> cf_list = {collision_cost, buffer_cost, inefficiency_cost};
    vector<double> weight_list = {COLLISION_COST, BUFFER_COST, EFFICIENCY};
    
    for (int i = 0; i < cf_list.size(); i++) {
        double new_cost = weight_list[i]*cf_list[i](sensor_fusion, car_s, intended_lane, prev_size, target_speed);
        cost += new_cost;
    }

    return cost;

}
