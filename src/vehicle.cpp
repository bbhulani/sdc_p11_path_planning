#include "vehicle.h"
#include <math.h>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "spline.h"
#include <iostream>

/**
 * Initializes vehicle
 */

vehicle::vehicle() {}
vehicle::~vehicle() {}

void vehicle::Init(double x, double y, double s, double d, double yaw, double speed, 
  vector<double> prev_path_x, vector<double> prev_path_y, 
  double end_s, double end_d, int prevsize, double refx, double refy, double refyaw) 
{ 
  this->car_x = x;
  this->car_y = y;
  this->car_s = s;
  this->car_d = d;
  this->car_yaw = yaw;
  this->car_speed = speed;
  this->previous_path_x = prev_path_x;
  this->previous_path_y = prev_path_y;
  this->end_path_s = end_s;
  this->end_path_d = end_d;
  this->prev_size = prevsize;
  this->ref_x = refx;
  this->ref_y = refy;
  this->ref_yaw = refyaw;

  this->state = "KL";
  //cout << "car_x=" << car_x << " car_y=" << car_y << " car_s=" << car_s
  //     << " car_d=" << car_d << " car_yaw=" << car_yaw << " car_speed=" << car_speed
  //     << " previous_path = " << prev_size << endl;
}

vector<double> vehicle::generate_predictions(vector<double> sensor_fusion) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
    return sensor_fusion;
}

// Gives the best (lowest cost) trajectory corresponding to the next state.
//void vehicle::choose_next_state(vector<double> predictions, int lane,
void vehicle::choose_next_state(int &lane, double &ref_vel, bool too_close, vector<vector<double>> sensor_fusion,
                                vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s) {
    /*
    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of vehicle objects representing
        the vehicle at the current timestep and X timesteps in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> final_states;
    vector<double> costs;
    int new_lane = lane;

    if(too_close) {
      if(ref_vel > lane_speed(sensor_fusion, lane)) {
        ref_vel -= 2*0.224;
      }
    } else if(ref_vel < 49.5) {
      ref_vel += 0.224;
    }
    // Get a list of next possible successor_states based on the current state
    vector<string> states = successor_states(lane, too_close);
    vector<vector<double>> state_x_vals;
    vector<vector<double>> state_y_vals;
    vector<int> new_lanes;
    // Loop through each next state
    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        if (it->compare("PLCL") == 0) {
          if(lane-1 >= MIN_LANES) new_lane = lane-1;
        } else if (it->compare("PLCR") == 0) {
          if(lane+1 <= MAX_LANES) new_lane = lane+1;
        } else {
          new_lane = lane;
        }

        generate_trajectory(new_lane, ref_vel, map_waypoints_x, map_waypoints_y, map_waypoints_s);
        state_x_vals.push_back(next_x_vals);
        state_y_vals.push_back(next_y_vals);

        // Caclulate cost associated with this trajectory and add it to final_trajectories
        double cost = calculate_cost(sensor_fusion, car_s, new_lane, prev_size, SPEED_LIMIT);
        //cout << " cost = " << cost << " new_lane = " << new_lane << endl;
        costs.push_back(cost);
        new_lanes.push_back(new_lane);
    }

    // Find the lowest cost index 
    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    // Update state
    this->state = states[best_idx];
    lane = new_lanes[best_idx];
    //cout << "****** Current state = " << this->state << " cost = " << costs[best_idx] << endl;

    // Update trajectory
    next_x_vals = state_x_vals[best_idx];
    next_y_vals = state_y_vals[best_idx];
}

vector<string> vehicle::successor_states(int lane, bool too_close) {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    // Vector of possible next states
    vector<string> states;

    // Next state can always be KL
    states.push_back("KL");

    // Get current state
    string state = this->state;

    if(too_close) {
      if(state.compare("KL") == 0) {
          // If current state is KL next state can be PLCL or PLCR
          states.push_back("PLCL");
          states.push_back("PLCR");
      } else if (state.compare("PLCL") == 0) {
          // If current state is PLCL, next state can be PLCL or LCL
          if (lane >= MIN_LANES) {
              states.push_back("PLCL");
              states.push_back("LCL");
          }
      } else if (state.compare("PLCR") == 0) {
          // If current state is PLCR, next state can be PLCR or LCR
          if (lane <= MAX_LANES) {
              states.push_back("PLCR");
              states.push_back("LCR");
          }
      }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

// Use this for generating my car's trajectory
// based on cur_state, lane, ref_vel etc
void vehicle::generate_trajectory(int lane, double ref_vel, vector<double> map_waypoints_x,
                                  vector<double> map_waypoints_y, vector<double> map_waypoints_s) {
  vector<double> ptsx;
  vector<double> ptsy;
  next_x_vals.clear();
  next_y_vals.clear();

  // Add 2 waypoints based on the previous path or based on the
  // cars current direction
  if(prev_size < 2)
  { 
    // Use 2 points that make the path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
  
    // Use 2 points that make the path tangent to the previous paths endpoint
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  // In frenet, add 3 evenly 30m spaced waypoints ahead of the starting 2 referene points
  vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  // Shift ptsx, ptsy to car's referene point
  for(int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
    ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
  }
  
  tk::spline s;
  s.set_points(ptsx, ptsy);
  
  
  // Push previous points that are not consumed
  for(int i = 0; i < previous_path_x.size(); i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  // Get the y coordinate for the target_x
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
  double x_add_on = 0;
  
  // Add new points based on the ref_vel and spline we created
  for(int i = 0; i < 50-previous_path_x.size(); i++)
  {
    double N = target_dist/(0.02*ref_vel/2.24);
    double x_point = x_add_on + target_x/N;
    // For the x_point get the corresponding y_point
    double y_point = s(x_point);
  
    x_add_on = x_point;
  
    double x_ref = x_point;
    double y_ref = y_point;
  
    // rotate back to normal global coordinates
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
  
    x_point += ref_x;
    y_point += ref_y;
  
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

constexpr double pi() { return M_PI; }

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> vehicle::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{ 
  int prev_wp = -1;
  
  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  { 
    prev_wp++;
  }
  
  int wp2 = (prev_wp+1)%maps_x.size();
  
  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);
  
  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
  
  double perp_heading = heading-pi()/2;
  
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  
  return {x,y};
}
