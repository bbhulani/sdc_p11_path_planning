# CarND-Path-Planning-Project
Model Documentation
   
### Overview
The Path Planning model at high level comprises of:
- Finite State Machine (FSM) for lane changing
- Trajectory Generation
- Cost function analysis for choosing a lane

The model I've created does not include generating predictions based on other (i.e. non-ego) vehicle's trajectories. That would be nice to have but I ran out of time to implement that. 

### FSM for lane changing
I used a very simple FSM that had 5 states and decisions were made based on if the ego vehicle was too close to the vehicle in front and its current state. The 5 states were Keep Lane (KL), Prepare Lane Change Left (PLCL), Prepare Lane Change Right (PLCR), Lane change left (LCL), Lane change right (LCR). If the ego vehicle got too close to a vehicle in front, it would evaluate the cost of changing lanes right or left in PLCR/PLCL or keeping its current lane. Without generating predictions based on the non-ego vehicle's trajectories I could have probably just used 3 states, but I designed it with the expectation of using predictions. For each of the successor states a corresponding trajectory for the ego vehicle is created and a cost function is used to evaluate the best lane to be in!  

### Trajectory Generation
To generate my car's trajectory I start out by consuming any of the previous points returned by the simulator. For the remaining points I use the spline library as described in the code walk through to generate trajectory of my ego vehicle. 

To create the spline path I start out with adding 2 waypoints based on the previous path or based on the cars current direction. In frenet, I add another 3 evenly 30m spaced waypoints ahead of the starting 2 referene points based on the the future selected lane. Now we use these 5 waypoints, shift it to the car's reference point and create a spline. To generate a trajectory of 50 waypoints I get the remaining points using this spline  

### Cost function analysis for choosing a lane
The cost functions I used for making a lane changing decision were:
- Collision cost: Binary cost function which penalizes collisions.
- Buffer cost: Penalizes getting close to other vehicles
- Inefficieny cost: Penalizes trajectories with intended lane that has traffic slower than vehicle's target speed.

Each of these functions had different weights with collion cost being the highest weigt (1000), followed by buffer cost (100), followed by inefficieny cost (10). This allowed me to change lanes without colliding with cars, maintaing a safe distance between cars, and picking a lane that has vehicles with higher speed or no vehicles at all.

### Potential improvements
Using predicted trajectories of non-ego vehicles in cost functions
Adding more cost functions to check if the car is in lane, out of road etc
Adding more cost functions to ensure max acceleration, jerk, total acceleration and jerk are not exceeded. 
