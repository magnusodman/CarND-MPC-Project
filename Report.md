Model Predictice Control
========================


Model predictive control is about turning the control problem into an optimization problem. First we identify the states that are relevant for the problem (cte, speed, direction, ..)  and how they interact (equations from time t to t+1). This model is then complemented with constraints for the states and a cost function. The optimization is then about minimizing the cost for a number of steps N in the future separated by timestep T) 

The model
---------

I used the state, actuators and update equations that were presented in the lesson.

States:
 1. x - x location in vehicle coordinates = 0 
 2. y - y location in vehicle coordinates = 0
 3. psi - direction in vehicle coordinate system = 0
 4. v - speed
 5. cte - cross track error, distance to center of road
 6. epsi - direction error in relation to road direction at car location.

Actuators:
 The actuators are variables that are part of the optimization problem and part of the cost function.

 1. delta - steer_value
 2. acceleration - throttle

Update equations:

  1. x' = x + v * cos(psi) * dt
  2. y' = y + v * sin(psi) * dt
  3. psi' = psi + v*delta* dt/Lf
  4. v' = v + a*dt
  5. cte' = cte + v*sin(epsi)*dt
  6. epsi' = epsi + v*delta*dt/Lf 

Timestep length and elapsed duration
------------------------------------

In my implementation I use a 3rd order polynomial approximation derived from simulator waypoints of the road. The product of the N (number of steps) and dt(step duration) is a measure of how forward looking our system is. 

We want the system to be forward looking enough to benefit from the information in polynomial but at the same time we don't want to have to many steps as that would be detrimental to the speed of the optimization or the prediction to crude (large dt). The is all about striking a good balance.

After some experimentation I settled for N = 12 and dt = 0.2. 


Polynomial fitting and MPC preprocessing
----------------------------------------

To calcuate the cte a polynomial is fitted to the waypoints from the simulator. This polynomial is used to calculate the road center location at the car position. To simplify calculations I transformed the waypoint coordinates to vehicle coordinate system. A lot of vehicle state was reduced to zeros due to this transformation.


Model predictive control with latency
-------------------------------------

My solution behaved good even with the latency present. This is handled implicitly by the cost parameters. An example is that the cost punish wild steering behaviour that could result from latency in actuation. A possible improvement is to incorporate the latency in the equations (if the latency is known and consistent).



