## MPC model
Kinematic model is adopted in this project where the following six states and two actuactors are used:
1) State

Vehicle's position (x, y) coordinates, orientation angle (psi), velocity (v), cross track error (cte) and psi error (epsi).

2) Actuators

Steering angle (delta) and acceleration (a)

3) Update equations 

x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt

y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt

psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt

v_[t+1] = v[t] + a[t] * dt

cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

## Parameter tuning 
Timestep length and dt (elapsed duration between timesteps) are selected as 15 and 0.1s, respectively. 
In addition, there are different components in the cost function, their weights are first selected intuitively (such as large weight for 
cross track error and psi error), and then fine tuned based on the vehicle's behavior. 

Firstly, 1.5s prediction into the future seems to be sufficient in this case. 
1) Timestep length is choosen based on the plot of MPC trajectory path and refence path, 15 seems to be a proper number to align
these two plots (prior values tried 20, 10). 

2) dt is choosen to be 0.1s to account for the latency of the controller, so the previous control input can be used for the 
current prediction. (lines 118-123 in MPC.cpp)

## Preprocessing waypoints
Waypoints are first transformed into vehicle's coordinate system. A polynomial is fitted to transfromed waypoints. This simplifies
the process of calculating cross track and error and psi error, as vehicle will be at origin (0,0) with an orientation of 0. 

Handle control latency
As mentioned earlier, dt is choosen to be 0.1s to account for the latency of the controller, so the previous control input can be used for the 
current prediction. (lines 118-123 in MPC.cpp)
