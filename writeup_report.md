# Implementation #


## The model ##

> Student describes their model in detail. This includes the state, actuators and update equations.
> 

The model used is a kinematic model composed of the following **state parameters**:

1. x: car position.
1. y: car position.
1. psi: car's orientation angle.
1. v: car's velocity.
1. cte: cross-track error.
1. epsi: orientation error.

This **model is updated** each timestep using the following equations:

    double x_est = 0 + v_prev * dt;
    double y_est = 0; 
    double psi_est = 0 - v_prev/Lf * delta * dt;
    double v_est = v_prev + a *dt;
    double cte_est = cte_prev + v_prev * sin(epsi) * dt;
    double epsi_est = epsi_prev -v/Lf * delta * dt;

However, these zeroes explicitly included in the updated equations are a reference of the coordinate system's used. If the map and the waypoint are expressed in the car's coordinate system, it means the car is located at x_ prev, y_ prev = 0,0 with a psi_ prev = 0. In other case, these zeroes need to be substituted with the corresponding values.

The **actuators** taken into account in this model are the following:

1. delta: steering angle. This value is constrained to the interval [-25º, 25º].
1. a: throttle. This value is constrained to [-1, 1]. 1 means full throttle and -1 means full brake.

One extra comment: these update equations are made considering that positive values for the actuator signal delta implies a right turn and negative values implies left turns.


## Timestep Length and Elapsed Duration (N & dt)

> Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

I tried with timestep length N = 15 and elapsed duration dt = 0.1 because those were values close to those used in the quizzes of these lessons. Also it seemed very reasonably: the time predicted ahead was 1.5 s, a bigger period could produce incorrect assumptions of the state and the car as they change very fast. After some laps running in the simulator, I thought the green bar (the predicted trajectory) was too small compared with the yellow bar (the polynomial fitted to the waypoints), hence I increased N. The result of this was a complete disaster. After some study, I realize that the optimizer had insufficient time to address this increase in complexity within the time slot allocated for it (max 0.5 s). Thus, the function didn't converge and I got wrong actuators' values, so I returned to the original values.


## Polynomial Fitting and MPC Preprocessing ##

> A polynomial is fitted to waypoints.
> 
> 
> If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

Waypoints came in the map's coordinate system but the visualization of the predicted trajectory (from MPC controller) and the polynomial-fitted trajectory (from the waypoints) required to be expressed in car's coordinate system, so it seemed a good idea to express everything in the same reference's system: the car. To do this, typical equations to rotate and shift a point in a 2d plane were used.

    for(unsigned int i=0; i<ptsx.size(); i++)
    {
    	// shift car reference angle to 90 degrees
    	shift_x = ptsx[i] - x;
    	shift_y = ptsy[i] - y;
    
    	ptsx[i] = shift_x * cos(0-psi) - shift_y * sin(0-psi);
    	ptsy[i] = shift_x * sin(0-psi) + shift_y * cos(0-psi);
    
    }

Also, it helped to compute cte and epsi values.

## Model Predictive Control with Latency ##
	

> The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Latency is a big issue. Despite being able to correctly calculate how to deal with the actuators, if you issue the command with a delay, it can produce undesired behaviors like dangerous turns or even going out off the road. To address this issue, the state of the car is estimated ahead of time before calling the MPC controller. The elapsed time estimated in the future is the same as the "estimated" (simulated) latency.