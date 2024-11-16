
%% comments
The values you chose for noise (std_state = [0.1; 0.1; 0.01; 0.05; 0.02] for state variables and std_control = [0.1; 0.01] for control variables) are likely based on the realistic characteristics of common sensors and actuators used in autonomous vehicles. Here's a reasoning that can be used in your thesis report:

State Noise (Position, Orientation, Velocity, Steering):

x, y position (0.1): A standard deviation of 0.1 meters reflects common GPS sensor inaccuracies. Modern GPS systems often have an accuracy between 0.1-1 meters depending on the conditions and the quality of the GPS receiver.
Theta (heading angle, 0.01): The small noise in the heading angle is justified by the use of high-precision gyroscopes, which typically exhibit low angular noise, on the order of 0.01 radians.
Velocity (0.05): Velocity estimation based on wheel encoders or IMUs may have a small noise factor due to ground friction, slip, and sensor noise, making 0.05 a reasonable choice.
Steering angle (0.02): This accounts for steering noise due to imperfections in the feedback system (from a steering angle sensor) or slight variations in control. It's small but realistic for most control systems.
Control Noise (Acceleration, Steering Rate):

Acceleration (0.1): A value of 0.1 represents a small but noticeable noise in the actuation of the vehicle's acceleration, likely reflecting variations due to engine or motor control fluctuations, particularly in a real-world environment.
Steering rate (0.01): This reflects the precision of steering actuation, suggesting that the steering rate control is quite accurate, but still subject to minimal noise, which is realistic for actuators in autonomous vehicles.
These noise values provide a balance between underestimating and overestimating sensor and actuator noise, allowing the particle filter to function effectively by simulating real-world sensor imperfections. This makes the simulation more robust and closer to reality, ensuring that the control algorithms are tested under realistic conditions.