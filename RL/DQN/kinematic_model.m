% Function to compute the next state
function next_state = kinematic_model(state, control_input, car_length, dt)
    x = state(1);
    y = state(2);
    theta = state(3);
    v = state(4);
    delta = state(5);
    
    a = control_input(1);
    delta_rate = control_input(2);
    
    next_x = x + v * cos(theta) * dt;
    next_y = y + v * sin(theta) * dt;
    next_theta = theta + (v / car_length) * tan(delta) * dt;
    next_v = v + a * dt;
    next_delta = delta + delta_rate * dt;
    
    next_state = [next_x; next_y; next_theta; next_v; next_delta];
end