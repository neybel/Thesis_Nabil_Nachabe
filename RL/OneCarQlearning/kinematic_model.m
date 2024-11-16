% Dynamics function for the kinematic bicycle model
function next_state = kinematic_model(state, u, L, dt)
    x = state(1);
    y = state(2);
    theta = state(3);
    v = state(4);
    delta = state(5);
    
    % Control inputs
    a = u(1); % Acceleration
    delta_rate = u(2); % Steering rate
    
    % Update equations
    x_next = x + v * cos(theta) * dt;
    y_next = y + v * sin(theta) * dt;
    theta_next = theta + (v / L) * tan(delta) * dt;
    v_next = v + a * dt;
    delta_next = delta + delta_rate * dt;
    
    next_state = [x_next; y_next; theta_next; v_next; delta_next];
end