function next_state = kinematic_model(state, u, L, dt)
    % Extract state variables
    x = state(1);
    y = state(2);
    theta = state(3);
    v = state(4);
    delta = state(5);

    % Apply the kinematic model
    x_next = x + v * cos(theta) * dt;
    y_next = y + v * sin(theta) * dt;
    theta_next = theta + v / L * tan(delta) * dt;
    v_next = v + u(1) * dt;  % Acceleration input
    delta_next = delta + u(2) * dt;  % Steering rate input

    % Update the state vector
    next_state = [x_next; y_next; theta_next; v_next; delta_next];
end
