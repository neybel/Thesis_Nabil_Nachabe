function cost = cost_function(state1, state2, ref_state1, ref_state2, u1, u2, Q_xy, Q_theta, Q_v, R_a, R_delta, collision_threshold, collision_penalty)
    % State tracking cost for car 1
    x_error_1 = Q_xy * (state1(1) - ref_state1(1))^2;
    y_error_1 = Q_xy * (state1(2) - ref_state1(2))^2;
    theta_error_1 = Q_theta * (state1(3) - ref_state1(3))^2;
    v_error_1 = Q_v * (state1(4) - ref_state1(4))^2;

    % Control effort cost for car 1
    a_cost_1 = R_a * u1(1)^2;
    delta_rate_cost_1 = R_delta * u1(2)^2;

    % State tracking cost for car 2
    x_error_2 = Q_xy * (state2(1) - ref_state2(1))^2;
    y_error_2 = Q_xy * (state2(2) - ref_state2(2))^2;
    theta_error_2 = Q_theta * (state2(3) - ref_state2(3))^2;
    v_error_2 = Q_v * (state2(4) - ref_state2(4))^2;

    % Control effort cost for car 2
    a_cost_2 = R_a * u2(1)^2;
    delta_rate_cost_2 = R_delta * u2(2)^2;

    % Collision cost
    distance_between_cars = sqrt((state1(1) - state2(1))^2 + (state1(2) - state2(2))^2);
    if distance_between_cars < collision_threshold
        collision_cost = collision_penalty * (collision_threshold - distance_between_cars);
    else
        collision_cost = 0;
    end

    % Total cost
    cost = x_error_1 + y_error_1 + theta_error_1 + v_error_1 + a_cost_1 + delta_rate_cost_1 + ...
           x_error_2 + y_error_2 + theta_error_2 + v_error_2 + a_cost_2 + delta_rate_cost_2 + ...
           collision_cost;
end
