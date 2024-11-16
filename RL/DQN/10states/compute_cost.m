% Function to compute cost
function cost = compute_cost(state, ref_state, control_input, collision_cost)
    pos_error = norm(state(1:2) - ref_state(1:2));
    velocity_error = (state(4) - ref_state(4))^2;
    control_cost = sum(control_input.^2);
    cost = pos_error^2 + 10 * velocity_error + 0.1 * control_cost + collision_cost;
end