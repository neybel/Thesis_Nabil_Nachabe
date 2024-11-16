function [next_state, reward] = environment_step(state, action, car_length, dt, goal_state, state2, state_dim)
    % Compute the next state using the kinematic model
    next_state = kinematic_model(state, action, car_length, dt);
    
    % Ensure next_state has the correct size
    assert(length(next_state) == state_dim, 'Next state dimension mismatch');
    
    % Compute the reward
    collision_cost = compute_collision_cost(state, state2);  % Compute collision cost
    reward = -compute_cost(next_state, goal_state, action, collision_cost);
end
