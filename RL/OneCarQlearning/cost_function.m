% Cost function (quadratic)
function cost = cost_function(state, ref_state, u, Q_xy,Q_theta, Q_v, R_a, R_delta)
    % State tracking cost
    x_error = Q_xy*(state(1) - ref_state(1))^2;
    y_error = Q_xy*(state(2) - ref_state(2))^2;
    theta_error = Q_theta * (state(3) - ref_state(3))^2;
    v_error = Q_v * (state(4) - ref_state(4))^2;
    
    % Control effort cost
    a_cost = R_a * u(1)^2;
    delta_rate_cost = R_delta * u(2)^2;
    
    % Total cost
    cost = x_error + y_error + theta_error + v_error + a_cost + delta_rate_cost;
end

% %%
% function reward = cost_function(state, ref_state, u, Q_xy, Q_theta, Q_v, R_a, R_delta)
%     % Define the threshold distance for reaching the goal
%     goal_threshold = 1.0; % Adjust this value as needed
%     large_reward = 100;    % Reward amount for reaching the goal
%     
%     % Calculate the distance to the reference (target) goal
%     distance_to_goal = norm(state(1:2) - ref_state(1:2));
%     
%     % Reward if the agent is within the goal threshold
%     if distance_to_goal < goal_threshold
%         reward = large_reward; % Large positive reward for reaching goal
%     else
%         % Regular penalties for state errors
%         x_error = Q_xy * (state(1) - ref_state(1))^2;
%         y_error = Q_xy * (state(2) - ref_state(2))^2;
%         theta_error = Q_theta * (state(3) - ref_state(3))^2;
%         v_error = Q_v * (state(4) - ref_state(4))^2;
% 
%         % Control effort penalties
%         a_cost = R_a * u(1)^2;
%         delta_rate_cost = R_delta * u(2)^2;
% 
%         % Distance penalty: higher for farther distances from the goal
%         distance_penalty = 10 * distance_to_goal;
% 
%         % Total reward: penalize errors, encourage goal proximity
%         reward = - (x_error + y_error + theta_error + v_error + a_cost + delta_rate_cost + distance_penalty);
%     end
% end

