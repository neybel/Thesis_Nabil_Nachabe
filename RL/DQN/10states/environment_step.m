function [next_state1, next_state2, reward1, reward2] = environment_step(state1, action1, state2, action2, car_length, dt, goal1, goal2)
    % Extract actions for both cars
    acc1 = action1(1);
    steer1 = action1(2);

    acc2 = action2(1);
    steer2 = action2(2);

    % Implement vehicle dynamics for both cars
    next_state1 = state1; % Replace with actual dynamics update logic
    next_state2 = state2; % Replace with actual dynamics update logic

    % Compute rewards for both cars
    reward1 = -norm(next_state1(1:2) - goal1(1:2)); % Example reward function (negative distance to goal)
    reward2 = -norm(next_state2(1:2) - goal2(1:2)); % Example reward function (negative distance to goal)
end
