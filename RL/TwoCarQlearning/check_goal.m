function has_reached_goal = check_goal(state, ref_state, goal_threshold)
    % Calculate the distance to the goal
    distance_to_goal = sqrt((state(1) - ref_state(1))^2 + (state(2) - ref_state(2))^2);
    
    % If the car is within the goal threshold, return true
    if distance_to_goal < goal_threshold
        has_reached_goal = true;
    else
        has_reached_goal = false;
    end
end
