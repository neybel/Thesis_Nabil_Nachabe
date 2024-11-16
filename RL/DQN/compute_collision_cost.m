function collision_cost = compute_collision_cost(state, state2)
    % Example collision cost function
    % You need to define the logic for collision cost based on the distance between cars
    dist = norm(state(1:2) - state2(1:2));
    collision_cost = max(0, 1 / dist);  % High cost for small distances
end
