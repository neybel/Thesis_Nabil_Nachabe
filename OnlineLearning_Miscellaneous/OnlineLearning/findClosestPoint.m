% function idx = findClosestPoint(current_state, trajectory)
%     % current_state: [x; y]
%     % trajectory: [2 x N] matrix of waypoints
% 
%     distances = sqrt((trajectory(1, :) - current_state(1)).^2 + (trajectory(2, :) - current_state(2)).^2);
%     [~, idx] = min(distances);
% end
function idx = findClosestPoint(current_state, trajectory)
    % current_state: [x; y]
    % trajectory: [2 x N] matrix of waypoints

    distances = sqrt((trajectory(1, :) - current_state(1)).^2 + (trajectory(2, :) - current_state(2)).^2);
    [~, idx] = min(distances);

    % Ensure we select the closest forward point, not a past point
    if idx > 1 && current_state(1) < trajectory(1, idx-1)
        idx = idx - 1;
    end
end