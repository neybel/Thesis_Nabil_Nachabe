%% Helper function: Check if state is terminal
function done = is_terminal_state(state)
    % Define terminal conditions (e.g., collision or successful navigation)
    if state == 3 || state == 6 || state == 9 % Example terminal states
        done = true; % These could represent collisions or goals
    else
        done = false;
    end
end