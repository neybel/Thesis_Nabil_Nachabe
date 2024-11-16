% Define simulateTrajectoryACADOS Function 
%% Aim: OCP do for each particle filter 
function predicted_trajectory = OCP_solve(ocp,initial_state,W_x, W_u)
    nx = length(initial_state);
    nu = 4;
    h = 0.05; % Time step
    ocp_N=20;
    T=h*ocp_N;
    % Initialize the predicted trajectory
%     predicted_trajectory = zeros(nx, N);
%     predicted_trajectory(:, 1) = initial_state;
    %% OCP everything needs to be defined previously in terms of the acados solver and sim model ... but here we dont care about the sim
D=WAY1_new();
COST=[];
STATUS=[];
	ocp.set('constr_x0', initial_state);
    current_state = initial_state; % Assuming the current state is [x; y; ...]
    closest_idx = findClosestPoint(current_state(1:2), D);
    if closest_idx + 20 <= size(D, 2)
        reference_points = D(:, closest_idx:closest_idx+20);
    else
        remaining_points = size(D, 2) - closest_idx + 1;
        reference_points = D(:, closest_idx:end);
        
        % Calculate the number of additional points needed
        additional_points = 21 - remaining_points;
        % Extend the trajectory in the same direction
        last_x = D(1, end);
        last_y = D(2, end);
        
        new_x = last_x - (1:additional_points) * 0.1; % Assuming 0.1m per step
        new_y = last_y * ones(1, additional_points);
        
        extended_points = [new_x; new_y];
        
        % Combine the existing and extended points
        reference_points = [reference_points, extended_points];
    end
             step_size2=T*initial_state(9);
             step_size4=0;
             x_next2 = linspace(initial_state(6), initial_state(6) - step_size2, ocp_N + 1);
             y_next2 = linspace(0,0 + step_size4 , ocp_N + 1);
             x_traj_init = [reference_points;0*ones(1, ocp_N + 1); initial_state(4) * ones(1, ocp_N + 1); zeros(1, ocp_N + 1);x_next2; y_next2; (pi)*ones(1, ocp_N + 1); initial_state(9) * ones(1, ocp_N + 1); zeros(1, ocp_N + 1)];
             p_ref=x_traj_init;
    for jj = 0 : ocp_N - 1
        ocp.set('p', [p_ref(:,jj+1);W_x;W_u], jj);
    end
    ocp.set('p', [p_ref(:,end);W_x;W_u], ocp_N);
% 	ocp.set('init_x', x_traj_init);
% 	ocp.set('init_u', u_traj_init);
	ocp.solve();
	if 1
        disp('')
		status = ocp.get('status');sqp_iter = ocp.get('sqp_iter');time_tot = ocp.get('time_tot');time_lin = ocp.get('time_lin');time_qp_sol = ocp.get('time_qp_sol');
        cost = ocp.get_cost();COST = [COST cost];STATUS=[STATUS status]; fprintf('\nstatus = %d, sqp_iter = %d, time_int = %f [ms] (time_lin = %f [ms], time_qp_sol = %f [ms])\n',...
            status, sqp_iter, time_tot*1e3, time_lin*1e3, time_qp_sol*1e3);
        if status~=0
            disp('acados ocp solver failed');
        end
	end
x_traj = ocp.get('x');
u_traj = ocp.get('u');
if any(STATUS == 4)
    disp('there have been a status 4!!')
    indices = find(STATUS == 4)
end
predicted_trajectory = x_traj;
end

