% Define simulateTrajectoryACADOS Function 
%% Aim: OCP do for each particle filter 
function [predicted_trajectory,predicted_control,u0,test_theta1,test_theta2] = OCP_solve(ocp,initial_state,W_xu, W_xu1,car,testtheta1,testtheta2,D1,D2)
    nx = length(initial_state);
    nu = 4;
    h = 0.1; % Time step
    ocp_N=20;
    T=h*ocp_N;
    % Initialize the predicted trajectory
%     predicted_trajectory = zeros(nx, N);
%     predicted_trajectory(:, 1) = initial_state;
% test_theta1=(pi/2)*ones(1,ocp_N+1);
% test_theta2=(0)*ones(1, ocp_N + 1);
    %% OCP everything needs to be defined previously in terms of the acados solver and sim model ... but here we dont care about the sim
% D1=W2();D2=W3();
% D1=W2_new();D2=W3_new();
COST=[];
STATUS=[];
	 ocp.set('constr_x0', initial_state);
     current_state1 = initial_state(1:2); current_state2 = initial_state(6:7);
     closest_idx1 = findClosestPoint(current_state1, D1);
     closest_idx2 = findClosestPoint(current_state2, D2);
%% checking for car 1
       if closest_idx1 + ocp_N <= size(D1, 2)
        reference_points1 = D1(:, closest_idx1:closest_idx1+ocp_N);
       else
        remaining_points1 = size(D1, 2) - closest_idx1 + 1;
        reference_points1 = D1(:, closest_idx1:end);
        additional_points1 = (ocp_N+1) - remaining_points1;

        last_x1 = D1(1, end);
        last_y1 = D1(2, end);
        new_x1 = last_x1 - (1:additional_points1) * 0.1; % Assuming 0.1m per step
        new_y1 = last_y1 * ones(1, additional_points1);
        
        extended_points1 = [new_x1; new_y1];
        
        % Combine the existing and extended points
        reference_points1 = [reference_points1, extended_points1];
       end
       %% checking for car 2
              if closest_idx2 + ocp_N <= size(D2, 2)
        reference_points2 = D2(:, closest_idx2:closest_idx2+ocp_N);
       else
        remaining_points2 = size(D2, 2) - closest_idx2 + 1;
        reference_points2 = D2(:, closest_idx2:end);
        additional_points2 = (ocp_N+1) - remaining_points2;

        last_x2 = D2(1, end);
        last_y2 = D2(2, end);
        new_x2 = last_x2 + (1:additional_points2) * 0.1; % Assuming 0.1m per step
        new_y2 = last_y2 * ones(1, additional_points2);
        
        extended_points2 = [new_x2; new_y2];
        
        % Combine the existing and extended points
        reference_points2 = [reference_points2, extended_points2];
              end
           x_traj_init = [reference_points1; testtheta1; 0*ones(1, ocp_N + 1);zeros(1,ocp_N+1); reference_points2; testtheta2; 0 * ones(1, ocp_N + 1); zeros(1, ocp_N + 1)];
             p_ref=x_traj_init;

             %% Editing

    for jj = 0 : ocp_N - 1
        if car
        ocp.set('p', [p_ref(:,jj+1);W_xu1(1:5);W_xu(1:5);W_xu1(6:7);W_xu(6:7)], jj);
        else
        ocp.set('p', [p_ref(:,jj+1);W_xu(1:5);W_xu1(1:5);W_xu(6:7);W_xu1(6:7)], jj);
        end

    end
    if car
    ocp.set('p', [p_ref(:,end);W_xu1(1:5);W_xu(1:5);W_xu1(6:7);W_xu(6:7)], ocp_N);
    else 
    ocp.set('p', [p_ref(:,end);W_xu(1:5);W_xu1(1:5);W_xu(6:7);W_xu1(6:7)], ocp_N);
    end

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
u0 = ocp.get('u', 0);
if any(STATUS == 4)
    disp('there have been a status 4!!')
    indices = find(STATUS == 4)
end
test_theta1=x_traj(3,:);test_theta2=x_traj(8,:);
predicted_trajectory = x_traj;
predicted_control = u_traj;
end

