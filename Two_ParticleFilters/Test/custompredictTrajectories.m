function [predicted_trajectories,predicted_controls] = custompredictTrajectories(particles, x0, N, W_xu1,ocp,car,testtheta1,testtheta2,pf,D1,D2,K)
num_particles = size(particles.W_xu, 1);
    predicted_trajectories = cell(num_particles, 1);
    predicted_controls = cell(num_particles, 1);
    
%     parfor i = 1:num_particles
    for i = 1:num_particles
        W_xu = particles.W_xu(i, :);  % Current particle's W_x
        W_xu=W_xu';
        if pf==1
        % Solve OCP for the current particle's W_x and fixed W_u
        [predicted_trajectories{i},predicted_controls{i},~,~] = OCP_solve(ocp, x0, W_xu, W_xu1,car,testtheta1,testtheta2,D1,D2);
%         predicted_trajectories{i} = predicted_trajectories{i}(6:10,:);
        else
        [predicted_trajectories{i},predicted_controls{i},~,~] = OCP_solve(ocp, x0, W_xu1, W_xu,car,testtheta1,testtheta2,D1,D2);
        end
        if pf==1
            predicted_trajectories{i}= predicted_trajectories{i}(6:10,1:K+1);
            predicted_controls{i}= predicted_controls{i}(3:4,1:K);
        else
            predicted_trajectories{i}= predicted_trajectories{i}(1:5,1:K+1);
            predicted_controls{i}= predicted_controls{i}(1:2,1:K);
        
        end
    end

%     plotRoundaboutWithLanes()
%     hold on
%     for u=1:num_particles
%     plot(predicted_trajectories{u}(1,:),predicted_trajectories{u}(2,:));
%     hold on
    end

