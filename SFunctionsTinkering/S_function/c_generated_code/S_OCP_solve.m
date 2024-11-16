function S_OCP_solve(block)
    setup(block);
end

function setup(block)
    % Register number of input and output ports
    block.NumInputPorts  = 8; % State, initial_guess, W_xu, car, testtheta1, testtheta2, D1, D2
    block.NumOutputPorts = 4; % X_star, U_star, u0, test_theta_full

    % Setup input/output port sizes
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Define the dimensions of the input and output ports
    block.InputPort(1).Dimensions = [nx, 1]; % Initial state (nx is the state dimension)
    block.InputPort(2).Dimensions = [5, ocp_N+1]; % Initial guess belief (example size, adjust)
    block.InputPort(3).Dimensions = [7, 1]; % W_xu (example size, adjust)
    block.InputPort(4).Dimensions = [1, 1]; % Car indicator (binary)
    block.InputPort(5).Dimensions = [1, ocp_N+1]; % testtheta1 (example size, adjust)
    block.InputPort(6).Dimensions = [1, ocp_N+1]; % testtheta2 (example size, adjust)
    block.InputPort(7).Dimensions = [2, D1_size]; % Reference trajectory D1 (adjust as needed)
    block.InputPort(8).Dimensions = [2, D2_size]; % Reference trajectory D2 (adjust as needed)

    block.OutputPort(1).Dimensions = [nx, ocp_N+1]; % X_star trajectory
    block.OutputPort(2).Dimensions = [nu, ocp_N]; % U_star control trajectory
    block.OutputPort(3).Dimensions = [nu, 1]; % u0 (first control action)
    block.OutputPort(4).Dimensions = [1, ocp_N+1]; % Test theta full for car 1
    block.OutputPort(5).Dimensions = [1, ocp_N+1]; % Test theta full for car 2

    % Register methods
    block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
    % Inputs from Simulink model
    initial_state = block.InputPort(1).Data;
    initial_guess_belief = block.InputPort(2).Data;
    W_xu = block.InputPort(3).Data;
    car = block.InputPort(4).Data;
    testtheta1 = block.InputPort(5).Data;
    testtheta2 = block.InputPort(6).Data;
    D1 = block.InputPort(7).Data;
    D2 = block.InputPort(8).Data;
    
    % Retrieve the OCP object from the base workspace
    ocp = evalin('base', 'ocp');
    
    % Call OCP_solve function
    [X_star, U_star, u0, test_theta1, test_theta2] = OCP_solve(ocp, initial_state, W_xu, initial_guess_belief, car, testtheta1, testtheta2, D1, D2);
    
    % Assign the outputs
    block.OutputPort(1).Data = X_star;   % Predicted trajectory
    block.OutputPort(2).Data = U_star;   % Predicted control
    block.OutputPort(3).Data = u0;       % First control input
    block.OutputPort(4).Data = test_theta1; % Test theta 1
    block.OutputPort(5).Data = test_theta2; % Test theta 2
end
