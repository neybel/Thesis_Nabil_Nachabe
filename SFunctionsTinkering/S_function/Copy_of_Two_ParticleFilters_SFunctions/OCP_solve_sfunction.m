function OCP_solve_sfunction(block)
    % Setup basic S-function methods
    setup(block);

function setup(block)
    %% Register the number of ports
    block.NumInputPorts  = 9; % Adjust based on how many inputs you have (e.g., initial_state, W_xu, etc.)
    block.NumOutputPorts = 4; % Outputs: predicted_trajectory, predicted_control, u0, test_theta

    % Setup the input port properties (adjust sizes based on inputs)
    block.SetPreCompInpPortInfoToDynamic;
    block.InputPort(1).Dimensions        = [8, 1]; % initial_state (example)
    block.InputPort(2).Dimensions        = [7, 1]; % W_xu (example)
    block.InputPort(3).Dimensions        = [7, 1]; % W_xu1 (example)
    block.InputPort(4).Dimensions        = 1;      % car (example)
    block.InputPort(5).Dimensions        = [20, 1]; % testtheta1 (example)
    block.InputPort(6).Dimensions        = [20, 1]; % testtheta2 (example)
    block.InputPort(7).Dimensions        = [2, 20]; % D1 (example)
    block.InputPort(8).Dimensions        = [2, 20]; % D2 (example)

    % Setup the output port properties
    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions       = [8, 20]; % predicted_trajectory
    block.OutputPort(2).Dimensions       = [4, 20]; % predicted_control
    block.OutputPort(3).Dimensions       = [4, 1];  % u0
    block.OutputPort(4).Dimensions       = [2, 20]; % test_theta1, test_theta2

    % Set block sample time
    block.SampleTimes = [0.1 0]; % Continuous sample time for OCP

    % Set up the block methods
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);

function Outputs(block)
    % Get inputs from Simulink (Example input mapping, adjust as per your setup)
    initial_state = block.InputPort(1).Data;
    W_xu          = block.InputPort(2).Data;
    W_xu1         = block.InputPort(3).Data;
    car           = block.InputPort(4).Data;
    testtheta1    = block.InputPort(5).Data;
    testtheta2    = block.InputPort(6).Data;
    D1            = block.InputPort(7).Data;
    D2            = block.InputPort(8).Data;

    % Define ocp, this might need to be globally initialized or stored outside this function
    persistent ocp;
    if isempty(ocp)
        % Initialize OCP once (this needs to be handled with persistence, depending on acados setup)
        ocp = initializeOcpSolver(); % You will need to define how ocp is initialized based on acados
    end

    % Call the original OCP_solve function
    [predicted_trajectory, predicted_control, u0, testtheta1, testtheta2] = OCP_solve(ocp, initial_state, W_xu, W_xu1, car, testtheta1, testtheta2, D1, D2);

    % Set the outputs back to Simulink
    block.OutputPort(1).Data = predicted_trajectory;
    block.OutputPort(2).Data = predicted_control;
    block.OutputPort(3).Data = u0;
    block.OutputPort(4).Data = [testtheta1; testtheta2];

function Terminate(block)
    % Cleanup function (optional)
end
