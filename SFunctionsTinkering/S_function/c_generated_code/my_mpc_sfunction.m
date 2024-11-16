function my_mpc_sfunction(block)
    setup(block);
end

function setup(block)
    % Register the number of input and output ports
    block.NumInputPorts  = 2;  % Two inputs: state and initial guess
    block.NumOutputPorts = 2;  % Two outputs: new state and control inputs

    % Set the dimensions of the input and output ports
    block.InputPort(1).Dimensions = 10;  % Adjust as needed (e.g., state dimension)
    block.InputPort(2).Dimensions = 7;   % Adjust as needed (e.g., initial guess dimension)
    
    block.OutputPort(1).Dimensions = 10; % New state output
    block.OutputPort(2).Dimensions = 4;  % Control inputs output

    % Set sample time
    block.SampleTimes = [0.1 0];  % Set the sample time (change as needed)

    % Register the methods
    block.RegBlockMethod('Outputs', @Output);
end

function Output(block)
    % Get inputs
    x0 = block.InputPort(1).Data;    % Current state
    initial_guess = block.InputPort(2).Data;  % Initial guess for beliefs
    
    % Parameters (you may want to move these to block parameters)
    W_xu11 = [10; 10; 1e-4; 1e-4; 1e-4; 0.50; 0.50]; % Weights for car 1
    num_iterations = 250;

    % Initialize arrays for storing outputs
    x_mpc = zeros(length(x0), num_iterations);
    u_mpc = zeros(4, num_iterations);  % Adjust based on your control input size

    % Main MPC loop
    for k = 1:num_iterations
        % Call your existing MPC solving logic here
        [X_star1, U_star1, u01] = OCP_solve(x0, initial_guess, W_xu11);  % Adjust this function call

        % Update state and control inputs
        x0 = updateState(X_star1);  % Implement this function to update the state based on your logic
        u_mpc(:, k) = u01;  % Store control inputs

        % Store results for output
        x_mpc(:, k) = x0;  
    end

    % Set outputs
    block.OutputPort(1).Data = x_mpc(:, end);  % Last state
    block.OutputPort(2).Data = u_mpc(:, end);  % Last control inputs
end
