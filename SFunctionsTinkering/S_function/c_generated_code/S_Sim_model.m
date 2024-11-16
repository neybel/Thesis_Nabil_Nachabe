function S_Sim_model(block)
    setup(block);
end

function setup(block)
    % Register number of input and output ports
    block.NumInputPorts  = 2; % Control input (u0) and initial state (x0)
    block.NumOutputPorts = 1; % Simulated next state

    % Setup input/output port sizes
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Define the dimensions of the input and output ports
    block.InputPort(1).Dimensions = [nu, 1]; % Control input (nu is control dimension)
    block.InputPort(2).Dimensions = [nx, 1]; % Initial state

    block.OutputPort(1).Dimensions = [nx, 1]; % Next state (simulated)

    % Register methods
    block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
    % Get inputs from Simulink
    u0 = block.InputPort(1).Data;  % Control input
    x0 = block.InputPort(2).Data;  % Initial state

    % Retrieve the Sim object from the base workspace
    sim = evalin('base', 'sim');

    % Call Sim_model to simulate the next state
    next_state = Sim_model(sim, u0, x0);

    % Assign the output
    block.OutputPort(1).Data = next_state;
end
