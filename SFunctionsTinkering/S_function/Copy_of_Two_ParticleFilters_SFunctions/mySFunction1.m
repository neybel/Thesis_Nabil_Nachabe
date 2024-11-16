function mySFunction1(block)
    % Setup the S-function block
    setup(block);
end

function setup(block)
    % Register number of input and output ports
    block.NumInputPorts  = 1;  % Example: 1 input port
    block.NumOutputPorts = 2;   % Example: 2 output ports

    % Set input and output port properties
    block.InputPort(1).Dimensions = 10;  % Example: Input size
    block.OutputPort(1).Dimensions = 1;   % Example: Output 1 size
    block.OutputPort(2).Dimensions = 1;   % Example: Output 2 size

    % Set up block sample time
    block.SampleTimes = [1 0]; % Run every 1 second

    % Register methods
    block.RegBlockMethod('SetInputPortSamplingMode', @SetInputPortSamplingMode);
    block.RegBlockMethod('SetOutputPortSamplingMode', @SetOutputPortSamplingMode);
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);
    block.RegBlockMethod('Terminate', @Terminate);
end

function SetInputPortSamplingMode(block, port, samplingMode)
    % Set the input port sampling mode
    block.InputPort(port).SamplingMode = samplingMode;
end

function SetOutputPortSamplingMode(block, port, samplingMode)
    % Set the output port sampling mode to match the input
    block.OutputPort(port).SamplingMode = block.InputPort(1).SamplingMode;
end

function InitializeConditions(block)
    % Initialize the outputs or state variables
    block.OutputPort(1).Data = 0; % Example initialization for output 1
    block.OutputPort(2).Data = 0; % Example initialization for output 2
end

function Outputs(block)
    % Define the outputs of the S-function
    x0 = block.InputPort(1).Data;  % Get input data
    [ocp, sim] = customSettingUp(x0);  % Custom setup function
    block.OutputPort(1).Data = ocp;   % Set output 1
    block.OutputPort(2).Data = sim;   % Set output 2
end

function Update(block)
    % Logic to update states or outputs at each time step
    % Implement necessary updates if required
end

function Terminate(block)
    % Logic to execute at the end of the simulation (optional)
    disp('Simulation finished.');
end
