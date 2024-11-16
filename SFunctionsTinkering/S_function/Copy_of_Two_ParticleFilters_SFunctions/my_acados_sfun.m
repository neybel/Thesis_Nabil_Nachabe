function my_acados_sfun(block)
    % Level-2 S-Function for acados setup with input and output
    setup(block);
end

function setup(block)
    % Specify the number of input ports (1 for initial conditions)
    block.NumInputPorts  = 1;    % 1 input port for initial conditions
    block.InputPort(1).Dimensions  = 10; % Expecting a 10x1 vector
    block.InputPort(1).DatatypeID  = 0;  % double
    block.InputPort(1).Complexity  = 'Real';
    block.InputPort(1).SamplingMode  = 'Sample'; % Ensure known frame status

    % Specify the number of output ports (ocp and sim)
    block.NumOutputPorts = 2;      % Output ocp and sim
    block.OutputPort(1).Dimensions  = 1; % Adjust based on ocp structure
    block.OutputPort(2).Dimensions  = 1; % Adjust based on sim structure
    block.OutputPort(1).DatatypeID  = 0;  % double
    block.OutputPort(1).Complexity  = 'Real';
    block.OutputPort(1).SamplingMode  = 'Sample'; % Ensure known frame status
    block.OutputPort(2).DatatypeID  = 0;  % double
    block.OutputPort(2).Complexity  = 'Real';
    block.OutputPort(2).SamplingMode  = 'Sample'; % Ensure known frame status

    % Sample times for the block
    block.SampleTimes    = [0 0];  % Continuous sample time

    % Register methods
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
    block.RegBlockMethod('SetInputPortSamplingMode', @SetInputPortSamplingMode);
end

function Outputs(block)
    % Retrieve initial conditions from the input port
    initial_conditions = block.InputPort(1).Data; % Expecting a 10x1 vector
    
    % Call the custom setup function to obtain ocp and sim
    [ocp, sim] = customSettingUp(initial_conditions);

    % Output the ocp and sim objects
%     block.OutputPort(1).Data = ocp;   % Output the ocp object
%     block.OutputPort(2).Data = sim;   % Output the sim object
end

function Terminate(block)
    % Clean-up code, if necessary
end

function SetInputPortSamplingMode(block, port, samplingMode)
    % This function allows setting the sampling mode for input ports
    block.InputPort(port).SamplingMode = samplingMode;
end
