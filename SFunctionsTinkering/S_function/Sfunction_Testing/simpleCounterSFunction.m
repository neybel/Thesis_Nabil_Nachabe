function simple_counter_sfunction(block)
    % Setup the S-function block
    setup(block);
end

function setup(block)
    %% Register number of ports
    block.NumInputPorts  = 0;  % No input ports
    block.NumOutputPorts = 1;   % One output port

    % Set up block sample time
    block.SampleTimes = [1 0];   % Run every 1 second

    % Register methods
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropagationSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
end

function DoPostPropagationSetup(block)
    % This is where we register the Dwork vector
    block.NumDworks = 1; % We need 1 Dwork for the counter
    block.Dwork(1).Name = 'Counter'; % Name the Dwork
    block.Dwork(1).Dimensions = 1; % Scalar value
    block.Dwork(1).DatatypeID = 0; % 0 means double
    block.Dwork(1).Complexity = 'Real'; % Real number
    block.Dwork(1).UsedAsDiscState = true; % Discrete state
end

function Start(block)
    % Initialize the counter at the start of the simulation
    disp('Starting the counter...');
    block.Dwork(1).Data = 0; % Initialize counter to 0
end

function Outputs(block)
    % Increment the counter at each time step
    counter = block.Dwork(1).Data;
    counter = counter + 1;
    
    % Store the updated counter
    block.Dwork(1).Data = counter;

    % Output the current counter value
    block.OutputPort(1).Data = counter;  % Send the counter value to the output
end

function Terminate(block)
    % Termination function (optional)
    disp('Simulation finished.');
end
