%% Initial state
x0 = [0;-45;pi/2;0;0;-30;0;0;10;0];
% plotRoundaboutWithLanes();
D1=W2_newnew(x0);D2=W3_newnew(x0); %% referencing
%% Setting Up acados (OCP and Sim)
if 1
[ocp,sim]=customSettingUp(x0);
end
%% Compile S functions of acados (ocp and sim)
cd c_generated_code
if 0
make_sfun_sim; % integrator
make_sfun; % ocp solver

%% Copy Simulink example block into c_generated_code
source_folder = fullfile(pwd, '..');
target_folder = pwd;
copyfile( fullfile(source_folder, 'Simulink_test_SFunction.slx'), target_folder );
%% Open Simulink example block
open_system(fullfile(target_folder, 'Simulink_test_SFunction.slx'))
end
%% Run the Simulink model
try
    sim('Simulink_test_SFunction.slx');
    cd ..
catch
    cd ..
    error('Simulink advanced closed loop example failed')
end