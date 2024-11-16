function my_acados_sfun1(block)
    % Level-2 S-Function template for acados OCP and Sim model setup
    
    setup(block);
end

function setup(block)
    % Set block properties
    block.NumDialogPrms  = 1;  % Number of dialog parameters (e.g., x0)
    block.NumInputPorts  = 0;   % No input ports
    block.NumOutputPorts = 0;   % No output ports
    block.SampleTimes    = [0 0]; % Continuous sample time
    
    % Register methods
    block.RegBlockMethod('InitConditions', @InitConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);
    block.RegBlockMethod('Terminate', @Terminate);
end

function InitConditions(block)
    % Initialize conditions (can be used to create ocp and sim objects)
    x0 = block.DialogPrm(1).Data; % Retrieve the initial state from the dialog parameter
    
    % Call customSettingUp to create ocp and sim
    [block.OCPSolver, block.Simulator] = customSettingUp(x0);
end

function Outputs(block)
    % Define output (if necessary)
    % Not needed in your case as outputs are not defined
end

function Update(block)
    % Update method (can be used for any internal logic)
end

function Terminate(block)
    % Terminate method (clean-up, if necessary)
end

function [ocp, sim] = customSettingUp(x0)
    % This is your existing customSettingUp function
    % Ensure that you include all the required code here
    % ...
    
    % Note: Make sure to change 'model_name' if necessary to avoid conflicts
    GENERATE_C_CODE = 0;
    model_name = 'BILOOOO';
    % Check if environment has been set up
    env_run = getenv('ENV_RUN');
    if (~strcmp(env_run, 'true'))
        error('env.sh has not been sourced! Before executing this example, run: source env.sh');
    end
    
    %% Options (same as your existing code)
    compile_interface = 'auto'; 
    codgen_model = 'true'; 
    sim_method = 'erk'; 
    sim_sens_forw = 'false'; 
    sim_num_stages = 4; 
    sim_num_steps = 4; 
    ocp_N = 20; 
    nlp_solver = 'sqp_rti'; 
    nlp_solver_exact_hessian = 'false'; 
    regularize_method = 'project_reduc_hess'; 
    qp_solver = 'partial_condensing_hpipm'; 
    qp_solver_iter_max = 100; 
    qp_solver_cond_N = 5; 
    qp_solver_warm_start = 0; 
    qp_solver_cond_ric_alg = 0; 
    qp_solver_ric_alg = 0; 
    ocp_sim_method = 'erk'; 
    ocp_sim_method_num_stages = 4; 
    ocp_sim_method_num_steps = 4; 
    cost_type = 'ext_cost'; 
    
    %% Create model entries (same as your existing code)
    model = Dynamics_Roundabout();
    h = 0.1;
    T = ocp_N * h; 
    nx = model.nx;
    nu = model.nu;
    ny = nu + nx; 
    ny_e = nx; 
    ng = 0; 
    ng_e = 0; 
    nbx = 0; 
    linear_constraints = 0; 
    nbu = nu; 
    nh = nu; 
    nh_e = 0; 
    
    % Constraints setup (same as your existing code)
    % ... Your existing code here for constraints
    
    %% CBF (same as your existing code)
    % ... Your existing code here for CBF
    
    %% acados ocp model setup
    ocp_model = acados_ocp_model();
    ocp_model.set('name', model_name);
    ocp_model.set('T', T);

    % Symbolics
    ocp_model.set('sym_x', model.sym_x);
    if isfield(model, 'sym_u')
        ocp_model.set('sym_u', model.sym_u);
    end
    if isfield(model, 'sym_xdot')
        ocp_model.set('sym_xdot', model.sym_xdot);
    end
    if isfield(model, 'sym_p')
        ocp_model.set('sym_p', model.sym_p);
    end

    % Cost
    ocp_model.set('cost_type', cost_type);
    ocp_model.set('cost_type_e', cost_type);
    ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
    ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
    
    % Dynamics
    if (strcmp(ocp_sim_method, 'erk'))
        ocp_model.set('dyn_type', 'explicit');
        ocp_model.set('dyn_expr_f', model.expr_f_expl);
    else % irk
        ocp_model.set('dyn_type', 'implicit');
        ocp_model.set('dyn_expr_f', model.expr_f_impl);
    end
    
    % Constraints
    ocp_model.set('constr_x0', x0);
    % Add constraints as needed...
    
    % acados ocp opts setup (same as your existing code)
    ocp_opts = acados_ocp_opts();
    ocp_opts.set('compile_interface', compile_interface);
    ocp_opts.set('codgen_model', codgen_model);
    ocp_opts.set('param_scheme_N', ocp_N);
    ocp_opts.set('nlp_solver', nlp_solver);
    ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
    ocp_opts.set('regularize_method', regularize_method);
    if (strcmp(nlp_solver, 'sqp'))
        ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
    end
    ocp_opts.set('qp_solver', qp_solver);
    if (strcmp(qp_solver, 'partial_condensing_hpipm'))
        ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
        ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);
        ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
        ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
    end
    ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);
    ocp_opts.set('sim_method', ocp_sim_method);
    ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);
    ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);
    
    %% Create acados ocp
    ocp = acados_ocp(ocp_model, ocp_opts);

    %% acados sim model setup
    sim_model = acados_sim_model();
    sim_model.set('sym_x', model.sym_x);
    if isfield(model, 'sym_u')
        sim_model.set('sym_u', model.sym_u);
    end
    if isfield(model, 'sym_xdot')
        sim_model.set('sym_xdot', model.sym_xdot);
    end
    if isfield(model, 'sym_p')
        sim_model.set('sym_p', model.sym_p);
    end
    
    % Model
    sim_model.set('T', T / ocp_N);
    if (strcmp(sim_method, 'erk'))
        sim_model.set('dyn_type', 'explicit');
        sim_model.set('dyn_expr_f', model.expr_f_expl);
    else % irk
        sim_model.set('dyn_type', 'implicit');
        sim_model.set('dyn_expr_f', model.expr_f_impl);
    end
    
    %% acados sim opts setup
    sim_opts = acados_sim_opts();
    sim_opts.set('compile_interface', compile_interface);
    sim_opts.set('codgen_model', codgen_model);
    sim_opts.set('num_stages', sim_num_stages);
    sim_opts.set('num_steps', sim_num_steps);
    sim_opts.set('method', sim_method);
    sim_opts.set('sens_forw', sim_sens_forw);
    
    % Create the sim object
    sim = acados_sim(sim_model, sim_opts);
end
