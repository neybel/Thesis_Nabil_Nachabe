function model_obs = moving_obstacle_dynamics()
    import casadi.*

    %% system dimensions
    nx = 2; % Only obstacle position (x_obs, y_obs)
    nu = 0; % No control inputs for the obstacle

    %% named symbolic variables
    x_obs = SX.sym('x_obs');  % obstacle x position [m]
    y_obs = SX.sym('y_obs');  % obstacle y position [m]

    %% (unnamed) symbolic variables
    sym_x = vertcat(x_obs, y_obs);
    sym_xdot = SX.sym('xdot', nx, 1);

    %% Dynamics
    v_obs = 1; % constant velocity of the obstacle (you can adjust this as needed)

    dx_obs = v_obs;
    dy_obs = 0;

    expr_f_expl = vertcat(dx_obs, dy_obs);
    expr_f_impl = expr_f_expl - sym_xdot;

    %% Constraints
    % No input constraints since there are no inputs

    %% Cost
    % No cost terms since we are only defining the dynamics here

    % Populate structure
    model.nx = nx;
    model.nu = nu;
    model.sym_x = sym_x;
    model.sym_xdot = sym_xdot;
    model.expr_f_expl = expr_f_expl;
    model.expr_f_impl = expr_f_impl;
    % No input constraints
    model.lbu = [];
    model.ubu = [];

end
