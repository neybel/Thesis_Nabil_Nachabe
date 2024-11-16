function CBF_value = evaluate_CBF(x_val, v_val)
    import casadi.*
   global alpha
   global dsafe

    % Define symbolic variables
    x = MX.sym('x', 3);  % x(1), x(2), x(3)
    v = MX.sym('v');

    % Define symbolic expression for CBF
    obstacle_center = 50;  % Define obstacle center
    CBF_expr = 2 * (((x(1) - obstacle_center) * (v * cos(x(3)))) + (x(2) * (v * sin(x(3))))) + ( alpha * ((x(1) - obstacle_center)^2 + x(2)^2 - dsafe));

    % Create CasADi function to evaluate CBF
    CBF_fun = Function('CBF', {x, v}, {CBF_expr});

    % Evaluate CBF at the specified values
    CBF_value = CBF_fun(x_val, v_val);
end