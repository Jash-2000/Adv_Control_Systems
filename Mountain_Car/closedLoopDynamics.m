function dx = closedLoopDynamics(t, x, u_ff_func, K_func, x_ref_func)
    % Closed-loop dynamics with feedforward + feedback control

    u_ff = u_ff_func(t); % Feed-forward term from trajectory optimization
    K = K_func(t); % 2X1 feedback gain for each input.
    x_ref = x_ref_func(t); % 2X1 x_reference path using TO.

    % Feedback control law
    u = u_ff - K' * (x - x_ref); 

    % Apply dynamics - dynamics returns [2x1], extract as column
    dx_result = mountainCarDynamics(t, x, u);
    dx = dx_result(:);  % Ensure column vector
end