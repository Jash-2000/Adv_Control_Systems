function dx = closedLoopDynamics(t, x, u_ff_func, K_func, x_ref_func, u_max)
    % Closed-loop dynamics with feedforward + feedback control
    u_ff = u_ff_func(t);
    K = K_func(t);
    x_ref = x_ref_func(t);
    
    % Feedback control law
    u = u_ff - K * (x - x_ref);
    
    % Apply control saturation
    u = max(min(u, u_max), -u_max);
    
    % Apply dynamics
    dx_result = pendubotDynamics(t, x, u);
    dx = dx_result(:);
end