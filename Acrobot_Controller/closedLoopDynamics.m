
function dx = closedLoopDynamics(t, x, u_ff_func, K_func, x_ref_func, u_max)
    % Closed-loop dynamics with feedforward + feedback control
    u_ff = u_ff_func(t);
    K = K_func(t);        % K is 4x1 (since we use u = u_ff - K*(x-x_ref))
    x_ref = x_ref_func(t);
    
    % Feedback control law: note K is column vector 4x1 -> u_c = K'*(x - x_ref)
    u = u_ff - (K' * (x - x_ref));
    
    % Apply control saturation
    u = max(min(u, u_max), -u_max);
    
    % Apply dynamics
    dx_result = acrobotDynamics(t, x, u);
    dx = dx_result(:);
end