function [dx, u, mode] = acrobotClosedLoop(~, x, params, control, K, x_eq, E_desired)

    % State error (angles wrapped)
    x_err = [wrapToPi(x(1)-x_eq(1));
             wrapToPi(x(2)-x_eq(2));
             x(3)-x_eq(3);
             x(4)-x_eq(4)];

    angle_error = abs(x_err(1));
    velocity_norm = abs(x(3)) + abs(x(4));

    % SWITCHING
    if angle_error < control.theta_threshold && velocity_norm < control.omega_threshold
        u = -K * x_err;
        mode = 2;   % LQR
    else
        u = pflSwingUpController(x, params, control, E_desired);
        mode = 1;   % Swing-up
    end

    % Saturate
    u = max(min(u, control.u_max), -control.u_max);

    % Dynamics
    dx = acrobotDynamics(x, u, params);  % This version is a cached version of the Lagrangian EOMs and runs faster
    %dx = LagrangainEOMS(x, u, params);  % This version uses Lagrangian EOMs but it is very very slow as it evaluates the Jacobians at each time step

end
