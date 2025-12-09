function [A, B] = linearizeDynamics(x, u, params)
    % Linearize dynamics at state x and control u
    epsilon = 1e-6;
    n = length(x);
    m = length(u);
    
    A = zeros(n, n);
    B = zeros(n, m);
    
    % Compute A matrix (df/dx)
    for i = 1:n
        x_plus = x;
        x_plus(i) = x_plus(i) + epsilon;
        x_minus = x;
        x_minus(i) = x_minus(i) - epsilon;
        
        A(:, i) = (robotDynamics(0, x_plus, u, params) - ...
                   robotDynamics(0, x_minus, u, params)) / (2*epsilon);
    end
    
    % Compute B matrix (df/du)
    for i = 1:m
        u_plus = u;
        u_plus(i) = u_plus(i) + epsilon;
        u_minus = u;
        u_minus(i) = u_minus(i) - epsilon;
        
        B(:, i) = (robotDynamics(0, x, u_plus, params) - ...
                   robotDynamics(0, x, u_minus, params)) / (2*epsilon);
    end
end