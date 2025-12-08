function [A, B] = linearizeDynamics(x, u)
    % Linearize dynamics around operating point using numerical differentiation
    epsilon = 1e-6;
    n_states = length(x);
    
    % A matrix (df/dx)
    A = zeros(n_states, n_states);
    for i = 1:n_states
        x_plus = x;
        x_plus(i) = x_plus(i) + epsilon;
        x_minus = x;
        x_minus(i) = x_minus(i) - epsilon;
        
        dx_plus = pendubotDynamics(0, x_plus, u);
        dx_minus = pendubotDynamics(0, x_minus, u);
        
        A(:,i) = (dx_plus(:) - dx_minus(:)) / (2*epsilon);
    end
    
    % B matrix (df/du)
    dx_plus = pendubotDynamics(0, x, u + epsilon);
    dx_minus = pendubotDynamics(0, x, u - epsilon);
    B = (dx_plus(:) - dx_minus(:)) / (2*epsilon);
end