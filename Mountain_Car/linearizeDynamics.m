function [A, B] = linearizeDynamics(x, u)
    % Linearize dynamics around operating point
    % x is [2x1], u is scalar
    
    % Compute Jacobians numerically for accuracy
    epsilon = 1e-6;
    
    % A matrix (df/dx) - need to extract column vector from result
    dx_plus = mountainCarDynamics(0, x + [epsilon; 0], u);
    dx_minus = mountainCarDynamics(0, x - [epsilon; 0], u);
    dfdx1 = (dx_plus(:) - dx_minus(:)) / (2*epsilon);
    
    dx_plus = mountainCarDynamics(0, x + [0; epsilon], u);
    dx_minus = mountainCarDynamics(0, x - [0; epsilon], u);
    dfdx2 = (dx_plus(:) - dx_minus(:)) / (2*epsilon);
    
    A = [dfdx1, dfdx2];
    
    % B matrix (df/du)
    dx_plus = mountainCarDynamics(0, x, u + epsilon);
    dx_minus = mountainCarDynamics(0, x, u - epsilon);
    B = (dx_plus(:) - dx_minus(:)) / (2*epsilon);
end