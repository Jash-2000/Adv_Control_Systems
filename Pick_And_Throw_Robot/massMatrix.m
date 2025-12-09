function M = massMatrix(theta, params)
    % Compute mass matrix for 3-link arm
    m1 = params.m(1); m2 = params.m(2); m3 = params.m(3);
    L1 = params.L(1); L2 = params.L(2); L3 = params.L(3);
    I1 = params.I(1); I2 = params.I(2); I3 = params.I(3);
    
    q1 = theta(1); q2 = theta(2); q3 = theta(3);
    
    % Simplified mass matrix (using center of mass at L/2 for each link)
    M = zeros(3, 3);
    
    % Diagonal and symmetric terms
    M(1,1) = I1 + I2 + I3 + m1*L1^2/4 + m2*(L1^2 + L2^2/4 + L1*L2*cos(q2)) + ...
             m3*(L1^2 + L2^2 + L3^2/4 + 2*L1*L2*cos(q2) + 2*L1*L3*cos(q2+q3)/2 + L2*L3*cos(q3));
    
    M(1,2) = I2 + I3 + m2*(L2^2/4 + L1*L2*cos(q2)/2) + ...
             m3*(L2^2 + L3^2/4 + L1*L2*cos(q2) + L1*L3*cos(q2+q3)/2 + L2*L3*cos(q3)/2);
    
    M(1,3) = I3 + m3*(L3^2/4 + L1*L3*cos(q2+q3)/2 + L2*L3*cos(q3)/2);
    
    M(2,2) = I2 + I3 + m2*L2^2/4 + m3*(L2^2 + L3^2/4 + L2*L3*cos(q3));
    
    M(2,3) = I3 + m3*(L3^2/4 + L2*L3*cos(q3)/2);
    
    M(3,3) = I3 + m3*L3^2/4;
    
    % Symmetric terms
    M(2,1) = M(1,2);
    M(3,1) = M(1,3);
    M(3,2) = M(2,3);
end