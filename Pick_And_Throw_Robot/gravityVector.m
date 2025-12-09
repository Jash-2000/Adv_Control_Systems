function G = gravityVector(theta, params)
    % Compute gravity vector
    m1 = params.m(1); m2 = params.m(2); m3 = params.m(3);
    L1 = params.L(1); L2 = params.L(2); L3 = params.L(3);
    g = params.g;
    
    q1 = theta(1); q2 = theta(2); q3 = theta(3);
    
    G = zeros(3, 1);
    G(1) = g*(m1*L1/2*cos(q1) + m2*(L1*cos(q1) + L2*cos(q1+q2)/2) + ...
              m3*(L1*cos(q1) + L2*cos(q1+q2) + L3*cos(q1+q2+q3)/2));
    G(2) = g*(m2*L2*cos(q1+q2)/2 + m3*(L2*cos(q1+q2) + L3*cos(q1+q2+q3)/2));
    G(3) = g*m3*L3*cos(q1+q2+q3)/2;
end