function G = gravityVectorWithBall(theta, params)
    % Gravity vector including ball mass at end-effector
    G = gravityVector(theta, params);
    
    if isfield(params, 'm_ball')
        m_ball = params.m_ball;
        L = params.L;
        g = params.g;
        
        q1 = theta(1); q2 = theta(2); q3 = theta(3);
        
        % Add contribution from ball at end-effector
        J = computeJacobian(theta, L);
        F_ball = [0; -m_ball * g];  % Vertical force
        tau_ball = J' * F_ball;
        
        G = G + tau_ball;
    end
end