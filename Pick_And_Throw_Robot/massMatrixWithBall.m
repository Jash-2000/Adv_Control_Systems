function M = massMatrixWithBall(theta, params)
    % Mass matrix including ball at end-effector
    M = massMatrix(theta, params);
    
    if isfield(params, 'm_ball')
        m_ball = params.m_ball;
        L = params.L;
        
        % Add ball inertia (point mass at end-effector)
        J = computeJacobian(theta, L);
        M = M + m_ball * (J' * J);
    end
end