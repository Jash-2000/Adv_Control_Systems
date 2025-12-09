function m_ball = identifyMassStatic(x_hold, params, params_with_ball)
    % Method 1: Static force differential
    % Measure torques needed to hold position with and without ball
    
    theta = x_hold(1:3);
    
    % Torque without ball (only gravity compensation)
    G_without = gravityVector(theta, params);
    
    % Torque with ball (gravity compensation + ball weight)
    G_with = gravityVectorWithBall(theta, params_with_ball);
    
    % Differential torque
    tau_diff = G_with - G_without;
    
    % Compute Jacobian at end-effector
    J = computeJacobian(theta, params.L);
    
    % Extract vertical component (ball weight creates vertical force)
    % Force at end-effector: F = J^-T * tau_diff
    F_ee = J' \ tau_diff;
    
    % Vertical force component (y-direction)
    F_vertical = abs(F_ee(2));
    
    % Identify mass: F = m*g
    m_ball = F_vertical / params.g;
end