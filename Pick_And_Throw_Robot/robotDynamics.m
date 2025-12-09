function dx = robotDynamics(~, x, u, params)
    % Robot dynamics for 3-link planar arm
    % State: x = [theta1; theta2; theta3; thetaDot1; thetaDot2; thetaDot3]
    % Control: u = [tau1; tau2; tau3]
    
    % Handle vectorized input (multiple time points)
    if size(x, 2) > 1
        dx = zeros(size(x));
        for i = 1:size(x, 2)
            dx(:, i) = robotDynamics(0, x(:, i), u(:, i), params);
        end
        return;
    end
    
    theta = x(1:3);
    thetaDot = x(4:6);
    
    % Mass matrix and other dynamics terms
    M = massMatrix(theta, params);
    C = coriolisMatrix(theta, thetaDot, params);
    G = gravityVector(theta, params);
    
    % Equations of motion: M*thetaDotDot + C*thetaDot + G = u
    thetaDotDot = M \ (u - C*thetaDot - G);
    
    dx = [thetaDot; thetaDotDot];
end
