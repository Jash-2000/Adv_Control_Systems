function [p1, p2, p3] = forwardKinematics(theta, L)
    % Compute joint positions
    p1 = [L(1)*cos(theta(1)); L(1)*sin(theta(1))];
    p2 = p1 + [L(2)*cos(theta(1)+theta(2)); L(2)*sin(theta(1)+theta(2))];
    p3 = p2 + [L(3)*cos(theta(1)+theta(2)+theta(3)); L(3)*sin(theta(1)+theta(2)+theta(3))];
end