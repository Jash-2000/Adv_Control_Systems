function [c, ceq] = terminalConstraint(t0, x0, tF, xF, L)
    % Terminal constraint: enforce user-defined stopping condition at final time
    %
    % Inputs:
    %   t0, x0 - initial time and state
    %   tF, xF - final time and state
    %   L      - link lengths
    %
    % Outputs:
    %   c   - inequality constraints (c <= 0)
    %   ceq - equality constraints (ceq = 0)
    
    % Extract final state
    theta_final = xF(1:3);
    thetaDot_final = xF(4:6);
    
    % Compute final end-effector position and velocity
    [~, ~, ee_final] = forwardKinematics(theta_final, L);
    J_final = computeJacobian(theta_final, L);
    ee_vel_final = J_final * thetaDot_final;
    
    % Extract end-effector state
    ee_x = ee_final(1);
    ee_y = ee_final(2);
    ee_vx = ee_vel_final(1);
    ee_vy = ee_vel_final(2);
    
    % Evaluate user-defined stopping condition
    stop_condition = userDefinedStoppingCondition(ee_x, ee_y, ee_vx, ee_vy, tF);
    
    % Convert boolean to constraint
    % If stop_condition = true (1), then ceq = 1 - 1 = 0 (satisfied)
    % If stop_condition = false (0), then ceq = 1 - 0 = 1 (violated)
    ceq = 1 - double(stop_condition);
    
    % No inequality constraints
    c = [];
end
