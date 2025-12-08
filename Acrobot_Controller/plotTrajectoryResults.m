function plotTrajectoryResults(t, x, u, u_max)
    figure('Name', 'Part A: Trajectory Optimization (Acrobot)', 'Position', [100 100 1200 700]);
    
    subplot(3,2,1);
    plot(t, rad2deg(x(1,:)), 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Joint 1 Angle (Link 1 - Passive)');
    grid on;
    
    subplot(3,2,2);
    plot(t, rad2deg(x(2,:)), 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Joint 2 Angle (Link 2 - Actuated)');
    grid on;
    
    subplot(3,2,3);
    plot(t, x(3,:), 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    title('Joint 1 Angular Velocity');
    grid on;
    
    subplot(3,2,4);
    plot(t, x(4,:), 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    title('Joint 2 Angular Velocity');
    grid on;
    
    subplot(3,2,[5 6]);
    plot(t, u, 'g-', 'LineWidth', 2);
    hold on;
    plot(t, u_max*ones(size(t)), 'k--', 'LineWidth', 1);
    plot(t, -u_max*ones(size(t)), 'k--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Torque (N-m)');
    title('Control Input (Joint 2 Torque)');
    legend('Control', 'Limits', 'Location', 'best');
    grid on;
end
