function plotTrajectoryOptimization(t, x, u, u_max)
    figure('Name', 'Trajectory Optimization Results', 'Position', [100 100 1000 600]);
    
    subplot(3,1,1);
    plot(t, x(1,:), 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Optimal Position Trajectory');
    grid on;
    
    subplot(3,1,2);
    plot(t, x(2,:), 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Optimal Velocity Trajectory');
    grid on;
    
    subplot(3,1,3);
    plot(t, u, 'g-', 'LineWidth', 2);
    hold on;
    plot(t, u_max*ones(size(t)), 'k--', 'LineWidth', 1);
    plot(t, -u_max*ones(size(t)), 'k--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Control Force (N)');
    title('Optimal Control Input');
    legend('Control', 'Limits', 'Location', 'best');
    grid on;
end
