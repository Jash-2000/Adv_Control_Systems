function plotOpenLoopComparison(t_opt, x_opt, t_ode, x_ode, error_pos, error_vel)
    figure('Name', 'Open-Loop Simulation Comparison', 'Position', [150 150 1000 700]);
    
    subplot(3,2,1);
    plot(t_opt, x_opt(1,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ode, x_ode(:,1), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position: TO vs ODE45');
    legend('Traj. Opt.', 'ODE45', 'Location', 'best');
    grid on;
    
    subplot(3,2,2);
    plot(t_ode, error_pos, 'k-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Error (m)');
    title('Position Error');
    grid on;
    
    subplot(3,2,3);
    plot(t_opt, x_opt(2,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ode, x_ode(:,2), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity: TO vs ODE45');
    legend('Traj. Opt.', 'ODE45', 'Location', 'best');
    grid on;
    
    subplot(3,2,4);
    plot(t_ode, error_vel, 'k-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Error (m/s)');
    title('Velocity Error');
    grid on;
    
    subplot(3,2,[5 6]);
    plot(t_ode, sqrt(error_pos.^2 + error_vel.^2), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Total Error');
    title('Combined State Error Magnitude');
    grid on;
end
