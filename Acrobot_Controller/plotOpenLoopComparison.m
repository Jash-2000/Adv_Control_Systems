function plotOpenLoopComparison(t_opt, x_opt, t_ode, x_ode, err1, err2, err_v1, err_v2)
    figure('Name', 'Part B: Open-Loop Simulation (Acrobot)', 'Position', [150 150 1200 800]);
    
    subplot(3,2,1);
    plot(t_opt, rad2deg(x_opt(1,:)), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ode, rad2deg(x_ode(:,1)), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Joint 1 Angle: TO vs ODE45');
    legend('Traj. Opt.', 'ODE45', 'Location', 'best');
    grid on;
    
    subplot(3,2,2);
    plot(t_ode, rad2deg(err1), 'k-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Error (deg)');
    title('Joint 1 Angle Error');
    grid on;
    
    subplot(3,2,3);
    plot(t_opt, rad2deg(x_opt(2,:)), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ode, rad2deg(x_ode(:,2)), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Joint 2 Angle: TO vs ODE45');
    legend('Traj. Opt.', 'ODE45', 'Location', 'best');
    grid on;
    
    subplot(3,2,4);
    plot(t_ode, rad2deg(err2), 'k-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Error (deg)');
    title('Joint 2 Angle Error');
    grid on;
    
    subplot(3,2,5);
    plot(t_ode, err_v1, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity Error (rad/s)');
    title('Joint 1 Velocity Error');
    grid on;
    
    subplot(3,2,6);
    plot(t_ode, err_v2, 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity Error (rad/s)');
    title('Joint 2 Velocity Error');
    grid on;
end