function plotResults(t, x, u, titleStr)
    figure('Name', titleStr);
    
    % Joint angles
    subplot(3, 1, 1);
    plot(t, x(1:3, :), 'LineWidth', 2);
    ylabel('Joint Angles (rad)');
    legend('\theta_1', '\theta_2', '\theta_3');
    title(titleStr);
    grid on;
    
    % Joint velocities
    subplot(3, 1, 2);
    plot(t, x(4:6, :), 'LineWidth', 2);
    ylabel('Joint Velocities (rad/s)');
    legend('\omega_1', '\omega_2', '\omega_3');
    grid on;
    
    % Control torques
    subplot(3, 1, 3);
    plot(t, u, 'LineWidth', 2);
    ylabel('Torques (Nm)');
    xlabel('Time (s)');
    legend('\tau_1', '\tau_2', '\tau_3');
    grid on;
end