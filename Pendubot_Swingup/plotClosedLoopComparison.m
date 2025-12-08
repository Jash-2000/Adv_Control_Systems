function plotClosedLoopComparison(t_ref, x_ref, t_ol, x_ol, t_cl, x_cl)
    figure('Name', 'Part C: Closed-Loop LQR', 'Position', [200 200 1200 800]);
    
    subplot(2,2,1);
    plot(t_ref, rad2deg(x_ref(1,:)), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ol, rad2deg(x_ol(:,1)), 'r--', 'LineWidth', 1.5);
    plot(t_cl, rad2deg(x_cl(:,1)), 'g-.', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Joint 1 Angle');
    legend('Reference', 'Open-Loop (pert.)', 'Closed-Loop LQR', 'Location', 'best');
    grid on;
    
    subplot(2,2,2);
    plot(t_ref, rad2deg(x_ref(2,:)), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ol, rad2deg(x_ol(:,2)), 'r--', 'LineWidth', 1.5);
    plot(t_cl, rad2deg(x_cl(:,2)), 'g-.', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Joint 2 Angle');
    legend('Reference', 'Open-Loop (pert.)', 'Closed-Loop LQR', 'Location', 'best');
    grid on;
    
    subplot(2,2,3);
    plot(t_ref, x_ref(3,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ol, x_ol(:,3), 'r--', 'LineWidth', 1.5);
    plot(t_cl, x_cl(:,3), 'g-.', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    title('Joint 1 Angular Velocity');
    legend('Reference', 'Open-Loop (pert.)', 'Closed-Loop LQR', 'Location', 'best');
    grid on;
    
    subplot(2,2,4);
    plot(t_ref, x_ref(4,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ol, x_ol(:,4), 'r--', 'LineWidth', 1.5);
    plot(t_cl, x_cl(:,4), 'g-.', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    title('Joint 2 Angular Velocity');
    legend('Reference', 'Open-Loop (pert.)', 'Closed-Loop LQR', 'Location', 'best');
    grid on;
end