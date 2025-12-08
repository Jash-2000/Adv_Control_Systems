function plotClosedLoopComparison(t_ref, x_ref, t_ol, x_ol, t_cl, x_cl)
    figure('Name', 'Closed-Loop LQR Results', 'Position', [200 200 1000 600]);
    
    subplot(2,1,1);
    plot(t_ref, x_ref(1,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ol, x_ol(:,1), 'r--', 'LineWidth', 1.5);
    plot(t_cl, x_cl(:,1), 'g-.', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position Comparison');
    legend('Reference', 'Open-Loop (perturbed)', 'Closed-Loop LQR', 'Location', 'best');
    grid on;
    
    subplot(2,1,2);
    plot(t_ref, x_ref(2,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_ol, x_ol(:,2), 'r--', 'LineWidth', 1.5);
    plot(t_cl, x_cl(:,2), 'g-.', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Comparison');
    legend('Reference', 'Open-Loop (perturbed)', 'Closed-Loop LQR', 'Location', 'best');
    grid on;
end
