function plotMassIdentification(results, m_true, m_identified)
    % Plot results from dynamic mass identification
    
    figure('Name', 'Part D: Mass Identification Results');
    
    % Differential torques
    subplot(2, 2, 1);
    plot(results.t, results.tau_diff, 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Differential Torque (Nm)');
    title('Torque Difference (With Ball - Without Ball)');
    legend('\tau_1', '\tau_2', '\tau_3');
    grid on;
    
    % Joint angles during excitation
    subplot(2, 2, 2);
    plot(results.t, results.theta, 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Joint Angle (rad)');
    title('Excitation Trajectory');
    legend('\theta_1', '\theta_2', '\theta_3');
    grid on;
    
    % Mass estimates over time
    subplot(2, 2, 3);
    plot(results.masses, 'o-', 'LineWidth', 1.5, 'MarkerSize', 4);
    hold on;
    yline(m_true, 'g--', 'LineWidth', 2, 'DisplayName', 'True Mass');
    yline(m_identified, 'r--', 'LineWidth', 2, 'DisplayName', 'Identified Mass');
    xlabel('Sample Index');
    ylabel('Mass (kg)');
    title('Mass Estimates from Dynamic Data');
    legend('Location', 'best');
    grid on;
    ylim([0, max(m_true*1.5, 1)]);
    
    % Histogram of estimates
    subplot(2, 2, 4);
    histogram(results.masses(results.masses > 0 & results.masses < 5), 20);
    hold on;
    xline(m_true, 'g--', 'LineWidth', 2);
    xline(m_identified, 'r--', 'LineWidth', 2);
    xlabel('Mass (kg)');
    ylabel('Count');
    title('Distribution of Mass Estimates');
    legend('Estimates', 'True', 'Identified');
    grid on;
end