function animatePendubot(t, x, titleStr)
    % Physical parameters for animation
    L1 = 1.0;
    L2 = 1.0;
    
    figure('Name', titleStr, 'Position', [250 250 700 700]);
    
    for i = 1:3:length(t)
        clf;
        
        % Extract angles
        theta1 = x(1,i);
        theta2 = x(2,i);
        
        % Compute link positions
        x1 = L1 * cos(theta1);
        y1 = L1 * sin(theta1);
        x2 = x1 + L2 * cos(theta1 + theta2);
        y2 = y1 + L2 * sin(theta1 + theta2);
        
        % Plot base
        plot(0, 0, 'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'k');
        hold on;
        
        % Plot link 1
        plot([0 x1], [0 y1], 'b-', 'LineWidth', 6);
        
        % Plot joint 1
        plot(x1, y1, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
        
        % Plot link 2
        plot([x1 x2], [y1 y2], 'r-', 'LineWidth', 6);
        
        % Plot end effector
        plot(x2, y2, 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
        
        % Plot trajectory trace
        if i > 1
            for j = 1:i-1
                th1_j = x(1,j);
                th2_j = x(2,j);
                x2_j = L1*cos(th1_j) + L2*cos(th1_j + th2_j);
                y2_j = L1*sin(th1_j) + L2*sin(th1_j + th2_j);
                plot(x2_j, y2_j, 'g.', 'MarkerSize', 2);
            end
        end
        
        % Formatting
        axis equal;
        xlim([-2.5 2.5]);
        ylim([-2.5 2.5]);
        xlabel('X Position (m)');
        ylabel('Y Position (m)');
        title(sprintf('%s\nTime: %.2f s', titleStr, t(i)));
        grid on;
        
        % Add reference frame
        plot([-2.5 2.5], [0 0], 'k--', 'LineWidth', 0.5);
        plot([0 0], [-2.5 2.5], 'k--', 'LineWidth', 0.5);
        
        drawnow;
        pause(0.02);
    end
end