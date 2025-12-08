function animateMountainCar(t, x, titleStr)
    % Create mountain profile
    x_terrain = linspace(-4, 4, 500);
    y_terrain = 0.5*x_terrain.^2 - 0.1*x_terrain.^4 + 0.2*x_terrain.^3;
    figure('Name', titleStr, 'Position', [250 250 800 500]);
    
    for i = 1:5:length(t)
        clf;
        
        % Plot terrain
        plot(x_terrain, y_terrain, 'k-', 'LineWidth', 2);
        hold on;
        
        % Plot car position
        car_pos = x(1,i);
        car_height = 0.5*car_pos^2 - 0.1*car_pos^4 + 0.2*car_pos^3;
        plot(car_pos, car_height, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
        
        % Plot trajectory history
        if i > 1
            car_hist_pos = x(1,1:i);
            car_hist_height = 0.5*car_hist_pos.^2 - 0.1*car_hist_pos.^4 + 0.2*car_hist_pos.^3;
            plot(car_hist_pos, car_hist_height, 'b.', 'MarkerSize', 3);
        end
        
        xlabel('Position (m)');
        ylabel('Height (m)');
        title(sprintf('%s\nTime: %.2f s', titleStr, t(i)));
        grid on;
        axis equal;
        xlim([-4 4]);
        ylim([-2 3]);
        
        drawnow;
        pause(0.05);
    end
end