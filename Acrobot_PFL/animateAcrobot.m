function animateAcrobot(t, x, p, mode_history, x_eq)

l1 = p.l1; 
l2 = p.l2;

figure('Name','Acrobot Animation');
skip = max(1, floor(length(t)/400));

for k = 1:skip:length(t)

    theta1 = x(1,k);
    theta2 = x(2,k);

    % --- Correct geometry for Spong convention ---
    p0 = [0 0];
    p1 = [ l1*sin(theta1),  l1*cos(theta1) ];
    p2 = p1 + [ l2*sin(theta1+theta2),  l2*cos(theta1+theta2) ];

    clf;
    hold on; axis equal; grid on;
    xlim([-2 2]); ylim([-2 2]);

    % Plot links
    plot([p0(1) p1(1)], [p0(2) p1(2)], 'b-', 'LineWidth', 4);
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'r-', 'LineWidth', 4);

    % Plot joints
    plot(p0(1), p0(2), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    plot(p1(1), p1(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(p2(1), p2(2), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');

    if mode_history(k) == 1
        mode_str = "PFL SWING-UP";
    else
        mode_str = "LQR BALANCING";
    end

    title(sprintf("t = %.2f s — %s", t(k), mode_str));
    drawnow;
    pause(0.02);
end
end
