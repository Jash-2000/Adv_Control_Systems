function K = interpK(t, t_opt, K_lqr)
    % Interpolate LQR gain matrix
    if t <= t_opt(1)
        K = K_lqr(:, :, 1);
    elseif t >= t_opt(end)
        K = K_lqr(:, :, end);
    else
        idx = find(t_opt >= t, 1);
        if idx == 1
            K = K_lqr(:, :, 1);
        else
            alpha = (t - t_opt(idx-1)) / (t_opt(idx) - t_opt(idx-1));
            K = (1-alpha)*K_lqr(:, :, idx-1) + alpha*K_lqr(:, :, idx);
        end
    end
end