function [K, A_all, B_all] = computeLQRTrajectory(t, x, u)
    % Compute time-varying LQR gains along trajectory
    N = length(t);
    K = zeros(1, 4, N);
    A_all = zeros(4, 4, N);
    B_all = zeros(4, 1, N);
    
    % LQR weights
    Q = diag([20, 100, 5, 5]);  % Penalize angle errors more than velocity
    R = 1;                          % Control cost
    
    fprintf('Computing LQR gains along trajectory...\n');
    
    for i = 1:N
        [A, B] = linearizeDynamics(x(:,i), u(i));
        A_all(:,:,i) = A;
        B_all(:,:,i) = B;
        
        % Solve continuous-time algebraic Riccati equation
        [K_temp, ~, ~] = lqr(A, B, Q, R);
        K(:,:,i) = K_temp;
    end
    
    fprintf('LQR computation complete.\n');
end