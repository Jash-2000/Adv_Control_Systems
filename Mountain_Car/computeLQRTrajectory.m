function [K, A_all, B_all] = computeLQRTrajectory(t, x, u)
    % Compute time-varying LQR gains along trajectory
    N = length(t);
    K = zeros(1, 2, N);
    K_j = zeros(1,2,N);
    A_all = zeros(2, 2, N);
    B_all = zeros(2, 1, N);
    
    % LQR weights
    Q = diag([10, 2]);    % State cost
    R = 1;                % Control cost
    
    for i = 1:N
        [A, B] = linearizeDynamics(x(:,i), u(i));
        A_all(:,:,i) = A;
        B_all(:,:,i) = B;
        
        % Solve continuous-time algebraic Riccati equation
        [K_temp, ~,~ ] = lqr(A, B, Q, R);
        K_j(:,:,i) = K_temp;
        K(:,:,i) = real(K_j(:,:,i));
    end
end