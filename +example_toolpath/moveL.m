function [R_path, T_path, psi_path] = moveL(R_A, R_B, T_A, T_B, psi_A, psi_B, N_or_lambda)

    if isscalar(N_or_lambda)
        N = N_or_lambda;
        lambda = linspace(0, 1, N);
    else
        lambda = N_or_lambda;
        N = numel(lambda);
    end
    
    T_path = lambda .* T_B + (1-lambda) .* T_A;
    
    if ~isempty(R_A)
        R_BA = R_B * R_A';
    
        axang = rotm2axang(R_BA);
        axis_BA = axang(1:3)';
        theta_BA = axang(4); 
        theta = lambda .* theta_BA;
    
        R_path = NaN(3, 3, N);
        for i = 1:N
            R_path(:,:,i) = rot(axis_BA, theta(i))*R_A;
        end
    else
        R_path = [];
    end
    
    if ~isempty(psi_A)
        psi_path = lambda .* psi_B + (1-lambda) .* psi_A;
    end
end