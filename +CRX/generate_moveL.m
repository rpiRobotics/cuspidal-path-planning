function [R_path, T_path] = generate_moveL(R_A, R_B, T_A, T_B, N)

    lambda = linspace(0, 1, N);
    
    T_path = lambda .* T_B + (1-lambda) .* T_A;
    
    
    R_BA = R_B * R_A';

    % Careful to generate moveL first and then add offsets
    axang = rotm2axang(R_BA); 
    axis_BA = axang(1:3)';
    theta_BA = axang(4); 
    theta = lambda .* theta_BA;

    R_path = NaN(3, 3, N);
    for i = 1:N
        R_path(:,:,i) = rot(axis_BA, theta(i))*R_A;
    end
end