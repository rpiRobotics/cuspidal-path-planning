function T_path = generate_moveL(T_A, T_B, N)
    lambda = linspace(0, 1, N);
    T_path = lambda .* T_B + (1-lambda) .* T_A;
end