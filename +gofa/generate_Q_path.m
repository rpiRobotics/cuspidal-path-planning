function Q_path = generate_Q_path(kin, R_path, T_path)

N = width(T_path);

Q_path = NaN([6, 16, N]);

for i = 1:N
    Q_i = IK.IK_2_parallel_2_intersecting_mex(R_path(:,:,i), T_path(:,i), kin);
    Q_path(:, 1:width(Q_i), i) = Q_i;
    % disp(i);
end
end