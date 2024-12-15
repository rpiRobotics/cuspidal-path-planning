function Q_path = generate_Q_path(kin, R_path, T_path, psi_path)

N = width(T_path);

SEW = sew_conv([0;0;1]);

Q_path = NaN([7, 64, N]);

for i = 1:N
    Q_i = SEW_IK.IK_gen_7_dof_mex(R_path(:,:,i), T_path(:,i), SEW, psi_path(i), kin);
    Q_path(:, 1:width(Q_i), i) = Q_i;
    disp(i);
end
end