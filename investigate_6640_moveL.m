kin = hardcoded_IK_setups.IRB_6640.get_kin();
kin.P(:,end)=0;

q_A = rand_angle([6 1]);
q_B = rand_angle([6 1]);

[R_A, T_A] = fwdkin(kin, q_A);
[R_B, T_B] = fwdkin(kin, q_B);

[R_path, T_path] = CRX.generate_moveL(R_A, R_B, T_A, T_B, 1000);

Q_path = generate_Q_path(kin, R_path, T_path);

plot(squeeze(Q_path(4,:,:))', 'k.')


function Q_path = generate_Q_path(kin, R_path, T_path)
N = width(T_path);

Q_path = NaN([6, 12, N]);

for i = 1:N
    [Q_LS, is_LS_vec] = IK.IK_spherical_2_parallel(R_path(:,:,i), T_path(:,i), kin);
    Q_LS(:,is_LS_vec==1) = NaN;
    Q_i = Q_LS;
    Q_path(:, 1:width(Q_i), i) = Q_i;
end
end