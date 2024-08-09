function Q_path = CRX_generate_Q_path(kin, R_path, T_path)

LS_thresh = 1e-2;

N = width(T_path);

Q_path = NaN([6, 12, N]);

for i = 1:N
    [Q_LS, is_LS_vec] = IK_3_pairs_intersecting_LS_mex(R_path(:,:,i), T_path(:,i), kin, false);
    % Q_i = Q_LS(:, all( abs(is_LS_vec(1:2, :))<LS_thresh ));
    Q_i = Q_LS;
    Q_path(:, 1:width(Q_i), i) = Q_i;
end
end