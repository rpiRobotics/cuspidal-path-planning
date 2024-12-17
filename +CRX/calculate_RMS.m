CRX.initial_poses()
CRX.optimized_poses()


%%


calc_RMS(helix_pose_0_A, kin)
calc_RMS(helix_optimized_A, kin)
calc_RMS(helix_pose_0_B, kin)
calc_RMS(helix_optimized_B, kin)

function RMS = calc_RMS(pose, kin)

N = 500;

[R_path_orig, p_path_orig] = example_toolpath.helix(N=N);

[W, Q_path, G] = CRX.path_norm(pose(1:3), pose(4:6), p_path_orig, R_path_orig, kin);


% weight_i = norm(wrapToPi(Q_i(:,x) - Q_j(:,y)))^2 / (j-i);
% W = sum(weight_i)

DeltaT = diff(p_path_orig')';
L = sum(vecnorm(DeltaT));
DeltaLambda = L/(N-1);

C = W / DeltaLambda;

RMS = sqrt(C/L);
end