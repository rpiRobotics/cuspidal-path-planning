cuspidal_3R.initial_poses()
cuspidal_3R.optimized_poses()

pose_0 = helix_pose_0_A;
optimized = helix_optimized_A;


%%


calc_RMS(helix_pose_0_A, kin)
calc_RMS(helix_optimized_A, kin)

calc_RMS(helix_pose_0_B, kin)
calc_RMS(helix_optimized_B, kin)

function RMS = calc_RMS(pose, kin)

N = 500;

[~, p_path_orig] = example_toolpath.helix(H=1.2, R = 0.4, N=N);

[W, Q_path, G] = cuspidal_3R.path_norm(pose(1:3), pose(4:6), p_path_orig, kin);


% weight_i = norm(wrapToPi(Q_i(:,x) - Q_j(:,y)))^2 / (j-i);
% W = sum(weight_i)

DeltaT = diff(p_path_orig')';
L = sum(vecnorm(DeltaT));
DeltaLambda = L/(N-1);

C = W / DeltaLambda;

RMS = sqrt(C/L);
end