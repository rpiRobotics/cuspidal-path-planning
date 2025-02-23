q = rand_angle([3 1])
kin = cuspidal_3R.get_kin();
[~, p] = fwdkin(kin, q);


% q1 = [-1.8; -2.8; 1.9];
% q2 = [-0.9, -0.7, 2.5];
% q3 = [-2.9, -3, -0.2];
% q4 = [0.2, -0.3, -1.9];

% [~, p] = fwdkin(kin, q1)
% [~, p] = fwdkin(kin, q2)
% [~, p] = fwdkin(kin, q3)
% [~, p] = fwdkin(kin, q4)

% p = [2.5; 0; 0.5]

Q_t = cuspidal_3R.IK(p, kin)

