% https://github.com/FLo-ABB/GoFaPalettizer/blob/main/KinematicDimensions.xlsx



fprintf('\nGoFa 5\n')
DH = ... % # d alpha a
[0.265	sym(pi)/2	0
0	0	0.444
0	-sym(pi)/2	0.11
0.470	sym(pi)/2	0
0	-sym(pi)/2	0.08
0.101	0	0];
kin = dh_to_kin(DH(:,2), DH(:,3), DH(:,1));
kin.P = double(kin.P);
kin.H = double(kin.H);
[is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin);
print_intersecting_parallel_axes(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)


%% Kinematics to match RobotStudio

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

kin.H = [ez ey ey ex ey ex];
kin.P = [zv 0.265*ez 0.444*ez 0.11*ez 0.47*ex 0.08*ez 0.101*ex];
kin.joint_type = [0 0 0 0 0 0];

% Joint limits from Robot Studio
q_min = deg2rad([-180 -180 -225 -180 -180 -270]');
q_max = deg2rad([ 180  180   85  180  180  270]');

R_6T = round(rot(ey, pi/2));

%%

fprintf('\nGoFa 10\n')
DH = ... % # d alpha a
[0.4	-sym(pi)/2	0.15
0	0	0.707
0	-sym(pi)/2	0.11
0.637	sym(pi)/2	0
0	-sym(pi)/2	0.08
0.101	0	0];
kin = dh_to_kin(DH(:,2), DH(:,3), DH(:,1));
kin.P = double(kin.P);
kin.H = double(kin.H);
[is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin);
print_intersecting_parallel_axes(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)


%%
fprintf('\nGoFa 15\n')
DH = ... % # d alpha a
[0.338	-sym(pi)/2	0
0	0	0.707
0	-sym(pi)/2	0.11
0.534	sym(pi)/2	0
0	-sym(pi)/2	0.08
0.101	0	0];
kin = dh_to_kin(DH(:,2), DH(:,3), DH(:,1));
kin.P = double(kin.P);
kin.H = double(kin.H);
[is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin);
print_intersecting_parallel_axes(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)

