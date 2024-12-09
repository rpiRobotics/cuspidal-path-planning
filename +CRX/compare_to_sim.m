kin = define_CRX_roboguide;


R = rot(rand_normal_vec, rand_angle);
T = rand_vec;

Q = IK_3_pairs_intersecting_LS_mex(R, T, kin, true)

[R_t, T_t] = fwdkin(kin, Q(:,1))

%%
% q = deg2rad([0 0 0 0 0 0]) % TODO flip q_3
% q(3) = q(3) + q(2);

[R,T] = fwdkin(kin, q)



diagrams.setup(); hold on

diagrams.robot_plot(kin, q, auto_scale = true)


view(120,30)

diagrams.redraw(); hold off


function kin = define_CRX_roboguide()
% https://crx.fanucamerica.com/wp-content/uploads/2022/10/CRX-10iA-L-data-sheet.pdf
    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    zv = [0;0;0];
    kin.H = [ez ey -ey -ex -ey -ex];
    kin.P = [245*ez zv 710*ez zv 540*ex-150*ey zv 160*ex]/1000;
    kin.joint_type = zeros(1,6);

    % kin.R_6T = [ez -ey ex];
end

