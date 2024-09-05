kin = define_CRX;
q = zeros([6 1]);
CRX.initial_poses;
CRX.optimized_poses;
%% Helix initial guess
pose = helix_pose_0;
filename = "CRX_helix_0.gif";
color = diagrams.colors.red;
[R_path_orig, p_path_orig] = example_toolpath.helix();

%% Helix optimized
pose = helix_optimized;
filename = "CRX_helix_optimized.gif";
color = diagrams.colors.green;
[R_path_orig, p_path_orig] = example_toolpath.helix();

%% moveL initial guess
pose = moveL_pose_0;
filename = "CRX_moveL_0.gif";
color = diagrams.colors.red;
[R_path_orig, p_path_orig] = example_toolpath.moveL([], [], [0;0;0], [0;0;0.5], [], [], 100);

%% moveL optimized guess
pose = moveL_optimized;
filename = "CRX_moveL_optimized.gif";
color = diagrams.colors.green;
[R_path_orig, p_path_orig] = example_toolpath.moveL([], [], [0;0;0], [0;0;0.5], [], [], 100);


%%
[q_dot_norm, Q_path, G] = CRX.path_norm(pose(1:3), pose(4:6), p_path_orig, R_path_orig, kin);
Q_optimal = graph_path_planning.highlight_optimal_path(G, Q_path);


%%

clear im
i_im = 1;
for i = 1:10:width(Q_optimal)
% for i = 1
    h_fig=diagrams.setup(); hold on
    diagrams.robot_plot(kin, Q_optimal(:,i), ...
    'unit_size', 0.1, ...
    'cyl_half_length', 0.1, ...
    'cyl_radius', 0.05);
    
    plot_path(pose(1:3), pose(4:6), p_path_orig, 'color', color)
    
    % grid on
    % axis on
    campos([-3.5005   -4.0214    3.4885]);
    camva(10);
    camtarget([-0.1436    0.3534    0.3048]);


    diagrams.redraw(); hold off

    frame = getframe(h_fig);
    im{i_im} = frame2im(frame);
    i_im = i_im+1;
end
%%

for idx = 1:length(im)
    [A, map] = rgb2ind(im{idx}, 256);
    if idx == 1
        imwrite(A, map, filename, "gif", "LoopCount", Inf, "DelayTime", 1/10);
    else
        imwrite(A, map, filename, "gif", "WriteMode", "append", "DelayTime", 1/10);
    end
end

%%

function plot_path(p, quat_XY, p_path_orig, varargin)
    [~, p_path] = graph_path_planning.quat_offset_path(p, quat_XY, [], p_path_orig);
    diagrams.utils.plot3_mat(p_path, varargin{:});
end