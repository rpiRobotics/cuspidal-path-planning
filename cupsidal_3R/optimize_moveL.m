% pose_0 = [0; 0; 0; 0; 0];
% pose_0 = [0.2;0.2;0.2; pi/2; 0];
pose_0 = [0.2;0.2;0.2; 0;-1;0];
kin = define_cuspidal_3R;

options = optimset('PlotFcns',@optimplotfval);
pose = fminsearch(@(pose)moveL_norm(pose(1:3), pose(4:6), kin), pose_0, options);


%%
[q_dot_norm, Q_path, G] = moveL_norm(pose(1:3), pose(4), pose(5), kin);
%%
plot(permute(Q_path(3,:,:), [3 2 1]))
%% 
plot(diff(unwrap(permute(Q_path(:,2,:), [3 1 2]))))


%% Test error function

kin = define_cuspidal_3R;
q_dot_norm = moveL_norm([0.2;0.2;0.2], pi/2, 0, kin)


%%

function [q_dot_norm, Q_path, G] = moveL_norm(p, axang, kin)
    q_dot_norm = inf;
    p_A = p;
    % ex = [1; 0; 0];
    % ey = [0; 1; 0];
    % ez = [0; 0; 1];
    p_B = p_A + 0.5* axang/norm(axang);

    N = 100;
    lambda = linspace(0, 1, N);
    p_path = lambda.*p_B + (1-lambda).*p_A;
    Q_path = generate_Q_path_3R(kin, p_path);
    % Need to have at least 1 starting and ending point
    if all( isnan(Q_path(:,:,1)))
     return
    end
    if all( isnan(Q_path(:,:,end)))
     return
    end
    [G, start_nodes, end_nodes] = generate_path_graph_3R(Q_path);

    % For each start node, find connected end nodes
    for i = 1:length(start_nodes)
        start_node = start_nodes(i);
        for j = 1:length(end_nodes)
            end_node = end_nodes(j);
            [path, d] = shortestpath(G, start_node, end_node);
            if ~isempty(path)
                q_dot_norm = min(q_dot_norm, d);
            end
        end
    end
end