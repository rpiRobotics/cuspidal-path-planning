function [q_dot_norm, Q_path, G] = path_norm(p, quat_XY, p_path_orig, kin)
    [~, p_path] = graph_path_planning.quat_offset_path(p, quat_XY, [], p_path_orig);
    Q_path = cuspidal_3R.generate_Q_path(kin, p_path);

    G = graph_path_planning.generate_path_graph(Q_path);
    [~, q_dot_norm] = shortestpath(G, 1, 2);
end