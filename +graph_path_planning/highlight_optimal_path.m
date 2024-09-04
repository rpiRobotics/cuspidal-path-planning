function Q_optimal = highlight_optimal_path(G, Q_path)
    plot_path_graph(G, Q_path, 2); hold on;
    [P, q_dot_norm] = shortestpath(G, 1, 2);
    
    Q_optimal = NaN(height(Q_path), length(Q_path));
    
    W = width(Q_path);
    q_disp_ind = 2;
    for i = 2:(length(P)-1)
        soln_ind = rem(P(i), W);
        path_ind = (P(i)-soln_ind)/W;
        q_i = Q_path(q_disp_ind, soln_ind, path_ind);
        plot(path_ind, q_i, 'xk');
        Q_optimal(:,path_ind) = Q_path(:, soln_ind, path_ind);
    end
    hold off
end