function plot_path_graph(G, Q_path, joint_display, lambda)
    sz = size(Q_path);
    N_solns = sz(2);
    N = sz(3);
    
    if nargin == 3
        lambda = 1:N;
    end

    XData = [];
    YData = [];

    for i = 1:N
        for j = 1:N_solns
            if any(isnan(Q_path(:,j,i)))
                XData = [XData 0];
                YData = [YData 0];
            else
                XData = [XData lambda(i)];
                YData = [YData Q_path(joint_display, j, i)];
            end
        end
    end
    XData = [zeros([1 N_solns]) XData];
    YData = [zeros([1 N_solns]) YData];

    % Node 2 is the finish node
    XData(2) = lambda(N) + mode(diff(lambda));
    
    N_nodes = numnodes(G);
    plot(G, XData=XData(1:N_nodes), YData=YData(1:N_nodes));
end