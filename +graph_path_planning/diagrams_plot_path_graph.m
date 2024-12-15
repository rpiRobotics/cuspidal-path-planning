function diagrams_plot_path_graph(G, shortest_path, Q_path, joint_display, opts, lineOptions)
    arguments
        G
        shortest_path
        Q_path
        joint_display
        opts.lambda = [];
        opts.display_dots = true
        opts.display_SF = true
        opts.skip_wraparound = false
        opts.lambda_add_vertical = []
        opts.bold_det_J = []
        lineOptions.LineStyle (1,1) string = "-"
        lineOptions.LineWidth (1,1) {mustBeNumeric} = 1
        lineOptions.Color = 'k'
    end
    lineOptions = namedargs2cell(lineOptions);
    sz = size(Q_path);
    N_solns = sz(2);
    N = sz(3);

    if isempty(opts.lambda)
        lambda = 1:N;
    else
        lambda = opts.lambda;
    end

    XData = [];
    YData = [];
    sign_det_J_data = [];

    for i = 1:N
        for j = 1:N_solns
            if any(isnan(Q_path(:,j,i)))
                XData = [XData 0];
                YData = [YData 0];
                sign_det_J_data = [sign_det_J_data 0];
            else
                XData = [XData lambda(i)];
                YData = [YData Q_path(joint_display, j, i)];
                if ~isempty(opts.bold_det_J)
                    sign_det_J_data = [sign_det_J_data sign(opts.bold_det_J(j,i))];
                else
                    sign_det_J_data = [sign_det_J_data 0];
                end
            end
        end
    end
    XData = [zeros([1 N_solns]) XData];
    YData = [zeros([1 N_solns]) YData];
    sign_det_J_data = [zeros([1 N_solns]) sign_det_J_data];

    % Node 2 is the finish node
    XData(2) = lambda(N) + mode(diff(lambda));
    % 
    % if ~opts.display_SF
    %     XData([1 2]) = NaN;
    %     YData([1 2]) = NaN;
    % end

    % if ~opts.display_SF
    %     G = rmnode(G,[1 2]);
    %     XData = XData(3:end);
    %     YData = YData(3:end);
    % end
    

    % N_nodes = numnodes(G);
    % plot(G, XData=XData(1:N_nodes), YData=YData(1:N_nodes), LineWidth=4, LineStyle="-", ArrowSize=0, Marker='none', EdgeColor='k');
    
    compl_shortest_path = setdiff(1:length(XData), shortest_path);
    if opts.display_dots
        plot(XData(compl_shortest_path), YData(compl_shortest_path), 'ok', MarkerSize=5, MarkerFaceColor='k'); hold on
        plot(XData(shortest_path), YData(shortest_path), 'o', MarkerSize=5, MarkerFaceColor=diagrams.colors.green, MarkerEdgeColor=diagrams.colors.green);
    end

    N_edges = height(G.Edges);
    for i = 1:N_edges
        node_L = G.Edges.EndNodes(i,1);
        node_R = G.Edges.EndNodes(i,2);

        if ~opts.display_SF
            if node_L == 1 || node_L == 2 || node_R == 1 || node_R == 2 
                continue
            end
        end

        if opts.skip_wraparound
            if abs(YData(node_L)-YData(node_R)) > (2*pi - 0.1)
                continue
            end
        end

        % xyz_L = [XData(node_L); YData(node_L); 0];
        % xyz_R = [XData(node_R); YData(node_R); 0];
        % diagrams.arrow(xyz_L, xyz_R);
        
        % node_L_sign_det_J = 0;
        % if ~isempty(opts.bold_det_J)
        % % convert node_L to index
        % % recall index i_x = i*W+x
        % node_L_x = rem(node_L, N_solns);
        % node_L_i = (node_L-node_L_x)/N_solns;
        % if node_L_i > 0
        %     node_L_sign_det_J = sign(opts.bold_det_J(node_L_x, node_L_i));
        % end
        % end

        if any(node_L == shortest_path) && any(node_R == shortest_path)
            plot([XData(node_L) XData(node_R)], [YData(node_L) YData(node_R)],  Color=diagrams.colors.green, LineWidth=2); hold on
        else
            if sign_det_J_data(node_L) >=0
                plot([XData(node_L) XData(node_R)], [YData(node_L) YData(node_R)], lineOptions{:}); hold on
            else
                plot([XData(node_L) XData(node_R)], [YData(node_L) YData(node_R)], lineOptions{:}, LineWidth=2.5); hold on
            end
        end
    end

    % Add vertical lines where needed
    for i = 1:numel(opts.lambda_add_vertical)
        lambda_i = opts.lambda_add_vertical(i);
        Q_i = Q_path(:, :, lambda_i==lambda);
        for i_A = 1:width(Q_i)
        for i_B = 1:width(Q_i)
            if i_A == i_B
                continue
            end
            q_A = Q_i(:,i_A);
            q_B = Q_i(:,i_B);
            if norm(wrapToPi(q_A - q_B)) < 0.5 && norm(q_A(joint_display) - q_B(joint_display)) < 0.1
                plot([lambda_i lambda_i], [q_A(joint_display) q_B(joint_display)], lineOptions{:});
            end
        end
        end
    end

    hold off
end