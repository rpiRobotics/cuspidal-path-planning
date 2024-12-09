function [G, start_nodes, end_nodes] = generate_path_graph(Q_path, opts)
arguments
    Q_path
    opts.thresh = []
    opts.det_J_path = []
end

if isempty(opts.thresh)
    THRESH = 4e-1 * norm(ones(1, height(Q_path))); % More DOF means higher THRESH
else
    THRESH = opts.thresh;
end
sz = size(Q_path);
N = sz(3);
W = sz(2);


START_NODE = 1;
FINISH_NODE = 2;

tails = [];
heads = [];
weights = [];
% Index i_x = i*W+x

start_nodes = 1*W + (1:sum(~isnan(Q_path(1,:,1)))); % TODO rename this
end_nodes   = N*W + (1:sum(~isnan(Q_path(1,:,N))));

for i = 1:length(start_nodes)
    tails = [tails START_NODE];
    heads = [heads start_nodes(i)]; % TODO optimize this
    weights = [weights 0];
end
for i = 1:length(end_nodes)
    tails = [tails end_nodes(i)];
    heads = [heads FINISH_NODE];
    weights = [weights 0];
end

for i = 1:(N-1)
    % disp(i)
for j = (i+1):min(i+1, N) % TODO was 5
    Q_i = Q_path(:,:,i);
    Q_j = Q_path(:,:,j);

    % Connect i_x to j_y if ||q_i_x - q_j_y|| < thresh
    % Careful to account for angle wrapping
    for x = 1:W
    for y = 1:W
        if ~isempty(opts.det_J_path)
            % don't connect if different signs of det(J)
            S1 = sign(opts.det_J_path(x,i));
            S2 = sign(opts.det_J_path(y,j));
            if S1 * S2 < 0 % diff signs
                continue
            end
        end
        weight_i = norm(wrapToPi(Q_i(:,x) - Q_j(:,y))) / (j-i);
        if weight_i < THRESH
            tails = [tails i*W+x];
            heads = [heads j*W+y];
            weights = [weights weight_i];
        end
    end
    end
end
end

N_nodes = max([2 heads tails]);
G = digraph(tails, heads, weights, N_nodes);

end
