kin = cuspidal_3R.get_kin;

T_A = [1; 0; 0]; % 0 feasible
T_B = [4; 0; 0];

% T_A = [2; 0; 0]; % 2 feasible
% T_B = [1; 0; 0];

% T_A = [200; 0; 0]; % No solutions
% T_B = [100; 0; 0];

N = 100;
T_path = cuspidal_3R.generate_moveL(T_A, T_B, N);
Q_path = cuspidal_3R.generate_Q_path(kin, T_path);
[G, start_nodes, end_nodes] = generate_path_graph(Q_path);

%%
plot_path_graph(G, Q_path, 1)

%%
[P, d] = shortestpath(G, 1, 2)