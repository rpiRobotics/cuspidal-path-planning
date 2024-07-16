kin = define_cuspidal_3R;

% p_A = [1; 0; 0]; % 0 feasible
% p_B = [4; 0; 0];

p_A = [2; 0; 0]; % 2 feasible
p_B = [1; 0; 0];

is_feasible = find_feasible_moveL(kin, p_A, p_B)

%% Plot feasible solns
N = 1e3;
lambda = linspace(0, 1, N);
p_path = lambda .* p_B + (1 - lambda) .* p_A;


q1_path = NaN(4, N);
for i=1:N
    Q_i = IK_3R(p_path(:, i), kin);
    q1_path(1:width(Q_i), i) = Q_i(1,:);
end

plot(lambda, q1_path' , '.k'); hold on
N_feasible = sum(is_feasible);
plot(zeros(N_feasible, 1), q1_path(is_feasible, 1), 'or', MarkerSize=10)

ylim([-pi, pi]);
ylabel("q_1");
xlabel("\lambda"); hold off