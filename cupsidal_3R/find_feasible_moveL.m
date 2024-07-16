function is_feasible = find_feasible_moveL(kin, p_A, p_B)
% Just 3R for now

THRESH = 1e-2;

N = 1e3;
lambda = linspace(0, 1, N);
p_path = lambda .* p_B + (1 - lambda) .* p_A;

Q_A = IK_3R(p_A, kin);
N_solns_A = width(Q_A);
is_feasible = true([1 N_solns_A]);
last_match_index = 1:N_solns_A;
Q_prev = Q_A;

for i = 2:N
    Q_i = IK_3R(p_path(:, i), kin);
    for i_A = 1:N_solns_A
        if ~is_feasible(i_A)
            continue
        end
        q_prev = Q_prev(:,last_match_index(i_A));
        [~, index_q, diff_norm] = closest_q(Q_i, q_prev);
        if diff_norm > THRESH
            is_feasible(i_A) = false;
            continue
        end
        last_match_index(i_A) = index_q;
    end
    Q_prev = Q_i;
end

end