function [is_feasible, stop_at] = CRX_feasible_moveL(kin, R_path, T_path, i_ignore)

THRESH = 5e-2;

N = width(T_path);

[Q_LS, is_LS_vec] = IK_3_pairs_intersecting_LS_mex(R_path(:,:,1), T_path(:,1), kin, false);
Q_A = Q_LS(:, ~any(is_LS_vec(1:2, :)));

N_solns_A = width(Q_A);
is_feasible = true([1 N_solns_A]);
stop_at = NaN([1 N_solns_A]);
last_match_index = 1:N_solns_A;
Q_prev = Q_A;

for i = 2:N
    if any (i==i_ignore)
        continue
    end
    
    [Q_LS, is_LS_vec] = IK_3_pairs_intersecting_LS_mex(R_path(:,:,i), T_path(:,i), kin, false);
    Q_i = Q_LS(:, ~any(is_LS_vec(1:2, :)));

    for i_A = 1:N_solns_A
        if ~is_feasible(i_A)
            continue
        end
        q_prev = Q_prev(:,last_match_index(i_A));
        [~, index_q, diff_norm] = closest_q(Q_i, q_prev);
        if diff_norm > THRESH
            is_feasible(i_A) = false;
            stop_at(i_A) = i;
            continue
        end
        last_match_index(i_A) = index_q;
    end
    Q_prev = Q_i;
end

end