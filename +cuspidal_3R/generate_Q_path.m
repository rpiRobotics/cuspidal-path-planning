function Q_path = generate_Q_path(kin, T_path)
    N = width(T_path);
    
    Q_path = NaN([3, 4, N]);
    
    for i = 1:N
        Q_i = cuspidal_3R.IK(T_path(:,i), kin);
        Q_path(:, 1:width(Q_i), i) = Q_i;
    end
    end