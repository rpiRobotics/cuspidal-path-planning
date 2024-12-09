function det_J_path = Q_to_det_J(Q_path, kin)
% Find Jacobian determinant for each q in Q_path
sz = size(Q_path);
N = sz(3);

det_J_path = NaN(width(Q_path), N);
for i = 1:N
    for j = 1:width(Q_path)
        J = robotjacobian(kin, Q_path(:, j, i));
        if width(J)==3 % spatial manipulators
            J = J(4:6, :); % Ignore orientation part
        end
        det_J_path(j, i) = det(J);
    end
end

end