syms q2_sym q3_sym real

kin = cuspidal_3R.get_kin;

q1 = 0;

[~, p] = fwdkin(kin, [q1; q2_sym; q3_sym]);
r = sqrt(p(1)^2+p(2)^2);
z = p(3);

r = simplify(r);
z = simplify(z);
%%
J = cuspidal_3R.robotjacobian_sym(kin, [q1; q2_sym; q3_sym]);
Jp = J(4:6,:);
dJ = det(Jp);
dJ = simplify(dJ);

q2 = pi/6;
dJ_i = subs(dJ, q2_sym, q2)
q3_bounds = solve(dJ, q3_sym, Real=true)
q2_bounds = solve(dJ, q2_sym, Real=true)

r_sing_A = subs(r, q3_sym, q3_bounds(1));
z_sing_A = subs(z, q3_sym, q3_bounds(1));

r_sing_B = subs(r, q3_sym, q3_bounds(1)+pi);
z_sing_B = subs(z, q3_sym, q3_bounds(1)+pi);

fplot(r_sing_A, z_sing_A, 'k'); hold on
fplot(r_sing_B, z_sing_B, 'k'); 

axis equal
hold off
