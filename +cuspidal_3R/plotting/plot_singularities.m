syms q [3 1] real

kin = cuspidal_3R.get_kin;

J = cuspidal_3R.robotjacobian_sym(kin, q);
Jp = J(4:6,:);
dJ = det(Jp);
dJ = simplify(dJ)
%%
t2 = solve(dJ, q2)

syms T

T3 = cos(T)
T2 = cos(t2)
T2 = subs(T2, q3, T3)

%%

[~, p] = fwdkin(kin, q)

r = sqrt(p(1)^2+p(2)^2)
r = simplify(r)
z = p(3)
%%
r12 = subs(r, q2, t2)
z12 = subs(z, q2, t2)

fp1 = fplot(r12(1), z12(1), 'k'); hold on
fp2 = fplot(r12(2), z12(2), 'k'); hold off

fp1.MeshDensity = 100;
fp2.MeshDensity = 100;

axis equal

%%%

