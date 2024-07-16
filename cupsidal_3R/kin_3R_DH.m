alpha_vec = [-sym(pi)/2, sym(pi)/2, 0];
a_vec = [1, 2, 3/2];
d_vec =[0 1 0];

kin_DH = dh_to_kin(alpha_vec, a_vec, d_vec);
kin_POE = define_cuspidal_3R();