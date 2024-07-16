function kin = define_cuspidal_3R()
    ex = [1; 0; 0];
    ey = [0; 1; 0];
    ez = [0; 0; 1];
    zv = [0; 0; 0];

    d2 = 1;
    d3 = 2;
    d4 = 1.5;
    r2 = 1;
    r3 = 0;

    kin.H = [ez ey ez];

    kin.P = [zv d2*ex r2*ey+d3*ex r3*ez+d4*ex];

    kin.joint_type = [0 0 0];

end