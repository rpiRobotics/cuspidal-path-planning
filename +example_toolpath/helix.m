function [p_path, R_path] = helix()
R = 0.1;
H = 0.3;
TURNS = 5;
N = 500;


lambda = linspace(0,1,N);

X = R*cos(2*pi*TURNS*lambda);
Y = R*sin(2*pi*TURNS*lambda);
Z = H*lambda;

p_path = [X;Y;Z];


ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
R_path = NaN(3, 3, N);
R_0 = rot(ey, pi+pi/6);

for i = 1:N
    R_path(:,:,i) = rot(ez, 2*pi*TURNS*lambda(i))*R_0;
end

end