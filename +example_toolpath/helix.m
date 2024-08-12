function p_path = helix()
R = 0.1;
H = 0.3;
TURNS = 5;
N = 500;


lambda = linspace(0,1,N);

X = R*cos(2*pi*TURNS*lambda);
Y = R*sin(2*pi*TURNS*lambda);
Z = H*lambda;

p_path = [X;Y;Z];
end