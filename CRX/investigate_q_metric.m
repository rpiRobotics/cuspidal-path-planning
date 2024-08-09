metrics = NaN([1 500]);
Ns = logspace(0, 5, 500);
for i = 1:500
    N = Ns(i);
    x = linspace(0, 1, N);
    F = f_1(x);
    D = diff(F);
    metrics(i) = norm(D)^2 * N;
end

semilogx(Ns, metrics)



function r = f_1(x)
    r = sin(x);
end