found = false;

while ~found
q = rand_angle([6 1]);
[R,T] = fwdkin(kin, q);
Q = IK.IK_2_parallel_2_intersecting_mex(R, T, kin);
N = width(Q);
if N>15
    found=true;
end
end
N
%%

IK.IK_2_parallel_2_intersecting(R, T, kin);

rad2deg(Q(:,1))

%%
plot(rad2deg(Q(3,:)), 'x'); hold on
plot(rad2deg(Q(3,:))-360, 'x'); hold off
yline(-225);
yline(85);