kin = cuspidal_3R.get_kin;

p_1 = [1; 0; 0]; % 0 feasible
p_2 = [3.5; 0; 0];
p_3 = [3.5; 0;-2.5];
p_4 = [1; 0; -2.5];

N_i = 1e3;
N = N_i * 4;
lambda_i = linspace(0, 1, N_i);
lambda = linspace(0, 4, N);
p_path_12 = lambda_i .* p_2 + (1 - lambda_i) .* p_1;
p_path_23 = lambda_i .* p_3 + (1 - lambda_i) .* p_2;
p_path_34 = lambda_i .* p_4 + (1 - lambda_i) .* p_3;
p_path_41 = lambda_i .* p_1 + (1 - lambda_i) .* p_4;
p_path = [p_path_12 p_path_23 p_path_34 p_path_41];

q1_path = NaN(4, N);
q2_path = NaN(4, N);
q3_path = NaN(4, N);
for i=1:N
    Q_i = cuspidal_3R.IK(p_path(:, i), kin);
    q1_path(1:width(Q_i), i) = Q_i(1,:);
    q2_path(1:width(Q_i), i) = Q_i(2,:);
    q3_path(1:width(Q_i), i) = Q_i(3,:);
end
%%
plot(lambda, q1_path' , '.k');
% plot(lambda, q1_path');
ylim([-pi, pi]);
ylabel("q_1");
xlabel("\lambda");

%%
figure
fp1 = fplot(r12(1), z12(1), 'k'); hold on
fp2 = fplot(r12(2), z12(2), 'k'); 

fp1.MeshDensity = 100;
fp2.MeshDensity = 100;


    r = vecnorm(p_path(1:2,:));
    z = p_path(3,:);
    plot(r,z, 'b');


axis equal

xlabel("r")
ylabel("z")

hold off

%% Generate a gif of the (r,z) plot
filename = "cuspidal_3R_rz.gif";
clear im
h_fig = figure();
i_im = 1;
for i = 1:15:N

    fp1 = fplot(r12(1), z12(1), 'k'); hold on
    fp2 = fplot(r12(2), z12(2), 'k'); 
    
    fp1.MeshDensity = 100;
    fp2.MeshDensity = 100;
    
    
    r = vecnorm(p_path(1:2,:));
    z = p_path(3,:);
    plot(r,z, 'b');

    scatter(r(i), z(i), 'r', MarkerFaceColor='r');
    
    
    axis equal
    
    xlabel("r")
    ylabel("z")
    
    hold off

    frame = getframe(h_fig);
    im{i_im} = frame2im(frame);
    i_im = i_im+1
end

%% gif of the (lambda, q_1) plot
filename = "cuspidal_3R_lambda_q1.gif";
clear im
h_fig = figure();
i_im = 1;
for i = 1:15:N

    plot(lambda, q1_path' , '.k'); hold on
    % plot(lambda, q1_path');
    ylim([-pi, pi]);
    ylabel("q_1");
    xlabel("\lambda");

    scatter(lambda(i), q1_path(:,i), 'r', MarkerFaceColor='r');
    hold off;
    

    frame = getframe(h_fig);
    im{i_im} = frame2im(frame);
    i_im = i_im+1
end


%% gif of the 3D robots
filename = "cuspidal_3R_3D.gif";
clear im

opts = {'show_base_label', false,...
        'show_task_label', false,...
        'show_base_frame', false,...
        'show_task_frame', false,...
        'show_arrow_labels', false,...
        'show_joint_labels', false,...
        'show_arrows', false,...
        'cyl_half_length',0.25,...
        'cyl_radius',0.125};

colors = [diagrams.colors.blue
          diagrams.colors.red
         diagrams.colors.green
         diagrams.colors.black];

i_im = 1;
for i = 1:15:N
    h_fig = diagrams.setup(); hold on
    
    for i_rob = 1:sum(~isnan(q1_path(:,i)))
        q = [q1_path(i_rob,i); q2_path(i_rob,i); q3_path(i_rob,i)];
            diagrams.robot_plot(kin, q, opts{:}, 'link_color', colors(i_rob,:));
    end

    diagrams.utils.plot3_mat([p_1 p_2 p_3 p_4 p_1]);
    
    campos([ -11.9004  -16.4565   12.1509]);
    camva(14);
    camtarget([0.9115    0.2403   -0.0000]);

    diagrams.redraw;
    hold off;
    

    frame = getframe(h_fig);
    im{i_im} = frame2im(frame);
    i_im = i_im+1;
end


%% Export gif
for idx = 1:length(im)
    [A, map] = rgb2ind(im{idx}, 256);
    if idx == 1
        imwrite(A, map, filename, "gif", "LoopCount", Inf, "DelayTime", 1/30);
    else
        imwrite(A, map, filename, "gif", "WriteMode", "append", "DelayTime", 1/30);
    end
end

