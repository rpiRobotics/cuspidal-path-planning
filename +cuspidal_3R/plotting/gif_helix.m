% filename = "helix_optimized.gif";
filename = "helix_0.gif";

orig = [0.7013    1.8369   -0.6936   -1.6368   -0.5241    1.5935]';
optimized = [0.8462    2.2120    0.0234    0.1465   -2.5520   -2.4581]';

kin = cuspidal_3R.get_kin();
q = zeros([3 1]);
p_path_orig = example_toolpath.helix();

clear im
i_im = 1;
for i = 1:10:width(Q_0)
% for i = 1
    h_fig=diagrams.setup(); hold on
    diagrams.robot_plot(kin, Q_0(:,i));
    
    plot_helix_path(orig(1:3), orig(4:6), p_path_orig, 'color', diagrams.colors.red)
    % plot_helix_path(optimized(1:3), optimized(4:6), p_path_orig, 'color', diagrams.colors.green)
    
    % grid on
    % axis on
    campos([-13.9236  -17.8167   13.5565]);
    camva(9.7122);
    camtarget([0.3738    0.8160   -0.0031]);


    diagrams.redraw(); hold off

    frame = getframe(h_fig);
    im{i_im} = frame2im(frame);
    i_im = i_im+1;
end
%%

for idx = 1:length(im)
    [A, map] = rgb2ind(im{idx}, 256);
    if idx == 1
        imwrite(A, map, filename, "gif", "LoopCount", Inf, "DelayTime", 1/10);
    else
        imwrite(A, map, filename, "gif", "WriteMode", "append", "DelayTime", 1/10);
    end
end

%%

function plot_helix_path(p, axang, p_path_orig, varargin)
    R = rot(axang/norm(axang), norm(axang));
    p_path = p + R*p_path_orig;
    diagrams.utils.plot3_mat(p_path, varargin{:});
end