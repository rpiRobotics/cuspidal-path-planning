%% Plot just position part
[~, p_path] = example_toolpath.helix();

plot3(p_path(1,:), p_path(2,:), p_path(3,:), '-x');
axis equal

%% Plot position and rotation

[R_path, p_path] = example_toolpath.helix();

diagrams.setup(); hold on
diagrams.utils.plot3_mat(p_path);

UNIT_SIZE = 0.05;

for i = 1:30:length(p_path)
    diagrams.arrow(p_path(:,i), p_path(:,i)+UNIT_SIZE*R_path(:,1,i), 'color', diagrams.colors.red);
    diagrams.arrow(p_path(:,i), p_path(:,i)+UNIT_SIZE*R_path(:,2,i), 'color', diagrams.colors.green);
    diagrams.arrow(p_path(:,i), p_path(:,i)+UNIT_SIZE*R_path(:,3,i), 'color', diagrams.colors.blue);
end

diagrams.dot(p_path(:,1), 'color', diagrams.colors.green);
diagrams.dot(p_path(:,end), 'color', diagrams.colors.red);

diagrams.redraw(); hold off