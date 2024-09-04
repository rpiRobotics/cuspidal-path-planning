function p_path = quat_offset_path(p, quat_XY, p_path_orig)
    R = quat2rotm([quat_XY' 0]); % Auto-normalize
    p_path = R*(p + p_path_orig); % Careful - position then rotation
end