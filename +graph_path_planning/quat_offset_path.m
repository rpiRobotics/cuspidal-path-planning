function [R_path, p_path] = quat_offset_path(p, quat_XY, R_path_orig, p_path_orig)
    R = quat2rotm([quat_XY' 0]); % Auto-normalize
    p_path = R*(p + p_path_orig); % Careful - position then rotation
    
    if ~isempty(R_path_orig)
        R_path = pagemtimes(R,R_path_orig);
    else
        R_path = [];
    end
end