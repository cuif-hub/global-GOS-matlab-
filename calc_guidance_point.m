function guide_point = calc_guidance_point(pos, pos_next, obs, inflation, path_set)
%CALC_GUIDANCE_POINTS 给每个与线段相交的障碍物计算引导点（一个障碍物对应一个引导点）
% 返回 [x, y, dist] 或 []

[left_cands, right_cands, max_d_left, max_d_right] = calc_candidate_points(pos, pos_next, obs, inflation, path_set);
if isempty(left_cands) && isempty(right_cands)
    guide_point = []; return;
end
if ~isempty(left_cands)
    [~, idx] = max(left_cands(:,3)); 
    p_left = left_cands(idx, :);
else
    p_left = [];
end
if ~isempty(right_cands)
    [~, idx] = max(right_cands(:,3));
    p_right = right_cands(idx, :);
else
    p_right = [];
end

if ~isempty(p_left) && ~isempty(p_right)
    opt_ps = [];
    if ~intersect_poly_edges(p_left(1:2), pos_next, obs)
        opt_ps = [opt_ps; p_left];
    end
    if ~intersect_poly_edges(p_right(1:2), pos_next, obs)
        opt_ps = [opt_ps; p_right];
    end
    if ~isempty(opt_ps)
        [~, idx_min] = min(opt_ps(:,3));
        guide_point = opt_ps(idx_min, :);
    else
        p_left(3) = max_d_left; p_right(3) = max_d_right;
        if p_left(3) <= p_right(3)
            guide_point = p_left;
        else
            guide_point = p_right;
        end
    end
elseif ~isempty(p_left)
    guide_point = p_left;
else
    guide_point = p_right;
end

end

