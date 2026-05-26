function [path_line, is_again_check] = global_ovs_strategy(pos, pos_next, intersected_obs, obstacles, inflation, path, path_set)
%GLOBAL_OVS_STRATEGY 最优导引点候选策略
% 返回 path_line: 2x2 矩阵 [pos; guide_point] 或 []
%     is_again_check: bool 

if ~isempty(intersected_obs)
    vertex_points = []; vertex_pointsB = [];
    for k = 1:length(intersected_obs)
        vp = calc_guidance_point(pos, pos_next, intersected_obs{k}, inflation, path_set);
        if ~isempty(vp)
            if ~is_intersect_obstacles(pos, vp(1:2), obstacles)
                vertex_points = [vertex_points; vp];
            else
                vertex_pointsB = [vertex_pointsB; vp];
            end
        end
    end

    if ~isempty(vertex_points)
        [~, idx] = max(vertex_points(:,3));
        gp = vertex_points(idx, 1:2);
        is_again_check = false;
        path_line = [pos; gp];
    elseif ~isempty(vertex_pointsB)
        [~, idx] = max(vertex_pointsB(:,3));
        gp = vertex_pointsB(idx, 1:2);
        is_again_check = true;
        path_line = [pos; gp];
    else
        path_line = []; is_again_check = [];
    end
else
    is_again_check = false;
    path_line = [pos; pos_next];
end

end

