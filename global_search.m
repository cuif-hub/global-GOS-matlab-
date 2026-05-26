function [path, smooth_path, smooth_len] = global_search(start, goal, obstacles, inflation)
%GLOBAL_GOS 快速路径搜索函数

pos = start;
pos_next = goal;
% 寻找与线段相交的障碍物（快速剔除+精确边-边判断）
init_intersect_idx = init_check_all_intersect_obstacles(pos,pos_next,obstacles);
intersect_idx = exact_check_all_intersect_obstacles(pos,pos_next,obstacles,init_intersect_idx);
intersected_obs = obstacles(intersect_idx);

path = start;              % 路径点矩阵 Mx2
path_set = start;          % 已访问点集

while true
    [path_line, is_again_check] = global_ovs_strategy(pos, pos_next, intersected_obs, ...
        obstacles, inflation, path, path_set);
    if isempty(path_line)
        disp('Solve Failed, no path found.');
        path = []; smooth_path = []; smooth_len = []; return;
    end

    % 判断线段是否安全可通行
    if ~is_again_check || ~is_intersect_obstacles(path_line(1,:), path_line(2,:), obstacles)
        % 加入路径（去掉重复起点）
        path = [path; path_line(2,:)];
        path_set = [path_set; path_line(2,:)];
        if norm(path(end,:)-goal)<1e-10
            [smooth_path, smooth_len] = smooth(path, obstacles);
            return;
        end
        pos = path(end,:);
        pos_next = goal;
    else
        pos_next = path_line(2,:);   % 临时目标，不加入路径
    end

    % 更新相交障碍物集合
    init_intersect_idx = init_check_all_intersect_obstacles(pos, pos_next, obstacles);
    intersect_idx = exact_check_all_intersect_obstacles(pos,pos_next,obstacles,init_intersect_idx);
    intersected_obs = obstacles(intersect_idx);
end

end

