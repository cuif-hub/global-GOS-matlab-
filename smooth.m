function [smooth_path, path_length] = smooth(path,obstacles)
%SMOOTH 贪心剪枝平滑路径

smooth_path = path(1, :);
route = path;

while true
    n = size(route, 1);
    if n == 1, break; end
    found = false;
    for i = n-1:-1:1
        if i == 1
            smooth_path = [smooth_path; route(2, :)];
            route(1, :) = [];
            found = true; break;
        else
            pos = route(1, :);
            pos_next = route(i+1, :);
            if ~is_intersect_obstacles(pos, pos_next, obstacles)
                smooth_path = [smooth_path; pos_next];
                route(1:i, :) = [];
                found = true; break;
            end
        end
    end
    if ~found
        % 理论上不会发生，如果发生则保留下一个点避免死循环
        smooth_path = [smooth_path; route(2, :)];
        route(1, :) = [];
    end
end

path_length = sum(sqrt(sum(diff(smooth_path).^2,2)));

end

