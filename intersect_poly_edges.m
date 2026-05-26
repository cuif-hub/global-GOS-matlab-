function is_intersect = intersect_poly_edges(pos, pos_next, obs)
%INTERSECT_POLY_EDGES 线段pos-pos_next与obs相交，返回true

is_intersect = false;

n = size(obs,1); %障碍物的边数
for i = 1:n
    p1 = obs(i, :);
    p2 = obs(mod(i, n)+1, :);
    if seg_is_intersect(pos, pos_next, p1, p2)
        is_intersect = true;
        return;
    end
end

% 兜底检查：如果pos和pos_next都是obs上的顶点且不相邻，直接返回true
[ipo1,idx1] = is_point_obs(pos, obs);
[ipo2,idx2] = is_point_obs(pos_next, obs);

if ipo1 && ipo2 %检查两个顶点是不是都是障碍物顶点
    if ~(abs(idx1 - idx2) == 1 || (idx1 == 1 && idx2 == n) || (idx1 == n && idx2 == 1)) %idx1和idx2不相邻返回true
        is_intersect = true;
        return;
    end
end

end


% 辅助函数：判断点 pt 是否为多边形 obs 的顶点
function [yes,idx] = is_point_obs(pt, obs)
    yes = false;
    idx = 0;

    val =  max(abs(obs - pt),[],2);
    for i = 1:size(val, 1)
        if val(i) < 1e-10
            yes = true;
            idx = i;
            return;
        end
    end
end



