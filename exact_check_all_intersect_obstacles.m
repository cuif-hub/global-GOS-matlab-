function exact_intersect_idx = exact_check_all_intersect_obstacles(pos,pos_next,obstacles,intersect_idx)
%EXACT_CHECK_INTERSECT_OBSTACLES 精确边-边检测线段是否与障碍物相交

exact_intersect_idx = [];
for i = 1:size(intersect_idx,2)
    obs = obstacles{intersect_idx(i)};
    if intersect_poly_edges(pos, pos_next, obs) %线段与obs的边相交则返回true
        exact_intersect_idx(end+1) = intersect_idx(i);
    end
end

end

