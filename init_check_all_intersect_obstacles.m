function intersect_idx = init_check_all_intersect_obstacles(pos,pos_next,obstacles)
%INIT_CHECK_ALL_INTERSECT_OBSTACLES 
% 两步剔除快速筛选与线段（可能）相交的障碍物（初步筛选，后面还要精选）
%   (1) 所有顶点在直线同侧则剔除
%   (2) 所有顶点投影在两垂线之外则剔除
% 返回逻辑索引，指示哪些障碍物与线段（可能）相交

% 初始化
num_obs = length(obstacles);
intersect_idx = 1:num_obs;
d = pos_next - pos;

% 计算pos->pos_next直线的参数化方程ax+by+c=0，并排除满足条件(1)的障碍物
eq = [pos(2)-pos_next(2), pos_next(1)-pos(1), pos(1)*pos_next(2)-pos_next(1)*pos(2)]';
del_idx = [];
for i = intersect_idx
    obs = obstacles{i};
    val = [obs ones(size(obs,1),1)] * eq;
    if min(val) * max(val) > 0
        del_idx(end+1) = i;
    end
end
intersect_idx(del_idx) = [];

% 计算pos-pos_next线段两端点的垂线的参数化方程，要求两垂线内部ax+by+c<=0，并排除满足条件(2)的障碍物
% 过pos的垂线
eq1 = [-d(1), -d(2), d(1)*pos(1)+d(2)*pos(2)]';
del_idx = [];
for i = 1:size(intersect_idx,2)
    obs = obstacles{intersect_idx(i)};
    val = [obs ones(size(obs,1),1)] * eq1;
    if min(val)>=0
        del_idx(end+1) = i;
    end
end
intersect_idx(del_idx) = [];

% 过pos_next的垂线
eq2 = [d(1), d(2), -(d(1)*pos_next(1)+d(2)*pos_next(2))]';
del_idx = [];
for i = 1:size(intersect_idx,2)
    obs = obstacles{intersect_idx(i)};
    val = [obs ones(size(obs,1),1)] * eq2;
    if min(val)>=0
        del_idx(end+1) = i;
    end
end
intersect_idx(del_idx) = [];

end

