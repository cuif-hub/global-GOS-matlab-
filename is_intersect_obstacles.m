function is_intersect = is_intersect_obstacles(pos,pos_next,obstacles)
%IS_INTERSECT_OBSTACLES 检查线段pos-pos_next是否与至少一个障碍物相交，一旦相交则返回true
%初步筛选+精确边-边检测

is_intersect = false;

num_obs = length(obstacles);
d = pos_next - pos;

% 计算pos->pos_next直线的参数化方程ax+by+c=0，并排除满足条件(1)的障碍物
eq = [pos(2)-pos_next(2), pos_next(1)-pos(1), pos(1)*pos_next(2)-pos_next(1)*pos(2)]';
% 计算pos-pos_next线段两端点的垂线的参数化方程，要求两垂线内部ax+by+c<=0
eq1 = [-d(1), -d(2), d(1)*pos(1)+d(2)*pos(2)]'; %过pos
eq2 = [d(1), d(2), -(d(1)*pos_next(1)+d(2)*pos_next(2))]'; %过pos_next

for i = 1:num_obs
    obs = obstacles{i};
    obs_aug = [obs ones(size(obs,1),1)];
    val = obs_aug * eq; 
    if min(val) * max(val) <= 0 %线段pos-pos_next可能与该障碍物相交
        val1 = obs_aug * eq1;
        val2 = obs_aug * eq2;
        if (min(val1)<0.0) && (min(val2)<0.0) %线段pos-pos_next与该障碍物相交的概率增大
            if intersect_poly_edges(pos, pos_next, obs) %线段pos-pos_next与该障碍物确定相交
                is_intersect = true;
                return;
            end
        end        
    end
end

end

