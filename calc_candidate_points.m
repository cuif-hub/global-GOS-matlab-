function [left_cands, right_cands, max_dist_left, max_dist_right] = calc_candidate_points(pos, pos_next, obs, inflation, path_set)
%CALC_CANDIDATE_POINGS 计算单个障碍物的候选导向点
% obs: 障碍物顶点矩阵 Nx2
% 返回 left_cands, right_cands 为矩阵，每行 [x, y, dist]

min_acceptable_dist = 0.5 * inflation;
n = size(obs, 1);
left_cands = []; left_candsB = [];
right_cands = []; right_candsB = [];
max_dist_left = 0.0; max_dist_right = 0.0;

%线段pos-pos_next的参数化方程ax+by+c=0
eq = [pos(2)-pos_next(2), pos_next(1)-pos(1), pos(1)*pos_next(2)-pos_next(1)*pos(2)];
fm = sqrt(eq(1)^2 + eq(2)^2); %距离计算公式的分母

for i = 1:n
    opt_p = obs(i, :);
    % 条件1: 直接视线可达
    if intersect_poly_edges(pos, opt_p, obs)
        continue;
    end
    % 条件2: 最小距离
    if norm(opt_p - pos) <= min_acceptable_dist
        continue;
    end
    % 条件3: 未访问过
    if ismember_in_path_set(opt_p, path_set)
        continue;
    end
    condition4 = dot(opt_p - pos, pos_next - opt_p) > 0;
    side = (pos_next(1)-pos(1))*(opt_p(2)-pos(2))-(pos_next(2)-pos(2))*(opt_p(1)-pos(1)); %叉积大于0在左侧
    dist = abs(eq(1)*opt_p(1) + eq(2)*opt_p(2) + eq(3))/fm; %候选导引点到线段的垂直距离
    if side > 0
        if dist > max_dist_left, max_dist_left = dist; end
        if condition4
            left_cands = [left_cands; opt_p, dist];
        else
            left_candsB = [left_candsB; opt_p, dist];
        end
    elseif side < 0
        if dist > max_dist_right, max_dist_right = dist; end
        if condition4
            right_cands = [right_cands; opt_p, dist];
        else
            right_candsB = [right_candsB; opt_p, dist];
        end
    end
end

% 按原策略返回：优先采用满足 condition4 的候选
if isempty(left_cands) && isempty(right_cands)
    left_cands = left_candsB; right_cands = right_candsB;
elseif isempty(left_cands)
    left_cands = left_candsB;
elseif isempty(right_cands)
    right_cands = right_candsB;
end

end



function flag = ismember_in_path_set(pt, path_set)
    if isempty(path_set)
        flag = false; return;
    end
    flag = any(ismember(path_set, pt, 'rows'));
end



