load('dt25.mat')

% 边界、障碍物
%[env_size,obstacles] = env();
[env_size,obstacles] = darpa_offset();

%起点、终点
start = [100,80]; si = 10;
goal = [450 225];
%小车半径即膨胀大小
inflation = 1;


%规划路径
tic
[path, smooth_path, smooth_len] = global_search(start, goal, obstacles, inflation);
%convseq = path2convseq(path,si,convset,adj,deg,comface,comfaceRowIdx)
toc

%可视化
agents = cell(1,3);
agents{1} = [20 20;20 30;20 40;20 50];
agents{2} = [40 20;40 30;40 40;40 50];
agents{3} = [60 20;60 30;60 40;60 50];
agents_color = ["r", "b", "c"];

if ~isempty(smooth_path)
    draw_result(env_size,obstacles,[],agents,agents_color,[]);
    hold on;                            % 保持当前图形
    plot(smooth_path(:,1), smooth_path(:,2), 'ro-', 'LineWidth', 2);  % 绘制连线
    hold off;                           % 可选，释放 hold
end

