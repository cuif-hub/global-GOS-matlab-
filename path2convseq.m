function convseq = path2convseq(path,si,convset,adj,deg,comface,comfaceRowIdx)
%PATH2CONVSEQ 提取路径path所穿过的凸多边形序列

convseq = [si];

cur_convIdx = si; %当前凸多边形序号
conv = convset{cur_convIdx}; %当前凸多边形
e1 = conv([2:end,1],:) - conv; %凸多边形的逆时针边矢量
last_convIdx = 0; %上一个凸多边形序号

for i = 1:size(path,1)-1
    cur = path(i,:);
    next = path(i+1,:);
    e2 = next - conv; %当前凸多边形顶点->next的矢量

    while true %直到点next位于凸多边形cur_convIdx内部，停止迭代
        for j = 1:deg(cur_convIdx) %当前凸多边形的相邻多边形的索引
            if adj(j,cur_convIdx) == last_convIdx
                continue;
            end
            comf = comface(comfaceRowIdx(j,cur_convIdx),:); %当前凸多边形与相邻多边形的公共边端点[p1,p2]
            
            if seg_is_intersect_standard(cur,next,comf(1:2),comf(3:4)) %线段cur-next与p1-p2相交
                last_convIdx = cur_convIdx;
                cur_convIdx = adj(j,cur_convIdx);
                convseq = [convseq cur_convIdx];
                %更新凸多边形、e1和e2
                conv = convset{cur_convIdx};
                e1 = conv([2:end,1],:) - conv;
                e2 = next - conv;
                break;
            end
        end
        %若找不到与线段cur-next相交的公共边，有可能其位于当前凸多边形内部
        
        % 使用叉积法判断next是否在新的当前凸多边形内部，是则进入下一线段
        if all( (e1(:,1).*e2(:,2)-e1(:,2).*e2(:,1))>=-1e-10 ) %点next在新的凸多边形内部
            break;
        end
    end
end

end

