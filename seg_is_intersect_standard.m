function is_intersect = seg_is_intersect_standard(p1, p2, p3, p4)
    % 快速排斥实验：以两线段为对角线的矩形是否相交
    if max(p1(1), p2(1)) < min(p3(1), p4(1)) || ...
       max(p3(1), p4(1)) < min(p1(1), p2(1)) || ...
       max(p1(2), p2(2)) < min(p3(2), p4(2)) || ...
       max(p3(2), p4(2)) < min(p1(2), p2(2))
        is_intersect = false;
        return;
    end
    
    % 跨立实验的叉积函数
    cross = @(o,a,b) (a(1)-o(1))*(b(2)-o(2)) - (a(2)-o(2))*(b(1)-o(1));
    
    d1 = cross(p1, p2, p3);
    d2 = cross(p1, p2, p4);
    d3 = cross(p3, p4, p1);
    d4 = cross(p3, p4, p2);
    
    % 如果两条线段两端点都相互跨立（严格异号），则一定相交
    if d1*d2 < 0 && d3*d4 < 0
        is_intersect = true;
        return;
    end
    
    % 检查端点是否落在另一线段上（包含端点重合与内部点）
    on_segment = @(a,b,c) min(a(1),b(1)) <= c(1) && c(1) <= max(a(1),b(1)) && ...
                           min(a(2),b(2)) <= c(2) && c(2) <= max(a(2),b(2));
    
    if d1 == 0 && on_segment(p1, p2, p3)
        is_intersect = true;
    elseif d2 == 0 && on_segment(p1, p2, p4)
        is_intersect = true;
    elseif d3 == 0 && on_segment(p3, p4, p1)
        is_intersect = true;
    elseif d4 == 0 && on_segment(p3, p4, p2)
        is_intersect = true;
    else
        is_intersect = false;
    end
end
