function is_intersect = seg_is_intersect(p1, p2, p3, p4)
%SEG_IS_INTERSECT 判断线段p1-p2与线段p3-p4是否相交,is_intersect = true
%只有一个公共端点不算相交
    
    function val = cross2(o, a, b) %跨立实验（二维叉积）
        val = (a(1)-o(1))*(b(2)-o(2)) - (a(2)-o(2))*(b(1)-o(1));
    end

    % 快速排斥：检查以两线段为对角线的矩形是否相交
    if max(p1(1), p2(1)) >= min(p3(1), p4(1)) && ...
       max(p3(1), p4(1)) >= min(p1(1), p2(1)) && ...
       max(p1(2), p2(2)) >= min(p3(2), p4(2)) && ...
       max(p3(2), p4(2)) >= min(p1(2), p2(2))
        
        eps = 1e-10;   % 容差，用于处理浮点误差
        % 计算叉积乘积
        c1 = cross2(p1, p2, p3) * cross2(p1, p2, p4);
        c2 = cross2(p3, p4, p1) * cross2(p3, p4, p2);
        
        % 跨立实验：若两组跨立条件均满足（允许容差），则相交
        if c1 < -eps && c2 < -eps
            is_intersect = true;
        else
            is_intersect = false;
        end
    else
        is_intersect = false;
    end


end