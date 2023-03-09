function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    d_order = (n_order + 1)/2;
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    %  位置约束
    % s*c < box
    v = ones(n_all_poly,1);
    for i = 0:n_seg - 1
        v(i *(n_order + 1)+1:i *(n_order + 1)+n_order + 1) = v(i *(n_order + 1)+1:i *(n_order + 1)+n_order + 1) * ts(i+1);
    end
    
    
    Aieq_p = diag(v);
    bieq_p = zeros(n_all_poly,1);
    for i = 0:n_seg - 1
        bieq_p(i *(n_order + 1)+1:i *(n_order + 1)+n_order + 1) = corridor_range(i+1,2);
    end
    
    Aieq_p = [Aieq_p;-Aieq_p];
    bieq_p = [bieq_p;bieq_p];
    
    for i = 0:n_seg - 1
        bieq_p(n_all_poly + i *(n_order + 1)+1:n_all_poly + i *(n_order + 1)+n_order + 1)...
            = (-1) * corridor_range(i + 1,1);
    end
    

    %#####################################################
    % 速度约束
    % vmin < n(ci+1-ci) < vmax
    % 记得阶数-1了
    Aieq_v = zeros((n_order+1-1) * n_seg, n_all_poly);
    j = 1;
    for i = 1:n_order * n_seg
        Aieq_v(i,j:j+1) = n_order * [-1, 1];
        if mod(j + 1, n_order + 1) == 0 %因为少了一阶，所以要跳一格
            j = j + 2;
        else
            j = j + 1;
        end
    end
    
    Aieq_v = [Aieq_v;-Aieq_v];
    bieq_v = ones(2 * n_order * n_seg,1)* v_max; % 大概是没定义最小值，所以不管最小值了。虽然利于之后最小值的加入，但是现在没必要加上吧。

    %#####################################################
    % 加速度约束
    % amin < n(n-1)(ci+2-2ci+1+ci)/s < amax
    Aieq_a = zeros((n_order - 1) * n_seg, n_all_poly);
    j = 1;
    for i = 1:(n_order - 1) * n_seg
        Aieq_a(i,j:j+2) = n_order * (n_order - 1)*[1, -2, 1]/ts(floor(j/(n_order + 1)) + 1);
        if mod(j + 2, n_order + 1) == 0 %因为少了2阶，所以要跳2格
            j = j + 3;
        else
            j = j + 1;
        end
    end
    
    Aieq_a = [Aieq_a;-Aieq_a];
    bieq_a = ones((n_order - 1) * n_seg * 2,1)*a_max;
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
end