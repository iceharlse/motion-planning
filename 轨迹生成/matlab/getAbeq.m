function [Aeq, beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    %为了求 Aeq * p = beq中的p，所以要把已知矩阵都表示出来

    n_all_poly = n_seg*(n_order+1); %总系数个数
    %#####################################################
    % p,v,a,j 的起点约束, 
    Aeq_start = zeros(4, n_all_poly); %4行代表4次求导后的多项式，代表p,v,a,j的约束，列代表所有系数 
    
    % Ae1_start和beq_start的表达式
    Aeq_start(1:4,1:8) = getCoeffCons(0); %T=0时的初始约束
    beq_start =  start_cond';
    
    %#####################################################
    % p,v,a,j 的终端约束
    Aeq_end = zeros(4, n_all_poly);
    t = ts(end);
    Aeq_end(1:4, end-7:end) = getCoeffCons(t);
    beq_end =  end_cond';
    
    %#####################################################
    % 中间节点的位置约束，只有一个位置约束，所以只有n_seg-1个约束，而不用乘4。
    % 只计算每段的尾结点约束，由于总结束的节点已经写好了，不用算最后一段了
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    for k = 0:1:n_seg-2 
        beq_wp(k+1, 1) = waypoints(k+2); %将节点位置结果输入
        coeff = getCoeffCons(ts(k+1)); %计算4个约束
        Aeq_wp(k+1, 1+k*8:8+k*8) = coeff(1, :);   %因为只有位置约束，所以只取第一行
        %由于第一段头和尾是共用系数的，所以可以看到依然是从一开始开始写入系数。没有问题。
    end
    
    % 连续性约束
    %#####################################################
    % 两段之间节点的p,v,a,j要一样，因此b恒为0
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    
    % 前一段的末尾=后一段的开头
    for k = 0:1:n_seg-2 
        Aeq_con(1+4*k:4+4*k,1+8*k:8+8*k) = getCoeffCons(ts(k+1));
        Aeq_con(1+4*k:4+4*k,1+8*(k+1):8+8*(k+1)) = -getCoeffCons(0);            
    end
    %#####################################################
    % 构造约束矩阵
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end