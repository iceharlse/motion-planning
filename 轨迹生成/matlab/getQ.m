function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = [];
        %#####################################################
        % 计算第k段的矩阵Q_k 
        for i = 4:n_order
            for j = 4:n_order
                % 因为多项式第一项是0次方，所以存储的时候需要+1
                % factorial为阶乘函数
                Q_k(i+1,j+1) = factorial(i)/factorial(i-4)*factorial(j)/factorial(j-4)/(i+j-n_order)*ts(k)^(i+j-n_order);
            end
        end
        % blkdiag是在右下角增加矩阵块组合的函数
        Q = blkdiag(Q, Q_k);
    end
end