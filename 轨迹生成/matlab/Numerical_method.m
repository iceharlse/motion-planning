clc;clear;close all;
path = ginput() * 100.0; % ginput在坐标中任取点，使用enter结束并返回坐标，默认[0*100,1*100]的范围

n_order       = 7; % 多项式的阶数，和状态数有关
n_seg         = size(path,1)-1;% 分段数
n_poly_perseg = (n_order+1); % 每段的系数个数

ts = zeros(n_seg, 1); % time_seg设定每段之间的给定时间

% 根据距离长度的比例计算每两个点之间的时间分配
dist     = zeros(n_seg, 1); %每段的直线距离
dist_sum = 0;
T        = 25;  %设定飞行总时间25
t_sum    = 0;

for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum+dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i)/dist_sum*T;
    t_sum = t_sum+ts(i);
end
ts(n_seg) = T - t_sum;

% 或者简单地设置每一段的时间分配为1
%for i = 1:n_seg
%    ts(i) = 1.0;
%end

% 分别对x和y方向求取对应的多项式系数
poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);


% 用于显示轨迹
X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % 得到每一段的x和y方向的系数
    Pxi = poly_coef_x((n_order+1)*(i)+1:(n_order+1)*(i)+n_order+1); 
    Pyi = poly_coef_y((n_order+1)*(i)+1:(n_order+1)*(i)+n_order+1);

    %带入计算，算出每个时间间间隔坐标
    for t = 0:tstep:ts(i+1)
        %flip反转系数
        X_n(k)  = polyval(flip(Pxi), t);
        Y_n(k)  = polyval(flip(Pyi), t);
        k = k + 1;
    end
end
 
plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);
hold on
scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));

% Minisnap求解器
function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    % 起点约束
    start_cond = [waypoints(1), 0, 0, 0];  
    % 终点约束
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % 计算Q矩阵 p'Q p
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % 计算对应的约束矩阵A_beq
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    
    % 求解多项式系数
    f = zeros(size(Q,1),1);%作为二次规划的一次项系数，本问题为0，不用考虑
    %quadprog用来求解二次规划，quadprog(Q,f,A,b,Aeq,beq)分别为最小化二次项，一次项，不等式约束A,b，等式约束Aeq,Beq
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end