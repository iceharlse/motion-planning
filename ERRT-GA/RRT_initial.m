function new_x = RRT_initial(x,option,data)
    % 使用RRT+A*算法初始化种群
    S=data.S; % 起点的坐标
    E=data.E; % 终点的坐标
    Thr= 10;  % 设置目标点阈值
    Delta= 3;  % 设置扩展步长
    XL = data.sizeMap(1); % 地图的长
    YL = data.sizeMap(2); % 地图的高
    new_x =x;
    flag1=zeros(length(data.node(:,1)),1); % 点的标记点，标记这个点是否探寻过
    
    %% 建树初始化
    T.v(1).x = S(1);         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
    T.v(1).y = S(2); 
    T.v(1).xPrev = S(1);     % 起始节点的父节点仍然是其本身
    T.v(1).yPrev = S(2);
    T.v(1).index = data.noS;  %   每个节点的一维引索
    T.v(1).indPrev = 0;     % 父节点在树中的引索
    flag1(data.noS) = 1;
    
    count=1;
    get_dist = @(x1,y1,x2,y2) sqrt((x1-x2)^2 + (y1-y2)^2); %欧几里得距离
    
    %% 开始构建树
    for iter = 1:3000
        %在地图中随机采样一个点x_rand
        x_rand = [rand()*XL,rand()*YL];
        
        %遍历树，从树中找到最近邻近点x_near 
        dist = arrayfun((@(x) get_dist(x_rand(1),x_rand(2),x.x,x.y)),T.v);
        [~,idx] = min(dist);
        x_near = [T.v(idx).x,T.v(idx).y];
        
        %扩展得到x_new节点
        theta = atan2(x_rand(2)-x_near(2),x_rand(1)-x_near(1));
        x_new=[round(x_near(1) + Delta * cos(theta)),round(x_near(2) + Delta * sin(theta))];
        find_index = find(data.node(:,1)==x_new(1) & data.node(:,2)==x_new(2));
        if flag1(find_index) == 1
            continue;
        end
            
        if (x_new(1) <= 0) || (x_new(1)>XL) || (x_new(2) <= 0) || (x_new(2)>YL) %判断是否越界
            continue;
        end

        %检查节点是否是collision-free
        if checkIfinObstacle(data.map,x_new,x_near) % 判断与目标点的连线是否碰到障碍物
            continue;
        end
        
        % 将x_new插入树T 
        count=count+1;
        T.v(count).x = x_new(1);
        T.v(count).y = x_new(2);
        T.v(count).xPrev = x_near(1);
        T.v(count).yPrev = x_near(2);
        T.v(count).index = find_index;
        T.v(count).indPrev = idx;
        flag1(T.v(count).index) = 1;
 
        %检查是否到达目标点附近 
        if(get_dist(x_new(1),x_new(2),E(1),E(2))<Thr)
            break;
        end
    end
    
    %% 改变相关x
    % 得到这个种群的路径
    index = T.v(end).indPrev;
    path_i = 0;
    path = [];
    while T.v(index).index ~= data.noS
        path_i = path_i + 1;
        path(path_i) = T.v(index).index;
        index = T.v(index).indPrev;
    end
    
    if ~isempty(path)
        % 改变x
        for i = 1:size(path)
            position = find(data.net(:,1)==path(i)); %得到所有以路径点为起点的边的引索
            new_x(position) = new_x(position) + 0.1; % 将所有的边的权重加上0.1
            new_x = checkX(new_x,option,data); %判断是否超过决策变量上限
        end
    end
    
        
        
        
    
    
    
end