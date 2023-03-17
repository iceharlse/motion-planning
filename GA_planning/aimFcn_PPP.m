function [fit,result]=aimFcn_1(x,option,data)
    % 函数用来初始化种群
    flag=zeros(length(data.net(:,1)),1); % 边的标记点，记录这条边有没有走过
    flag1=zeros(length(data.node(:,1)),1); % 点的标记点，标记这个点是否探寻过
    S=data.noS;
    E=data.noE;
    path=[S]; %路径加入起点
    flag1(S)=1;  % 路径点设为探寻过
    D=0; % 总开销
    v=[0,0];
    S1=data.S; %起点的坐标
    
    % 查找路径
    while S~=E  %只要没搜寻到终点
        position=find(data.net(:,1)==S & flag==0);  % 得到所有以S为起点的可以探查的边，其实就是周围8个动作
        if isempty(position) % 如果没有可用边
            S=path(end-1); % 返回去上一个节点，重新找
            path(end)=[];
            continue;
        end
        
        nextNode=data.net(position,2);  % 下一个节点的集合的一维引索
        
        for i=1:length(nextNode)
            xy=data.node(nextNode(i),1:2); % 得到节点的节点坐标
            v1=xy-S1; % 运动朝向，也可以称为速度
            if sum(v)==0 % 如果现在的速度为0，dir也设为0，因为0速度时，朝着哪个方向都可以，没有什么转变方向的控制损失
                dir(i)=0;
            else
                dir(i)=norm(v-v1); % 否则设为两个速度之间的距离，大概是方向改变的损失值。肯定朝着自己原来方向的跑最好
            end
        end
        
        D2=data.D1(nextNode);  % 后继点与终点的距离
        D1=data.net(position,3); % 后继点与本节点的距离
        pri=x(position)';  % 获得每条边选择的种群自身权重
        pri(flag1(nextNode)==1)=inf; % 后继点如果已经探寻过了，给他一个不可达
        if isinf(min(pri))  % 没有后继点，退回去重选
            S=path(end-1);
            path(end)=[];
            continue;
        end
        temp1=D1+D2; % 其实借鉴了A*算法h+g，D1就是应该花的cost，D2就是与终点的直线距离
        temp1=mapminmax(temp1',0,1);  % 将后继点的cost值归一化
        dir=mapminmax(dir,0,1);  % 将所有的转向损失也归一化
        temp=temp1'+pri+0.1 * dir;  % 然后将这些归一化的权重概率相加，得到选取每个方向上的权重
        [~,no]=min(temp);  % 取出花费最少的方向
        no=no(1);  % 如果有多个就选第一个
        v1=data.node(nextNode(no),1:2)-S1; % 得到方向
        dir=norm(v-v1); % 得到变向花费
        S1=data.node(nextNode(no),1:2);  % 新节点的坐标
        S=data.net(position(no),2); % 新节点的一维引索
        D=D+data.net(position(no),3); % 加上开销
        flag(position(no))=1; % 这条边设为探查过
        flag1(S)=1; % 这个点设为探查过
        path=[path;S]; % 路径加入这个点
        v=v1; % 得到最新方向
    end
    fit=D; % 得到总路程，作为适应度
    if nargout>1 % 输出
        result.fit=fit;
        result.path=path;
    end
end