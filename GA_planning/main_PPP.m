clc;
clear;
close all;
warning off

%% 固定随机数种子
noRNG=3;
rng('default')
rng(noRNG)

%% 载入数据
data.map=xlsread('data.xlsx',1); %载入Excel中第一个sheet的地图
data.sizeMap=size(data.map);

%% 数据初始化操作
[p1,p2]=find(data.map==3); %找到起点
data.S=[p1,p2];
[p1,p2]=find(data.map==4); %找到终点
data.E=[p1,p2];

data.map(data.S(1),data.S(2))=0; %得到起点终点就回归free，易于算法运算
data.map(data.E(1),data.E(2))=0;

[p1,p2]=find(data.map==0); %得到所有可行走的点
data.node=[p1,p2]; %一方面得到了所有可行点坐标，另一方面相当于引索就是他们的一维num引索，没有p2 *col + p1直接，但是更容易操作

data.noS=find(p1==data.S(1) & p2==data.S(2)); % 求出起始点和终点的一维引索
data.noE=find(p1==data.E(1) & p2==data.E(2));

data.D=pdist2(data.node,data.node); % 求出所有点之间的距离，返回为n*n的链接矩阵
data.D1=pdist2(data.node,data.E);  % 求出所有点和终点之间的距离

% 这他妈是个负优化
% for i=1:length(data.node(:,1))
%     Point1=data.node(i,:);
%     Point2=data.E;
%     if ~ismember(Point1,Point2,'rows') %将每行视作一个整体，判断Point1不是终点就继续
%         if checkIfinObstacle(data.map,Point1,Point2) % 判断与目标点的连线是否碰到障碍物
%             data.D1(i)=data.D1(i)*2; % 碰到就*2，已经是一个比较大的惩罚了。
%         end
%     end
% end

[p1,p2]=find(data.D<=sqrt(2)); % 找到每个点的邻居，有8个方向
index=sub2ind([length(data.node(:,1)),length(data.node(:,1))],p1,p2); % sub2ind是将坐标变为一维引索
                                                                      % 这边是相当于将所有的边一维引索得到，方便下面赋值
data.net=[p1,p2,data.D(index)]; % 得到所有动作集即边和cost
dim=length(index); % 边的数量

%%
option.dim=dim;
lb=0.5;  %决策变量的下限lower
ub=1;    %决策变量的上限upper
option.lb=lb;
option.ub=ub;
if length(option.lb)==1
    option.lb=ones(1,option.dim)*option.lb;
    option.ub=ones(1,option.dim)*option.ub;
end

%option.fobj0=option.fobj;
option.showIter=0;
%% 算法参数设置 Parameters
% 基本参数
option.numAgent=10;        %种群个体数
option.maxIteration=150;    %最大迭代次数
%% 遗传算法
option.p1_GA=0.9;  %选择概率
option.p2_GA=0.1;  %变异概率
%% 粒子群
option.w_pso=1.5;  %惯性系数
option.c1_pso=1;   %个人期望系数
option.c2_pso=1;   %公共期望系数

str_legend=[{'GA'}];
selectedAlgorithm=[{@GA_change}];

option.dim=dim;
option.gap0=ceil(sqrt(option.maxIteration*2))+1;
lb=-ones(1,dim)*0;
ub=ones(1,dim)*1;
option.lb=lb;
option.ub=ub;
%%
%% 初始化种群个体
%% 使用算法求解

generate_RRT = true;
option.fobj=@aimFcn_PPP; % 初始化函数设置
x=ones(option.numAgent,option.dim); % [种群数*节点数]的集合
y=ones(option.numAgent,1); % [种群的适应度]
for i=1:option.numAgent  %初始化种群
    % 这边这个x就是本次遗传算法的重点。x里面每条边的权重就是选择这条边的重要依据。所以后面的交叉变异也是以这些边的权重作为依据的
    % 随机给所有点一个处于决策变量上下限之间的概率权重，用来选择往哪走
    x(i,:)=rand(size(option.lb)).*(option.ub-option.lb)+option.lb; 
    if generate_RRT == true
        x(i,:)=RRT_initial(x(i,:),option,data);
    end
    y(i)=option.fobj(x(i,:),option,data); % 获得适应度
end

for ii=1:length(selectedAlgorithm)
    rng(noRNG)
    tic
    [bestY(ii,:),bestX(ii,:),recording{ii}]=selectedAlgorithm{ii}(x,y,option,data);
end
%%
figure
hold on
for i=1:length(recording)
    plot(recording{i}.bestFit,'LineWidth',2)
end
for i=1:length(recording)
    plot(recording{i}.meanFit,'LineWidth',2)
end
legend(str_legend)
xlabel('t')
ylabel('目标')
%%
rng(1)
for ii=1:length(selectedAlgorithm)
    option.fobj=@aimFcn_PPP;
    str=str_legend{ii};
    [fit(ii),result(ii)]=option.fobj(bestX(ii,:),option,data);
    drawPc_PPP(result(ii),option,data,str)
end