clc;
clear;
close all;
warning off

%% 载入数据
data.map=xlsread('data_change.xlsx',1); %载入Excel中第一个sheet的地图
data.sizeMap=size(data.map);

%% 数据初始化操作
S = ones(4,2);  % 起点集合
E = ones(4,2);  % 终点集合
[p1,p2]=find(data.map==3);
S(1,:)=[p1,p2];
E(4,:)=[p1,p2];
[p1,p2]=find(data.map==4); 
S(2,:)=[p1,p2];
E(1,:)=[p1,p2];
[p1,p2]=find(data.map==5);
S(3,:)=[p1,p2];
E(2,:)=[p1,p2];
[p1,p2]=find(data.map==6);
S(4,:)=[p1,p2];
E(3,:)=[p1,p2];

data.map(S(:,1),S(:,2))=0; %得到起点终点就回归free，易于算法运算
data.map(E(:,1),E(:,2))=0;

for i = 1:size(S,1)
    [data_fin,result(i)] = PATH_find(S(i,:),E(i,:),data);
end
str = 'GA';
CVRP_draw(result,data_fin,str);

