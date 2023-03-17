clc;
clear;
close all;
warning off

%% ��������
data.map=xlsread('data_change.xlsx',1); %����Excel�е�һ��sheet�ĵ�ͼ
data.sizeMap=size(data.map);

%% ���ݳ�ʼ������
S = ones(4,2);  % ��㼯��
E = ones(4,2);  % �յ㼯��
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

data.map(S(:,1),S(:,2))=0; %�õ�����յ�ͻع�free�������㷨����
data.map(E(:,1),E(:,2))=0;

for i = 1:size(S,1)
    [data_fin,result(i)] = PATH_find(S(i,:),E(i,:),data);
end
str = 'GA';
CVRP_draw(result,data_fin,str);

