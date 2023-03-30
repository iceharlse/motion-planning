clc;clear;close all;
% path = ginput() * 100.0;
% path = [3,3;98,85;3,3];
% [X_n,Y_n] = TSP(path);
% plot(X_n, Y_n ,'Color',[0 1.0 0],'LineWidth',2);
% hold on
% scatter(path(1:size(path,1),1),path(1:size(path,1),2));

path = [3,3;2,24;5,42;1,65;3,82;19,97;50,93;29,89;20,70;14,47;3,3];
[X_n,Y_n] = TSP(path);
plot(path(:,1),path(:,2),'--','Color',[0.5 0.5 0],'LineWidth',2);
hold on
plot(X_n, Y_n ,'Color',[1.0 0 0],'LineWidth',2);
hold on
scatter(path(1:size(path,1),1),path(1:size(path,1),2));

% path = [3,3;29,32;62,70;75,83;80,94;51,71;41,69;31,62;27,49;3,3];
% [X_n,Y_n] = TSP(path);
% plot(X_n, Y_n ,'Color',[0 0 1.0],'LineWidth',2);
% hold on
% scatter(path(1:size(path,1),1),path(1:size(path,1),2));
% 
% path = [3,3;23,15;48,30;85,60;91,75;98,52;84,39;13,7;3,3];
% % path = [3,3;23,15;48,30;85,60;91,75;98,52;84,39;3,3];
% [X_n,Y_n] = TSP(path);
% plot(X_n, Y_n ,'Color',[0 1.0 1.0],'LineWidth',2);
% hold on
% scatter(path(1:size(path,1),1),path(1:size(path,1),2));
% 
% path = [3,3;40,5;49,8;65,10;83,3;96,24;84,25;3,3]
% % path = [3,3;13,7;40,5;49,8;65,10;83,3;96,24;84,25;3,3]
% [X_n,Y_n] = TSP(path);
% plot(X_n, Y_n ,'Color',[0.9 1.0 0.7],'LineWidth',2);
% hold on
% scatter(path(1:size(path,1),1),path(1:size(path,1),2));

% legend('UAV-1','','UAV-2','','UAV-3','','UAV-4','','UAV-5')
legend('Straight path','smooth trajectory')
box on
xticks([])
yticks([])

