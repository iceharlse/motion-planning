function CVRP_draw(S,result,data,str)

x_all = [];
y_all = [];
result_all = 0;
for j = 1:size(result,2)
    %% 平滑
    result_all = result_all + result(j).fit;
    newPath=data.node(result(j).path,1:2);
    x = newPath(:,1);% 初始化
    y = newPath(:,2);
    alpha = 0.6; %
    beta = 0.6; % 平滑程度
    xi = x;% 初始化
    yi = y;
    numK=8; %迭代次数
    for k=1:numK
        for i = 2:1:(length(x)-1)
            % 不优化起始点S 和 终点 E
            xi(i) = xi(i) + alpha*(x(i) - xi(i)) + beta*(xi(i-1) - 2*xi(i) + xi(i+1));
            yi(i) = yi(i) + alpha*(y(i) - yi(i)) + beta*(yi(i-1) - 2*yi(i) + yi(i+1));
            Point1=round([xi(i),yi(i)]);
            Point2=round([xi(i-1),yi(i-1)]);
            if data.map(round(xi(i)),round(yi(i)))==1
                xi(i)=x(i);
                yi(i)=y(i);
            end
            if checkIfinObstacle(data.map,Point1,Point2)
                xi(i)=x(i);
                yi(i)=y(i);
                xi(i-1)=x(i-1);
                yi(i-1)=y(i-1);
            end
        end
    end
    disp(size(xi));
    x_all = [x_all;xi];
    disp(size(x_all));
    y_all = [y_all;yi];
end
%%
figure
hold on
[p1,p2]=find(data.map==2);
for i=1:length(p1)
    rectangle('Position',[p1(i)-0.5,p2(i)-0.5,1,1],'Curvature',0.2,'FaceColor',[0.7451,0.7451,0.7451]);
end
[p1,p2]=find(data.map==1);
for i=1:length(p1)
    rectangle('Position',[p1(i)-0.5,p2(i)-0.5,1,1],'FaceColor','k');
end
% [p1,p2]=find(data.map==0);
% for i=1:length(p1)
%     rectangle('Position',[p1(i)-0.5,p2(i)-0.5,1,1],'FaceColor','w');
% end
plot(x_all,y_all,'LineWidth',2)
axis([0.5,0.5+data.sizeMap(1),0.5,0.5+data.sizeMap(2)])
plot(S(1,1),S(1,2),'ro','MarkerFaceColor','g');
for i = 2:size(S,1)
    plot(S(i,1),S(i,2),'ro','MarkerFaceColor','r');
end
% title([str,',fit：',num2str(result_all)]);
box on
xticks([])
yticks([])
toc
end