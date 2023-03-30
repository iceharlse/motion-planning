
data.map=xlsread('data.xlsx',1); %载入Excel中第一个sheet的地图
data.sizeMap=size(data.map);
figure
hold on
[p1,p2]=find(data.map==1);
for i=1:length(p1)
    rectangle('Position',[(p1(i)-0.5),(p2(i)-0.5),1,1],'FaceColor','k');
end
[p1,p2]=find(data.map==2);
for i=1:length(p1)
    rectangle('Position',[p1(i)-0.5,p2(i)-0.5,1,1],'Curvature',0.2,'FaceColor',[0.7451,0.7451,0.7451]);
end
% [p1,p2]=find(data.map==3);
% plot(p1,p2,'ro','MarkerFaceColor','g')
% [p1,p2]=find(data.map==4);
% for i=1:length(p1)
%     plot(p1*2.5,p2*2.5,'ro','MarkerFaceColor','r')
% end
% legend('start','task')
box on
xticks([])
yticks([])