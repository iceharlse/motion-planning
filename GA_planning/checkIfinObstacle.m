%%判断A点和B点连线是否与障碍物相交，如果相交则返回isINObstacle=1
function isInObstacle = checkIfinObstacle(map,Point1,Point2) %创建函数isInObstacle
    isInObstacle = 0;
    x1 = Point1(1);
    y1 = Point1(2);
    x2 = Point2(1);
    y2 = Point2(2); 
    N=100; %分成100份，逐份判断是否碰到障碍物
    
    if x1~=x2
        dy = (x2-x1)/(N-1);
        tempX=x1:dy:x2;
        tempY= interp1([x1,x2],[y1,y2],tempX); %使用线性插值返回一维函数(线性函数)在特定查询点的插入值。
                                               % 第一个变量包含样本点，第二个变量是对应值。第三个变量包含查询点的坐标。
    elseif y1~=y2
        dy = (y2-y1)/(N-1);
        tempY=y1:dy:y2;
        tempX= interp1([y1,y2],[x1,x2],tempY);
    else
         tempY=y1;
         tempX=x1;
    end
    
    node=[tempX;tempY]';
    
    % 如果点能被0.5整除，那么这个点一定是在格子的边界点，不利于判断是左边格子还是右边格子，虽然感觉多此一举，但是也能用吧。
    tempNode=mod(node,0.5);
    if x1~=x2 && y1~=y2
        position=find(tempNode(:,1)==0 | tempNode(:,2)==0);
    elseif x1==x2
        position=find(tempNode(:,2)==0);
    elseif y1==y2
        position=find(tempNode(:,1)==0);
    end
    node(position,:)=[];
    node=round(node);
    node=unique(node,'rows'); % 得到这次连线所经过的所有格子
    
    for i=1:length(node(:,1))
        no1=node(i,1);
        no2=node(i,2);
        if (map(no1,no2)==1) || (map(no1,no2)==2)
            isInObstacle=1;
            continue;
        end
    end
end
            
            
            
