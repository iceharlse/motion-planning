%%�ж�A���B�������Ƿ����ϰ����ཻ������ཻ�򷵻�isINObstacle=1
function isInObstacle = checkIfinObstacle(map,Point1,Point2) %��������isInObstacle
    isInObstacle = 0;
    x1 = Point1(1);
    y1 = Point1(2);
    x2 = Point2(1);
    y2 = Point2(2); 
    N=100; %�ֳ�100�ݣ�����ж��Ƿ������ϰ���
    
    if x1~=x2
        dy = (x2-x1)/(N-1);
        tempX=x1:dy:x2;
        tempY= interp1([x1,x2],[y1,y2],tempX); %ʹ�����Բ�ֵ����һά����(���Ժ���)���ض���ѯ��Ĳ���ֵ��
                                               % ��һ���������������㣬�ڶ��������Ƕ�Ӧֵ������������������ѯ������ꡣ
    elseif y1~=y2
        dy = (y2-y1)/(N-1);
        tempY=y1:dy:y2;
        tempX= interp1([y1,y2],[x1,x2],tempY);
    else
         tempY=y1;
         tempX=x1;
    end
    
    node=[tempX;tempY]';
    
    % ������ܱ�0.5��������ô�����һ�����ڸ��ӵı߽�㣬�������ж�����߸��ӻ����ұ߸��ӣ���Ȼ�о����һ�٣�����Ҳ���ðɡ�
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
    node=unique(node,'rows'); % �õ�������������������и���
    
    for i=1:length(node(:,1))
        no1=node(i,1);
        no2=node(i,2);
        if (map(no1,no2)==1) || (map(no1,no2)==2)
            isInObstacle=1;
            continue;
        end
    end
end
            
            
            
