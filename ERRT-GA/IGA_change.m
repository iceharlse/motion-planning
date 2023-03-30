function [bestY,bestX,recording]=IGA(x,y,option,data)
    %% �Ŵ��㷨
    %% ��ʼ��
    recording.bestFit=zeros(option.maxIteration+1,1);
    recording.meanFit=zeros(option.maxIteration+1,1);
    %% ���¼�¼
    [y_g,position]=min(y);
    x_g=x(position(1),:);
    y_p=y;
    x_p=x;
    recording.bestFit=y_g;
    recording.meanFit=mean(y_p);
    % �����ʼ��
    dim=option.dim;
    x0=rand(1,dim); %ϵͳ��ʼ״̬
    phi=0.5;
    ub=option.ub;
    lb=option.lb;
    for i=1:option.numAgent
        newX(i,:)=(ub-lb).*x0+lb;
        p1=find(x0<phi);
        p2=find(x0>=phi);
        x0(p1)=x0(p1)/phi;
        x0(p2)=(1-x0(p2))./(1-phi);
        newY(i,:)=option.fobj(newX(i,:),option,data);
        if newY(i,:)<y(i)
            x(i,:)=newX(i,:);
            y(i)=newY(i,:);
        end
    end
    %% ��ʼ����
    for iter=1:option.maxIteration
        %disp(['GA,iter:',num2str(iter),',minFit:',num2str(y_g)])
        %% ��������ѡ�����
        for i=1:option.numAgent*2
            maxContestants=ceil(randi(option.numAgent)*option.p1_GA);
            index=randperm(option.numAgent,maxContestants);
            [~,position]=min(y_p(index));
            parent(i)=index(position(1));
        end
        newX=x_p*0;
        newY=y_p*0;
        %% �������
        for i=1:option.numAgent
            newX(i,:)=x_p(parent(i),:);
            if rand<option.p1_GA %��㽻��
                tempR=rand(size(x_p(i,:)));
                newX(i,tempR>0.5)=x_p(parent(i+option.numAgent),tempR>0.5);
            end
            if rand<option.p2_GA %������
                tempR=rand(size(x_p(i,:)));
                temp=rand(size(option.lb)).*(option.ub-option.lb)+option.lb;
                newX(i,tempR>0.5)=temp(tempR>0.5);
            end
        end
        %% ���¼�����Ӧ��ֵ
        for i=1:option.numAgent
            if i==1 
                newX(i,rand(1,option.dim)>0.5)=0;
            elseif i==2 
                newX(i,rand(1,option.dim)>0.5)=1;
            end
            newX(i,:)=checkX(newX(i,:),option,data);
            newY(i)=option.fobj(newX(i,:),option,data);
            if newY(i)<y_p(i)
                y_p(i)=newY(i);
                x_p(i,:)=newX(i,:);
                if y_p(i)<y_g
                    y_g=y_p(i);
                    x_g=x_p(i,:);
                end
            end
        end
        
                % �������Ӧ�ȵ�һ��С����
        [~,find_fit] = min(y_p(:));
        find_fit = find_fit(1);
        x_p(find_fit,:) = x_p(find_fit,:) + 0.1;
        x_p(i,:)=checkX(x_p(i,:),option,data);
 
        % �������Ӧ��һ��ר�Ҳ���
        if mod(iter, 5) == 0
            [~,find_fit] = max(y_p(:));
            find_fit = find_fit(1);
            x_p(find_fit,:) = RRT_initial(x_p(find_fit,:),option,data);
        end
        
        %% ���¼�¼
        recording.bestFit(1+iter)=y_g;
        recording.meanFit(1+iter)=mean(y_p);
    end
    bestY=y_g;
    bestX=x_g;
end