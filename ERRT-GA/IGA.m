function [bestY,bestX,recording]=IGA(x,y,option,data)
    %% 遗传算法
    %% 初始化
    recording.bestFit=zeros(option.maxIteration+1,1);
    recording.meanFit=zeros(option.maxIteration+1,1);
    %% 更新记录
    [y_g,position]=min(y);
    x_g=x(position(1),:);
    y_p=y;
    x_p=x;
    recording.bestFit=y_g;
    recording.meanFit=mean(y_p);
    % 混沌初始化
    dim=option.dim;
    x0=rand(1,dim); %系统初始状态
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
    %% 开始更新
    for iter=1:option.maxIteration
        %disp(['GA,iter:',num2str(iter),',minFit:',num2str(y_g)])
        %% 竞标赛法选择个体
        for i=1:option.numAgent*2
            maxContestants=ceil(randi(option.numAgent)*option.p1_GA);
            index=randperm(option.numAgent,maxContestants);
            [~,position]=min(y(index));
            parent(i)=index(position(1));
        end
        newX=x*0;
        newY=y*0;
        %% 交叉变异
        for i=1:option.numAgent
            newX(i,:)=x(parent(i),:);
            if rand<option.p1_GA %多点交叉
                tempR=rand(size(x(i,:)));
                newX(i,tempR>0.5)=x(parent(i+option.numAgent),tempR>0.5);
            end
            if rand<option.p2_GA %多点变异
                tempR=rand(size(x(i,:)));
                temp=rand(size(option.lb)).*(option.ub-option.lb)+option.lb;
                newX(i,tempR>0.5)=temp(tempR>0.5);
            end
        end
        %% 重新计算适应度值
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
        %% 更新记录
        recording.bestFit(1+iter)=y_g;
        recording.meanFit(1+iter)=mean(y_p);
    end
    bestY=y_g;
    bestX=x_g;
end