function [bestY,bestX,recording]=GA(x,y,option,data)

    %% 遗传算法
    %% 初始化
    recording.bestFit=zeros(option.maxIteration+1,1);  %最佳适应度
    recording.meanFit=zeros(option.maxIteration+1,1);  %平均适应度
    %% 更新记录
    [y_g,position]=min(y);  % 取出最佳适应度和种群号，即cost最小的，如果有同样的取第一个就行
    x_g=x(position(1),:);   % 获得里面的x，即每个节点的系数
    y_p=y;
    x_p=x;
    recording.bestFit=y_g; % 记录一下
    recording.meanFit=mean(y_p);
    %% 开始更新
    for iter=1:option.maxIteration
        %disp(['GA,iter:',num2str(iter),',minFit:',num2str(y_g)])
        %% 竞标赛法选择个体
        for i=1:option.numAgent*2 % 抽去2N个父辈
            maxContestants=ceil(randi(option.numAgent)*option.p1_GA); % 本次竞标赛抽取的种群个数
            index=randperm(option.numAgent,maxContestants); % 原种群随机抽取父辈候选
            [~,position]=min(y_p(index)); % 选择适应度最好的
            parent(i)=index(position(1)); % 加入父辈
        end
        newX=x_p*0;
        newY=y_p*0;
        %% 交叉变异
        for i=1:option.numAgent
            newX(i,:)=x_p(parent(i),:);
            if rand<option.p1_GA %多点交叉
                tempR=rand(size(x_p(i,:))); % 生成所有边的随机数
                newX(i,tempR>0.5)=x_p(parent(i+option.numAgent),tempR>0.5); % 概率大于0.5的和另一个父辈交换边的概率权重
            end
            if rand<option.p2_GA %多点变异
                tempR=rand(size(x_p(i,:)));
                temp=rand(size(option.lb)).*(option.ub-option.lb)+option.lb; % 重新生成每个边的随机权重
                newX(i,tempR>0.5)=temp(tempR>0.5); % 变异
            end
        end
        %% 重新计算适应度值
        for i=1:option.numAgent
%             if i==1 && iter>20
%                 newX(i,rand(1,option.dim)>0.5)=0;
%             elseif i==2 && iter==20
%                 newX(i,rand(1,option.dim)>0.5)=1;
%             end
            newX(i,:)=checkX(newX(i,:),option,data); % 判断每个边权重不能高过一个上下限
            newY(i)=option.fobj(newX(i,:),option,data); % 求出适应度
            if newY(i)<y_p(i)
                y_p(i)=newY(i);
                x_p(i,:)=newX(i,:);
                if y_p(i)<y_g  % 替换最佳适应度
                    y_g=y_p(i);
                    x_g=x_p(i,:);
                end
            end
        end
        
        
        % 给最佳适应度的一点小奖励
        [~,find_fit] = min(y_p(:));
        find_fit = find_fit(1);
        x_p(find_fit,:) = x_p(find_fit,:) + 0.1;
        x_p(i,:)=checkX(x_p(i,:),option,data);
 
        % 给最差适应度一点专家策略
        if mod(iter, 10) == 0
            [~,find_fit] = max(y_p(:));
            find_fit = find_fit(1);
            x_p(find_fit,:) = RRT_initial(x_p(find_fit,:),option,data);
        end
        
        % 使用狼群策略给最差适应度加入专家策略
        if mod(iter, 5) == 0
            y_wolf = y_p;
            [~,min_index] = sort(y_wolf(:));
            min_index = min_index(1:3); %找到前三适宜度
            [~,find_fit] = max(y_p(:)); %找到最差适宜度
            find_fit = find_fit(1);
            x_p(find_fit,:) = 0.1 * (x_p(min_index(1),:) + x_p(min_index(2),:) + x_p(min_index(3),:)) / 3 + x_p(find_fit,:);
        end
        
        %% 更新记录
        recording.bestFit(1+iter)=y_g;
        recording.meanFit(1+iter)=mean(y_p);
%         disp(['已经完成',num2str(iter),'次迭代']);
    end
    bestY=y_g;
    bestX=x_g;
end