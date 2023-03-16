function [bestY,bestX,recording]=PSO(x,y,option,data)
    %% PSO算法
    %% 初始化
    recording.bestFit=zeros(option.maxIteration+1,1);
    recording.meanFit=zeros(option.maxIteration+1,1);
%     index_All=1;
%     All_X=zeros((option.maxIteration+1)*option.numAgent,option.dim);
%     All_Y=zeros((option.maxIteration+1)*option.numAgent,1);
%     All_X(index_All:index_All+option.numAgent-1,:)=x;
%     All_Y(index_All:index_All+option.numAgent-1,:)=y;
%     index_All=index_All+option.numAgent;
    v=randn(size(x));
    %% 更新记录
    [y_g,position]=min(y);
    x_g=x(position(1),:);
    y_p=y;
    x_p=x;
    recording.bestFit=y_g;
    recording.meanFit=mean(y_p);
    w_pso=option.w_pso;
    c1_pso=option.c1_pso;
    c2_pso=option.c2_pso;
    LB=option.lb;
    UB=option.ub;
    fobj=option.fobj;
    numAgent=option.numAgent;
    dim=option.dim;
    %% 开始更新
    for iter=1:option.maxIteration
        %disp(['PSO,iter:',num2str(iter),',minFit:',num2str(y_g)])
        %% 更新
        r1=rand(numAgent,dim);
        r2=rand(numAgent,dim);
        for i=1:numAgent
            v(i,:)=w_pso*v(i,:)+c1_pso*r1(i,:).*(x_g-x(i,:))+c2_pso*r2(i,:).*(x_p(i,:)-x(i,:));
            x(i,:)=x(i,:)+v(i,:);
            x(i,x(i,:)<LB)=LB(x(i,:)<LB);
            x(i,x(i,:)>UB)=UB(x(i,:)>UB);
            y(i)=fobj(x(i,:),option,data);
            if y(i)<y_p(i)
                y_p(i)=y(i);
                x_p(i,:)=x(i,:);
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