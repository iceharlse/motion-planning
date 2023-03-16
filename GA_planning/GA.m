function [bestY,bestX,recording]=GA(x,y,option,data)

    %% �Ŵ��㷨
    %% ��ʼ��
    recording.bestFit=zeros(option.maxIteration+1,1);  %�����Ӧ��
    recording.meanFit=zeros(option.maxIteration+1,1);  %ƽ����Ӧ��
    %% ���¼�¼
    [y_g,position]=min(y);  % ȡ�������Ӧ�Ⱥ���Ⱥ�ţ���cost��С�ģ������ͬ����ȡ��һ������
    x_g=x(position(1),:);   % ��������x����ÿ���ڵ��ϵ��
    y_p=y;
    x_p=x;
    recording.bestFit=y_g; % ��¼һ��
    recording.meanFit=mean(y_p);
    %% ��ʼ����
    for iter=1:option.maxIteration
        %disp(['GA,iter:',num2str(iter),',minFit:',num2str(y_g)])
        %% ��������ѡ�����
        for i=1:option.numAgent*2 % ��ȥ2N������
            maxContestants=ceil(randi(option.numAgent)*option.p1_GA); % ���ξ�������ȡ����Ⱥ����
            index=randperm(option.numAgent,maxContestants); % ԭ��Ⱥ�����ȡ������ѡ
            [~,position]=min(y(index)); % ѡ����Ӧ����õ�
            parent(i)=index(position(1)); % ���븸��
        end
        newX=x*0;
        newY=y*0;
        %% �������
        for i=1:option.numAgent
            newX(i,:)=x(parent(i),:);
            if rand<option.p1_GA %��㽻��
                tempR=rand(size(x(i,:))); % �������бߵ������
                newX(i,tempR>0.5)=x(parent(i+option.numAgent),tempR>0.5); % ���ʴ���0.5�ĺ���һ�����������ߵĸ���Ȩ��
            end
            if rand<option.p2_GA %������
                tempR=rand(size(x(i,:)));
                temp=rand(size(option.lb)).*(option.ub-option.lb)+option.lb; % ��������ÿ���ߵ����Ȩ��
                newX(i,tempR>0.5)=temp(tempR>0.5); % ����
            end
        end
        %% ���¼�����Ӧ��ֵ
        for i=1:option.numAgent
%             if i==1 && iter>20
%                 newX(i,rand(1,option.dim)>0.5)=0;
%             elseif i==2 && iter==20
%                 newX(i,rand(1,option.dim)>0.5)=1;
%             end
            newX(i,:)=checkX(newX(i,:),option,data); % �ж�ÿ����Ȩ�ز��ܸ߹�һ��������
            newY(i)=option.fobj(newX(i,:),option,data); % �����Ӧ��
            if newY(i)<y_p(i)
                y_p(i)=newY(i);
                x_p(i,:)=newX(i,:);
                if y_p(i)<y_g  % �滻�����Ӧ��
                    y_g=y_p(i);
                    x_g=x_p(i,:);
                end
            end
        end
        %% ���¼�¼
        recording.bestFit(1+iter)=y_g;
        recording.meanFit(1+iter)=mean(y_p);
    end
    bestY=y_g;
    bestX=x_g;
end