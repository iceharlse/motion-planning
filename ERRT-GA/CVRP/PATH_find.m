function [data,result] = PATH_find(S,E,data0)

    %% �̶����������
    noRNG=1;
    rng('default')
    rng(noRNG)

    %% ���ݳ�ʼ������
    data = data0;
    data.S = S;
    data.E = E;
    
    [p1,p2]=find(data.map==0); %�õ����п����ߵĵ�
    data.node=[p1,p2]; %һ����õ������п��е����꣬��һ�����൱�������������ǵ�һάnum������û��p2 *col + p1ֱ�ӣ����Ǹ����ײ���

    data.noS=find(p1==data.S(1) & p2==data.S(2)); % �����ʼ����յ��һά����
    data.noE=find(p1==data.E(1) & p2==data.E(2));

    data.D=pdist2(data.node,data.node); % ������е�֮��ľ��룬����Ϊn*n�����Ӿ���
    data.D1=pdist2(data.node,data.E);  % ������е���յ�֮��ľ���

    % �������Ǹ����Ż�
%     for i=1:length(data.node(:,1))
%         Point1=data.node(i,:);
%         Point2=data.E;
%         if ~ismember(Point1,Point2,'rows') %��ÿ������һ�����壬�ж�Point1�����յ�ͼ���
%             if checkIfinObstacle(data.map,Point1,Point2) % �ж���Ŀ���������Ƿ������ϰ���
%                 data.D1(i)=data.D1(i)*2; % ������*2���Ѿ���һ���Ƚϴ�ĳͷ��ˡ�
%             end
%         end
%     end

    [p1,p2]=find(data.D<=sqrt(2)); % �ҵ�ÿ������ھӣ���8������
    index=sub2ind([length(data.node(:,1)),length(data.node(:,1))],p1,p2); % sub2ind�ǽ������Ϊһά����
                                                                          % ������൱�ڽ����еı�һά�����õ����������渳ֵ
    data.net=[p1,p2,data.D(index)]; % �õ����ж��������ߺ�cost
    dim=length(index); % �ߵ�����

    %%
    option.dim=dim;
    lb=0.5;  %���߱���������lower
    ub=1;    %���߱���������upper
    option.lb=lb;
    option.ub=ub;
    if length(option.lb)==1
        option.lb=ones(1,option.dim)*option.lb;
        option.ub=ones(1,option.dim)*option.ub;
    end

    %option.fobj0=option.fobj;
    option.showIter=0;
    %% �㷨�������� Parameters
    % ��������
    option.numAgent=20;        %��Ⱥ������
    option.maxIteration=1000;    %����������
    %% �Ŵ��㷨
    option.p1_GA=0.9;  %ѡ�����
    option.p2_GA=0.1;  %�������
    %% ����Ⱥ
    option.w_pso=1.5;  %����ϵ��
    option.c1_pso=1;   %��������ϵ��
    option.c2_pso=1;   %��������ϵ��

    str_legend=[{'GA'}];
    selectedAlgorithm=[{@GA_change}];

    option.dim=dim;
    option.gap0=ceil(sqrt(option.maxIteration*2))+1;
    lb=-ones(1,dim)*0;
    ub=ones(1,dim)*1;
    option.lb=lb;
    option.ub=ub;
    %%
    %% ��ʼ����Ⱥ����
    %% ʹ���㷨���

    generate_RRT = true;  % �Ƿ�ʹ��RRT��ʼ����Ⱥ
    option.fobj=@aimFcn_PPP; % ��ʼ����������
    x=ones(option.numAgent,option.dim); % [��Ⱥ��*�ڵ���]�ļ���
    y=ones(option.numAgent,1); % [��Ⱥ����Ӧ��]
    for i=1:option.numAgent  %��ʼ����Ⱥ
        % ������x���Ǳ����Ŵ��㷨���ص㡣x����ÿ���ߵ�Ȩ�ؾ���ѡ�������ߵ���Ҫ���ݡ����Ժ���Ľ������Ҳ������Щ�ߵ�Ȩ����Ϊ���ݵ�
        % ��������е�һ�����ھ��߱���������֮��ĸ���Ȩ�أ�����ѡ��������
        x(i,:)=rand(size(option.lb)).*(option.ub-option.lb)+option.lb; 
        if generate_RRT == true
            x(i,:)=RRT_initial(x(i,:),option,data);
        end
        y(i)=option.fobj(x(i,:),option,data); % �����Ӧ��
    end

    for ii=1:length(selectedAlgorithm)
        rng(noRNG)
        tic
        [bestY(ii,:),bestX(ii,:),recording{ii}]=selectedAlgorithm{ii}(x,y,option,data);
    end

    %%
    rng(1)
    option.fobj=@aimFcn_PPP;
    [~,result]=option.fobj(bestX(ii,:),option,data);
end