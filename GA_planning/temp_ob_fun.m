function temp_ob=temp_ob_fun(now_point,E_coo,obstacle_p,do,ob_r)

a1=atan2(E_coo(2)-now_point(2),E_coo(1)-now_point(1));
ob_n=size(obstacle_p,1);
a2=zeros(1,ob_n);
for i=1:ob_n
    temp=atan2(obstacle_p(i,2)-now_point(2),obstacle_p(i,1)-now_point(1));
    temp=temp-a1;
    if temp<=-pi
        temp=temp+2*pi;
    elseif temp>pi
        temp=temp-2*pi;
    end
    a2(i)=temp;
end
n1=length(find(a2>0));
n2=length(find(a2<0));
if n1>n2
    a1=a1+pi/2;
else
    a1=a1-pi/2;
end
temp_ob(1)=now_point(1)+(do+ob_r)*cos(a1);
temp_ob(2)=now_point(2)+(do+ob_r)*sin(a1);

    


