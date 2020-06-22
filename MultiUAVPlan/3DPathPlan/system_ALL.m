clc
close all
clear all
%================================��������=============================================%
global boundary setstart setfinal node delta_H danger_xi danger_yi danger_zi danger_ri weight;
%node=12;%���յ��������ʼ�㵽�յ�Ľڵ����(ż��)
L_FEN=26;%���ýڵ��ļ������
delta_H=[20 40];%���(���)�߶ȡ����й�����͸߶�
danger_xi=[207.1 393.9];%������в��������                
danger_yi=[333.3 414.1];
danger_zi=[389.9 349.2];
danger_ri=[0 0];%������в�뾶
weight=[1 0.01 0.3];%·�����ȡ������߶ȡ��н���ռȨ��
boundary=[500 0];%���û���������
setfinal=[291.8 219.8 492.4];%�����յ�
setstart_ALL=[176.8 237.4 254.5;%������㣬����Ϊ���˻�1��2��3...
             161.6 363.6 429;
             454.5 146.5 228.2;
             393.9 308.1 234.3];          
Vmin=3;%�������˻������ٶȷ�Χ��m/s��
Vmax=30;
Ways=2;%Ways=1ʱ��ʵ��ͬʱ��ɣ�ͬʱ����Ŀ�ꣻWays=2ʱ��ʵ��ָ��˳�����ε���Ŀ��
Order=[1 4 2 3];%���˻�����˳��
interval=9;%���˻�������ʱ�䣨s��
%====================================================================================%       
lineX=size(setstart_ALL,1);
%======================================���λ�ͼ=======================================%
SETenvironment;
surf(X,Y,Z);
box on;
rotate3d on;
xi=linspace(0,500,100);
yi=linspace(0,500,100);
[XI,YI]=meshgrid(xi,yi);
ZI=interp2(X,Y,Z,XI,YI,'cubic');
surf(XI,YI,ZI) %�⻬����+�ȸ���
hold on;
%=====================================��в�����ͼ====================================%
[x,y,z]=sphere(40);
for k=1:size(danger_xi,2)
 surf(danger_ri(k)*x+danger_xi(k),danger_ri(k)*y+danger_yi(k),danger_ri(k)*z+danger_zi(k));
hold on;
end
%===========================��ÿ��·���Ľڵ������¾��Ȼ�=================================%
lineX=size(setstart_ALL,1);%UAV����
A=zeros(lineX,1);
for r=1:lineX
A(r)=sqrt(((setstart_ALL(r,:)-setfinal).^2)*ones(3,1));
end
node=floor(min(A)/L_FEN);
if mod(node,2)==1
    node=node+1;
end
 ind=find(A==min(A));
 objval=A(ind(end))/node;
 orig_node=node;%��¼�����ĳ�ֵnode
%===================================���庯��===========================================% 
routes_distance=zeros(lineX,1);%��ʼÿ��·������Ϊ0
for r=1:lineX  
    B=0;
    if(r==ind)
    %=======================�������յ�ֱ�߾�����������˻�·��==========================%    
    node=orig_node;
    setstart=setstart_ALL(r,:);
    C=runUAVABC4(r);%�õ�·���ڵ�����
    for i=1:(size(C,1)-1)
       B=B+sqrt(((C(i+1,:)-C(i,:)).^2)*ones(3,1));
    end
    routes_distance(r)=B;
    %==================================================================================%
    %==============================�����������˻�·��===================================%
    else
    node=fix(A(r)/objval); 
    if rem(node,2)~=0
        node=node+1;%ȷ��nodeΪż��
    end
    setstart=setstart_ALL(r,:);
    C=runUAVABC4(r);
    for i=1:(size(C,1)-1)
      B=B+sqrt(((C(i+1,:)-C(i,:)).^2)*ones(3,1));
    end
    routes_distance(r)=B;
    end
    %===================================================================================%
   
end
%======================================================================================%
distance_min=A;
time_plan=zeros(lineX,1);
distance_min%����ʼ���ֱ�߾���
routes_distance%�滮�ĺ�������

if Ways==1%ͬʱ��ɣ�ͬʱ����
ind=max(routes_distance)*Vmin-min(routes_distance)*Vmax;%�ٽ�����
    if(ind<=0)
    time_plan=max(routes_distance)/Vmax;
    V_plan=routes_distance./time_plan;
    time_plan%�Ŷ�Ԥ�Ƶ����ʱ��
    V_plan%�滮�ĸ����˻��ٶ�
    else
    fprintf('����Эͬ�滮��Χ,������������·���ı�ֵӦС�����˻�Vmax/Vmin');
    end
    
else%ͬʱ��ɣ���ָ�������Ⱥ󵽴�
    
    time_max=routes_distance/Vmin;
    time_min=routes_distance/Vmax;  
    for k=1:lineX
         if k==1
    time_plan(Order(k))=routes_distance(Order(k))/Vmax;
         else
    time_plan(Order(k))=time_plan(Order(k-1))+interval;
         end
    end
ind=find(time_plan<time_min);
 while isempty(ind)==0
ind=ind(1);
time_plan(ind)=routes_distance(ind)/Vmax;
for a=1:lineX-1 
time_plan(a)=time_plan(ind)-(ind-a)*interval;
end
ind=find(time_plan<time_min);
 end

V_plan=routes_distance./time_plan;
ind=find(V_plan<Vmin);
if isempty(ind)~=0
time_plan%�Ŷ�Ԥ�Ƶ����ʱ��
V_plan%�滮�ĸ����˻��ٶ�
else
fprintf('����Эͬ�滮��Χ');
end
end

