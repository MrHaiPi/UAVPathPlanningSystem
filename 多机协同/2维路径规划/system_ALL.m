clc
close all
clear all
global boundary setstart setfinal node danger_xi danger_yi  danger_ri weight;
%================================��������=============================================%
%node=4;%���յ��������ʼ�㵽�յ�Ľڵ������ż����
L_FEN=42;%���ýڵ��ļ������
danger_xi=[300 100 400 70 250 600 500 200 600];%������в��������
danger_yi=[150 100 400 330 300 150 200 500 400];
danger_ri=[80  70  90  70 20 50 70 70 50];%������в�뾶
weight=[1 0.9];%·�����ȡ��н���ռȨ��
boundary=[700 0];%���û���������
setfinal=[200 250];%�����յ�
setstart_ALL=...
[0 20;
 50 450;
 500 50;
 600 600];%�������,���˻����϶������α��Ϊ1��2��3.....   
Vmin=3;%�������˻������ٶȷ�Χ��m/s��
Vmax=30;
Ways=1;%Ways=1ʱ��ʵ��ͬʱ��ɣ�ͬʱ����Ŀ�ꣻWays=2ʱ��ʵ��ָ��˳�����ε���Ŀ��
Order=[1 3 4 2];%���˻�����˳��
interval=5;%���˻�������ʱ�䣨s��
%===================================��в�����ͼ==========================================%
circle(danger_xi,danger_yi,danger_ri);
%===========================��ÿ��·���Ľڵ������¾��Ȼ�=================================%
lineX=size(setstart_ALL,1);%UAV����
A=zeros(lineX,1);
for r=1:lineX
A(r)=sqrt(((setstart_ALL(r,:)-setfinal).^2)*ones(2,1));
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
       B=B+sqrt(((C(i+1,:)-C(i,:)).^2)*ones(2,1));
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
      B=B+sqrt(((C(i+1,:)-C(i,:)).^2)*ones(2,1));
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
