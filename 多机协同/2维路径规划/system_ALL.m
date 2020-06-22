clc
close all
clear all
global boundary setstart setfinal node danger_xi danger_yi  danger_ri weight;
%================================参数设置=============================================%
%node=4;%离终点最近的起始点到终点的节点个数（偶数）
L_FEN=42;%设置节点间的间隔距离
danger_xi=[300 100 400 70 250 600 500 200 600];%设置威胁区域坐标
danger_yi=[150 100 400 330 300 150 200 500 400];
danger_ri=[80  70  90  70 20 50 70 70 50];%设置威胁半径
weight=[1 0.9];%路径长度、夹角所占权重
boundary=[700 0];%设置环境上下限
setfinal=[200 250];%设置终点
setstart_ALL=...
[0 20;
 50 450;
 500 50;
 600 600];%设置起点,无人机自上而下依次编号为1、2、3.....   
Vmin=3;%设置无人机飞行速度范围（m/s）
Vmax=30;
Ways=1;%Ways=1时，实现同时起飞，同时到达目标；Ways=2时，实现指定顺序依次到达目标
Order=[1 3 4 2];%无人机到达顺序
interval=5;%无人机到达间隔时间（s）
%===================================威胁区域绘图==========================================%
circle(danger_xi,danger_yi,danger_ri);
%===========================将每条路径的节点距离大致均匀化=================================%
lineX=size(setstart_ALL,1);%UAV数量
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
 orig_node=node;%记录给定的初值node
%===================================主体函数===========================================% 
routes_distance=zeros(lineX,1);%初始每条路径长度为0
for r=1:lineX  
    B=0;
    if(r==ind)
    %=======================计算离终点直线距离最近的无人机路径==========================%    
    node=orig_node;
    setstart=setstart_ALL(r,:);
    C=runUAVABC4(r);%得到路径节点坐标
    for i=1:(size(C,1)-1)
       B=B+sqrt(((C(i+1,:)-C(i,:)).^2)*ones(2,1));
    end
    routes_distance(r)=B;
    %==================================================================================%
    %==============================计算其余无人机路径===================================%
    else
    node=fix(A(r)/objval); 
    if rem(node,2)~=0
        node=node+1;%确保node为偶数
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
distance_min%各起始点的直线距离
routes_distance%规划的航迹长度

if Ways==1%同时起飞，同时到达
ind=max(routes_distance)*Vmin-min(routes_distance)*Vmax;%临界条件
    if(ind<=0)
    time_plan=max(routes_distance)/Vmax;
    V_plan=routes_distance./time_plan;
    time_plan%团队预计到达的时间
    V_plan%规划的各无人机速度
    else
    fprintf('超出协同规划范围,任意两条长短路径的比值应小于无人机Vmax/Vmin');
    end
    
else%同时起飞，按指定次序先后到达
    
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
time_plan%团队预计到达的时间
V_plan%规划的各无人机速度
else
fprintf('超出协同规划范围');
end
end
