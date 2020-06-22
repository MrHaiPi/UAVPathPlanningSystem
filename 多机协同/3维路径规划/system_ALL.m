clc
close all
clear all
%================================参数设置=============================================%
global boundary setstart setfinal node delta_H danger_xi danger_yi danger_zi danger_ri weight;
%node=12;%离终点最近的起始点到终点的节点个数(偶数)
L_FEN=26;%设置节点间的间隔距离
delta_H=[20 40];%起飞(落地)高度、飞行过程最低高度
danger_xi=[207.1 393.9];%设置威胁区域坐标                
danger_yi=[333.3 414.1];
danger_zi=[389.9 349.2];
danger_ri=[0 0];%设置威胁半径
weight=[1 0.01 0.3];%路径长度、离地面高度、夹角所占权重
boundary=[500 0];%设置环境上下限
setfinal=[291.8 219.8 492.4];%设置终点
setstart_ALL=[176.8 237.4 254.5;%设置起点，依次为无人机1、2、3...
             161.6 363.6 429;
             454.5 146.5 228.2;
             393.9 308.1 234.3];          
Vmin=3;%设置无人机飞行速度范围（m/s）
Vmax=30;
Ways=2;%Ways=1时，实现同时起飞，同时到达目标；Ways=2时，实现指定顺序依次到达目标
Order=[1 4 2 3];%无人机到达顺序
interval=9;%无人机到达间隔时间（s）
%====================================================================================%       
lineX=size(setstart_ALL,1);
%======================================地形绘图=======================================%
SETenvironment;
surf(X,Y,Z);
box on;
rotate3d on;
xi=linspace(0,500,100);
yi=linspace(0,500,100);
[XI,YI]=meshgrid(xi,yi);
ZI=interp2(X,Y,Z,XI,YI,'cubic');
surf(XI,YI,ZI) %光滑曲面+等高线
hold on;
%=====================================威胁区域绘图====================================%
[x,y,z]=sphere(40);
for k=1:size(danger_xi,2)
 surf(danger_ri(k)*x+danger_xi(k),danger_ri(k)*y+danger_yi(k),danger_ri(k)*z+danger_zi(k));
hold on;
end
%===========================将每条路径的节点距离大致均匀化=================================%
lineX=size(setstart_ALL,1);%UAV数量
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
       B=B+sqrt(((C(i+1,:)-C(i,:)).^2)*ones(3,1));
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
      B=B+sqrt(((C(i+1,:)-C(i,:)).^2)*ones(3,1));
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

