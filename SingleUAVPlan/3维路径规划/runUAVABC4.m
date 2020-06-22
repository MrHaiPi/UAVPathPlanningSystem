clc
close all
clear all
%================================参数设置=============================================%
global boundary setstart setfinal node delta_H danger_xi danger_yi danger_zi danger_ri weight setstart_2 setfinal_2 ;
%node=12;%离终点最近的起始点到终点的节点个数(偶数)
delta_H=[20 40];%起飞(落地)高度、飞行过程最低高度
danger_xi=[207.1 393.9];%设置威胁区域坐标                
danger_yi=[333.3 414.1];
danger_zi=[389.9 349.2];
danger_ri=[30 50];%设置威胁半径
weight=[1 0.05 0.6];%路径长度、离地面高度、夹角所占权重
boundary=[500 0];%设置环境上下限
%setfinal_ALL=[444 459 483];
setstart=[15.15 30.3 295.9];
setfinal=[449.5 459.6 422]; 
%==============确定节点数=================%
L_ALL=sqrt(((setfinal-setstart).^2)*ones(3,1));
L_FEN=17;%设置节点间的间隔距离
node=floor(L_ALL/L_FEN);
if mod(node,2)==1
    node=node+1;
end;
%====================================================================================%       
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
% for i=1:2
% 
%       h(i)=surf(X,Y,Z);
%       alpha(h(i),0.2*i)
%       Z=Z-100;
%     
% end
%=====================================威胁区域绘图====================================%
[x,y,z]=sphere(40);
for k=1:size(danger_xi,2)
 surf(danger_ri(k)*x+danger_xi(k),danger_ri(k)*y+danger_yi(k),danger_ri(k)*z+danger_zi(k));
 hold on;
end
%====================================================================================%

maxCycle=20;%算法迭代次数
%/* Problem specific variables*/
D=3; 
NP=40; 
FoodNumber=NP/2; 
limit=NP*D;
ub=ones(1,D)*(boundary(1)-10); 
lb=ones(1,D)*(boundary(2)+10);
setfinal_2=setfinal+[0 0 delta_H(1)];
setstart_2=setstart+[0 0 delta_H(1)];
runtime=node+1;
objfun='UAV2'; 

GlobalMins=zeros(1,runtime);
GlobalParams_s=[setstart_2;zeros(node+1,3);setfinal_2];

for r=1:2*runtime
Range = repmat((ub-lb),[FoodNumber 1]);
Lower = repmat(lb, [FoodNumber 1]);
Foods = rand(FoodNumber,D) .* Range + Lower;

%====================================================================================%
if r<=node/2%Step1:将奇数位置的节点双向规划（基于X坐标均分）
   X_ave=setstart_2(1):((setfinal_2(1)-setstart_2(1))/(node/2+1)):setfinal_2(1);
   X_ave=X_ave(2:end-1);
   if rem(r,2)~=0
     Foods(:,1)=X_ave((r+1)/2);
     ObjVal=feval(objfun,Foods,GlobalParams_s(r,:),GlobalParams_s(node+4-r,:));   
   else
     Foods(:,1)=X_ave(node/2+1-r/2);    
     ObjVal=feval(objfun,Foods,GlobalParams_s(r+1,:),GlobalParams_s(node+5-r,:));   
   end
else
    if r>node/2&&r<=runtime%Step2:将偶数位置的节点单向插缝规划（基于Step1所产生节点间的Y坐标中值）
   feckParams_s=[0;0;GlobalParams_s(1:(end-2),2)];
   feckParams_s=(GlobalParams_s(:,2)-feckParams_s)/2;
   feckParams_s=[feckParams_s(3:end);0;0];
   Y_ave=GlobalParams_s(:,2)+feckParams_s;
   Foods(:,2)=Y_ave(2*(r-node/2)-1);
   ObjVal=feval(objfun,Foods,GlobalParams_s(2*(r-node/2)-1,:),GlobalParams_s(2*(r-node/2)+1,:));   
    elseif r>runtime&&r<=runtime+node/2%Step3:重新将奇数位置的节点单向插缝规划（基于Step2所产生节点间的X坐标中值）
   feckParams_s=[0;GlobalParams_s(4:end,1);0;0];     
   feckParams_s=(feckParams_s-GlobalParams_s(:,1))/2;  
   X_ave=GlobalParams_s(:,1)+feckParams_s;
   Foods(:,1)=X_ave(2*(r-runtime));
   ObjVal=feval(objfun,Foods,GlobalParams_s(2*(r-runtime),:),GlobalParams_s(2*(r-runtime)+2,:));   
    else%Step4:重新将偶数位置的节点单向插缝规划（基于Step3所产生节点间的Y坐标中值）
   feckParams_s=[0;0;GlobalParams_s(1:(end-2),2)];
   feckParams_s=(GlobalParams_s(:,2)-feckParams_s)/2;
   feckParams_s=[feckParams_s(3:end);0;0];
   Y_ave=GlobalParams_s(:,2)+feckParams_s;
   Foods(:,2)=Y_ave(2*(r-node/2-runtime)-1);
   ObjVal=feval(objfun,Foods,GlobalParams_s(2*(r-node/2-runtime)-1,:),GlobalParams_s(2*(r-node/2-runtime)+1,:));            
    end    
end
%====================================================================================%

Fitness=calculateFitness(ObjVal);
%reset trial counters
trial=zeros(1,FoodNumber);

%/*The best food source is memorized*/
BestInd=find(ObjVal==min(ObjVal(find(ObjVal>0))));
BestInd=BestInd(end);
GlobalMin=ObjVal(BestInd);
GlobalParams=Foods(BestInd,:);

iter=1;
while (iter <= maxCycle),
%雇佣蜂先采一批蜜源回去供观察蜂观察
%%%%%%%%% EMPLOYED BEE PHASE %%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:(FoodNumber)
        Param2Change=fix(rand*D)+1;
        neighbour=fix(rand*FoodNumber)+1;      
            while(neighbour==i)%即如果随机生成的整数与i相等则再生成一个，直到不相等为止
                neighbour=fix(rand*FoodNumber)+1;
            end;
        
       sol=Foods(i,:);
       %与第个蜜源相对应的采蜜蜂依据如下公式寻找新的蜜源
       sol(Param2Change)=Foods(i,Param2Change)+(Foods(i,Param2Change)-Foods(neighbour,Param2Change))*(rand-0.5)*2;%(rand-0.5)*2  即产生一个[-1:1]的随机数
     
        ind=find(sol<(lb));%如果超出边界，则将值转变为边界值
        sol(ind)=lb(ind);
        ind=find(sol>(ub));
        sol(ind)=ub(ind);
        
        %evaluate new solution
        if r<=node/2
            if rem(r,2)~=0
          ObjValSol=feval(objfun,sol,GlobalParams_s(r,:),GlobalParams_s(node+4-r,:)); 
            else
          ObjValSol=feval(objfun,sol,GlobalParams_s(r+1,:),GlobalParams_s(node+5-r,:)); 
            end
      else
        if r>node/2&&r<=runtime
        ObjValSol=feval(objfun,sol,GlobalParams_s(2*(r-node/2)-1,:),GlobalParams_s(2*(r-node/2)+1,:));   
        elseif r>runtime&&r<=runtime+node/2
        ObjValSol=feval(objfun,sol,GlobalParams_s(2*(r-runtime),:),GlobalParams_s(2*(r-runtime)+2,:));   
        else
        ObjValSol=feval(objfun,sol,GlobalParams_s(2*(r-node/2-runtime)-1,:),GlobalParams_s(2*(r-node/2-runtime)+1,:));   
        end
       end
       
        FitnessSol=calculateFitness(ObjValSol);
        
       % /*a greedy selection is applied between the current solution i and its mutant 
       if (FitnessSol>Fitness(i)) %/*If the mutant solution is better than the current solution i, replace the solution with the mutant and reset the trial counter of solution i*/
            Foods(i,:)=sol;
            Fitness(i)=FitnessSol;
            ObjVal(i)=ObjValSol;
            trial(i)=0;
        else
            trial(i)=trial(i)+1; %/*if the solution i can not be improved, increase its trial counter*/
       end;
     end;

%%%%%%%%%%%%%%%%%%%%%%%% CalculateProbabilities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
prob=(0.9.*Fitness./max(Fitness))+0.1;%"."运算为矩阵运算
%prob=Fitness./sum(Fitness);
%%%%%%%%%%%%%%%%%%%%%%%% ONLOOKER BEE PHASE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

i=1;
t=0;
while(t<FoodNumber)
    if(rand<prob(i))
        t=t+1;
        Param2Change=fix(rand*D)+1;%随机生成[1：D]上的一个整数
        neighbour=fix(rand*(FoodNumber))+1;
        %/*Randomly selected solution must be different from the solution i*/        
            while(neighbour==i)
                neighbour=fix(rand*(FoodNumber))+1;
            end;
        
       sol=Foods(i,:);
       sol(Param2Change)=Foods(i,Param2Change)+(Foods(i,Param2Change)-Foods(neighbour,Param2Change))*(rand-0.5)*2;
       %  /*if generated parameter value is out of boundaries, it is shifted onto the boundaries*/
        ind=find(sol<(lb));%如果超出边界，则将值转变为边界值
        sol(ind)=lb(ind);
        ind=find(sol>(ub));
        sol(ind)=ub(ind);
        
        %evaluate new solution
        if r<=node/2
            if rem(r,2)~=0
          ObjValSol=feval(objfun,sol,GlobalParams_s(r,:),GlobalParams_s(node+4-r,:)); 
            else
          ObjValSol=feval(objfun,sol,GlobalParams_s(r+1,:),GlobalParams_s(node+5-r,:)); 
            end
      else
        if r>node/2&&r<=runtime
        ObjValSol=feval(objfun,sol,GlobalParams_s(2*(r-node/2)-1,:),GlobalParams_s(2*(r-node/2)+1,:));   
        elseif r>runtime&&r<=runtime+node/2
        ObjValSol=feval(objfun,sol,GlobalParams_s(2*(r-runtime),:),GlobalParams_s(2*(r-runtime)+2,:));   
        else
        ObjValSol=feval(objfun,sol,GlobalParams_s(2*(r-node/2-runtime)-1,:),GlobalParams_s(2*(r-node/2-runtime)+1,:));   
        end
       end
        
        FitnessSol=calculateFitness(ObjValSol);
        
       % /*a greedy selection is applied between the current solution i and its mutant*/
       if (FitnessSol>Fitness(i)) %/*If the mutant solution is better than the current solution i, replace the solution with the mutant and reset the trial counter of solution i*/
            Foods(i,:)=sol;
            Fitness(i)=FitnessSol;
            ObjVal(i)=ObjValSol;
            trial(i)=0;
        else
            trial(i)=trial(i)+1; %/*if the solution i can not be improved, increase its trial counter*/
       end;
    end;
    
    i=i+1;
     if (i==(FoodNumber)+1)
         i=1;
     end;   
end; 


%/*The best food source is memorized*/
         ind=find(ObjVal==min(ObjVal(find(ObjVal>0))));
         ind=ind(end);
         if (ObjVal(ind)<GlobalMin)
         GlobalMin=ObjVal(ind);
         GlobalParams=Foods(ind,:);%储存最小函数值对应参数
         end;
         
         
%%%%%%%%%%%% SCOUT BEE PHASE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%侦察蜂用于防止算法陷入局部最优
ind=find(trial==max(trial));
ind=ind(end);
if (trial(ind)>limit)
    Bas(ind)=0;
    
    sol=(ub-lb).*rand(1,D)+lb;%随机生成各参数值（即食物）
    
     if r<=node/2
            if rem(r,2)~=0
          ObjValSol=feval(objfun,sol,GlobalParams_s(r,:),GlobalParams_s(node+4-r,:)); 
            else
          ObjValSol=feval(objfun,sol,GlobalParams_s(r+1,:),GlobalParams_s(node+5-r,:)); 
            end
      else
        if r>node/2&&r<=runtime
        ObjValSol=feval(objfun,sol,GlobalParams_s(2*(r-node/2)-1,:),GlobalParams_s(2*(r-node/2)+1,:));   
        elseif r>runtime&&r<=runtime+node/2
        ObjValSol=feval(objfun,sol,GlobalParams_s(2*(r-runtime),:),GlobalParams_s(2*(r-runtime)+2,:));   
        else
        ObjValSol=feval(objfun,sol,GlobalParams_s(2*(r-node/2-runtime)-1,:),GlobalParams_s(2*(r-node/2-runtime)+1,:));   
        end
       end
     
    FitnessSol=calculateFitness(ObjValSol);%计算适宜度
    Foods(ind,:)=sol;%即超过‘limit’限制的食物被随机生成的食物代替
    Fitness(ind)=FitnessSol;
    ObjVal(ind)=ObjValSol;
end;

fprintf('代数=%d GlobalMin=%g\n',iter,GlobalMin);
iter=iter+1;
end ;% End of ABC

GlobalMins(r)=GlobalMin;

if r<=node/2
    if rem(r,2)~=0
    GlobalParams_s(r+2,:)=GlobalParams;
    else
    GlobalParams_s(node+3-r,:)=GlobalParams;        
    end
else
  if r>node/2&&r<=runtime
  GlobalParams_s(2*(r-node/2),:)=GlobalParams;
  elseif r>runtime&&r<=runtime+node/2
  GlobalParams_s(2*(r-runtime)+1,:)=GlobalParams;  
  else
  GlobalParams_s(2*(r-node/2-runtime),:)=GlobalParams;    
  end
end


end; %end of runs
%=====================================路径绘图========================================%
scatter3([setstart(1) setfinal(1)],[setstart(2) setfinal(2)],[setstart(3) setfinal(3)],'*','r');
% line([setfinal(1) GlobalParams_s(node+2,1) ],[setfinal(2) GlobalParams_s(node+2,2)],[setfinal(3) GlobalParams_s(node+2,3)],'Color','k','LineWidth',1);
% line([setstart(1) GlobalParams_s(1,1) ],[setstart(2) GlobalParams_s(1,2)],[setstart(3) GlobalParams_s(1,3)],'Color','k','LineWidth',1);
% scatter3(GlobalParams_s(:,1),GlobalParams_s(:,2),GlobalParams_s(:,3),'*','r');
% line(GlobalParams_s(:,1),GlobalParams_s(:,2),GlobalParams_s(:,3),'Color','k','LineWidth',1); 
%====================================================================================%
%hold on;
save all
CBI;%路径平滑绘图
%================================计算路径长度=========================================%
C=GlobalParams_s;
A=0;
for i=1:(size(C,1)-1)
A=A+sqrt(((C(i+1,:)-C(i,:)).^2)*ones(3,1));
end
A%输出规划航迹长度
min=sqrt(((setstart-setfinal).^2)*ones(3,1))%输出起始两点间的直线距离



