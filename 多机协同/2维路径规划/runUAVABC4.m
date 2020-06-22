function POS=runUAVABC4(obj)
global boundary setstart setfinal node setstart_2 setfinal_2 ;
maxCycle=10;%算法迭代次数
%/* Problem specific variables*/

D=2; 
NP=40; 
FoodNumber=NP/2; 
limit=NP*D;
ub=ones(1,D)*(boundary(1)-10); 
lb=ones(1,D)*(boundary(2)+10);
setfinal_2=setfinal;
setstart_2=setstart;
runtime=node+1;
objfun='UAV2';

GlobalMins=zeros(1,runtime);
GlobalParams_s=[setstart_2;zeros(node+1,2);setfinal_2];

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
            while(neighbour==i)
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
            trial(i)=trial(i)+1;
       end;
     end;

%%%%%%%%%%%%%%%%%%%%%%%% CalculateProbabilities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
prob=(0.9.*Fitness./max(Fitness))+0.1;
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
            end
        
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
    
    sol=(ub-lb).*rand(1,D)+lb;
  
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
    Foods(ind,:)=sol;
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
%line([setfinal(1) GlobalParams_s(node+2,1) ],[setfinal(2) GlobalParams_s(node+2,2)],'Color','k','LineWidth',1);
%line([setstart(1) GlobalParams_s(1,1) ],[setstart(2) GlobalParams_s(1,2)],'Color','k','LineWidth',1);
%plot(GlobalParams_s(2:(end-1),1),GlobalParams_s(2:(end-1),2),'*','Color','r');
plot(setstart(1) ,setstart(2) ,'*','Color','b');
plot( setfinal(1),setfinal(2),'hexagram','Color','g');
text(setstart(1)+10 ,setstart(2)+10,num2str(obj));
%line(GlobalParams_s(:,1),GlobalParams_s(:,2),'Color','k','LineWidth',1); 
%====================================================================================%
B;%B样条平滑
hold on;
save all
POS=GlobalParams_s;


