function UAVobjval=UAV2(obj1,obj2,obj3)
line=size(obj1,1);
UAVobjval_F=ones(line,1);
UAVobjval_angle=ones(line,1);
B=ones(line,1);
C=ones(line,1);
global  delta_H danger_xi danger_yi danger_zi danger_ri weight;
%=============================到上一个节点和下一个点的距离=========================================%
for r=1:line
    B(r)=sqrt(((obj1(r,:)-obj2).^2)*ones(3,1));
    C(r)=sqrt(((obj1(r,:)-obj3).^2)*ones(3,1));
    UAVobjval_F(r)=C(r)+B(r);
end;
%=============================到上一个节点和下一个点的夹角=========================================%
for r=1:line
    UAVobjval_angle(r)=(obj2-obj1(r,:))*((obj3-obj1(r,:))')/(2*B(r)*C(r));
end
ind=find(UAVobjval_angle>=0);
UAVobjval_angle(ind)=50;
ind=find(UAVobjval_angle<0);
UAVobjval_angle(ind)=10*(2+UAVobjval_angle(ind));
%=============================与地面的距离=============================================%
 H=500;
 h=5;
 UAVobjval_S=obj1(:,3);
 ind=find(obj1(:,3)>=H);
 UAVobjval_S(ind)=obj1(ind,3)*1;
 ind=find(obj1(:,3)<=h);
 UAVobjval_S(ind)=0;
 ind=find(obj1(:,3)<=(FUNenvironment(obj1)+delta_H(2)));
 UAVobjval_S(ind)=0;
 %=============================威胁区域================================================%
 danger=[danger_xi' danger_yi' danger_zi']; 
 d=size(danger,1);%威胁区域的个数
 Ldanger=ones(line,d);
 for i=1:d
 for r=1:line
 Ldanger(r,i)=sqrt(((obj1(r,:)-danger(i,:)).^2)*ones(3,1)); 
 end; 
 ind=find(Ldanger(:,i)<=danger_ri(1,i)+20);
 Ldanger(ind,i)=0;
 ind=find(Ldanger(:,i)>danger_ri(1,i)+20);
 Ldanger(ind,i)=1;  
  end;
 Ldanger_sum=sum(Ldanger,2);
 ind=find(Ldanger_sum<d);
 Ldanger_sum(ind)=0;
 ind=find(Ldanger_sum==d);
 Ldanger_sum(ind)=1;
 %============================总的代价函数=============================================%
 UAVobjval(:,1)=(UAVobjval_F.^weight(1)).*...
                (UAVobjval_S.^weight(2)).*...
                (UAVobjval_angle.^weight(3)).*...
                 Ldanger_sum;