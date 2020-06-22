function UAVobjval=UAV2(obj1,obj2,obj3)
line=size(obj1,1);
UAVobjval_S=ones(line,1);
UAVobjval_angle=ones(line,1);
B=ones(line,1);
C=ones(line,1);
global  danger_xi danger_yi danger_ri weight;
%=============================����һ���ڵ����һ����ľ���=========================================%
for r=1:line
    B(r)=sqrt(((obj1(r,:)-obj2).^2)*ones(2,1));
    C(r)=sqrt(((obj1(r,:)-obj3).^2)*ones(2,1));
    UAVobjval_S(r)=C(r)+B(r);
end;
%=============================����һ���ڵ���¸��ڵ�ļн�==============================%
for r=1:line
    UAVobjval_angle(r)=(obj2-obj1(r,:))*((obj3-obj1(r,:))')/(B(r)*C(r));
end
ind=find(UAVobjval_angle>=0);
UAVobjval_angle(ind)=50;
ind=find(UAVobjval_angle<0);
UAVobjval_angle(ind)=10*(2+UAVobjval_angle(ind));
 %=============================��в����================================================%
 danger=[danger_xi' danger_yi']; 
 d=size(danger,1);%��в����ĸ���
 Ldanger=ones(line,d);
 for i=1:d
 for r=1:line
 Ldanger(r,i)=sqrt(((obj1(r,:)-danger(i,:)).^2)*ones(2,1)); 
 end; 
 ind=find(Ldanger(:,i)<=danger_ri(1,i)+10);
 Ldanger(ind,i)=0;
 ind=find(Ldanger(:,i)>danger_ri(1,i)+10);
 Ldanger(ind,i)=1;  
  end;
 Ldanger_sum=sum(Ldanger,2);
 ind=find(Ldanger_sum<d);
 Ldanger_sum(ind)=0;
 ind=find(Ldanger_sum==d);
 Ldanger_sum(ind)=1;
 %============================�ܵĴ��ۺ���=============================================%
 UAVobjval(:,1)=(UAVobjval_S.^weight(1)).*...
                (UAVobjval_angle.^weight(2)).*...
                 Ldanger_sum;
                 