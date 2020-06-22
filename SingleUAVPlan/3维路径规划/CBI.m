X = [setstart;GlobalParams_s;setfinal];%ÿ��Ϊһ����
[m,n] = size(X); % CSJ: m Ϊ�������n Ϊ��ά��
%A:����ϵ��-----------------------------
A=zeros(m+2); % CSJ��Ϊ��ʹ����ͨ����ʼ��ĩ�˵����� 2 ��ϵ��
A(1,1)=1;A(1,2)=-2;A(1,3)=1;
A(m+2,m)=1;A(m+2,m+1)=-2;A(m+2,m+2)=1;
for i=2:(m+1)
    A(i,i-1)=1;
    A(i,i)=4;
    A(i,i+1)=1;
end
%e:�����ұ�.�õ��Ŀ��Ƶ���β���ֵ������ͬ.���������ĩ��������
e = zeros(m+2,n);
for i=2:m+1
    e(i,:)=6*X(i-1,:);
end
%�õ� d Boor ���Ƶ�
d=inv(A)*e;   % A * d =e
%����ͼ��
hold on
%ԭʼ����,��ɫ,��
%plot3(X(:,1),X(:,2),X(:,3),'r*');
%���ƶ����,��ɫ,��
%plot3(d(:,1),d(:,2),d(:,3),'b');
%��ֵB��������
uu=(0:0.01:1);
Num = length(uu);
x = zeros(Num,1);
y = zeros(Num,1);
z = zeros(Num,1);

for j=1:(m-1) % m�����ݵ�֮����m-1������
    for kk=1:Num
        x(kk)=d(j,1)*Nfun(0,uu(kk))+d(j+1,1)*Nfun(1,uu(kk))+d(j+2,1)*Nfun(2,uu(kk))+d(j+3,1)*Nfun(3,uu(kk)); 
        y(kk)=d(j,2)*Nfun(0,uu(kk))+d(j+1,2)*Nfun(1,uu(kk))+d(j+2,2)*Nfun(2,uu(kk))+d(j+3,2)*Nfun(3,uu(kk));
        z(kk)=d(j,3)*Nfun(0,uu(kk))+d(j+1,3)*Nfun(1,uu(kk))+d(j+2,3)*Nfun(2,uu(kk))+d(j+3,3)*Nfun(3,uu(kk));
    end
    plot3(x,y,z,'k','LineWidth',1);
    temPoint = [x y z];
     if j ~= m-1
         temPoint = temPoint(1:end-1,:); %�ų�ÿ������֮���ظ����ӵ�
     end
   
end
