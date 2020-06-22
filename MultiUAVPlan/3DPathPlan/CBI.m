X = [setstart;GlobalParams_s;setfinal];%每行为一个点
[m,n] = size(X); % CSJ: m 为点个数，n 为点维数
%A:方程系数-----------------------------
A=zeros(m+2); % CSJ：为了使曲线通过起始和末端点增加 2 行系数
A(1,1)=1;A(1,2)=-2;A(1,3)=1;
A(m+2,m)=1;A(m+2,m+1)=-2;A(m+2,m+2)=1;
for i=2:(m+1)
    A(i,i-1)=1;
    A(i,i)=4;
    A(i,i+1)=1;
end
%e:方程右边.得到的控制点首尾与插值数据相同.且与控制首末连线相切
e = zeros(m+2,n);
for i=2:m+1
    e(i,:)=6*X(i-1,:);
end
%得到 d Boor 控制点
d=inv(A)*e;   % A * d =e
%画出图形
hold on
%原始数据,红色,点
%plot3(X(:,1),X(:,2),X(:,3),'r*');
%控制多边形,蓝色,线
%plot3(d(:,1),d(:,2),d(:,3),'b');
%插值B样条曲线
uu=(0:0.01:1);
Num = length(uu);
x = zeros(Num,1);
y = zeros(Num,1);
z = zeros(Num,1);

for j=1:(m-1) % m个数据点之间有m-1段曲线
    for kk=1:Num
        x(kk)=d(j,1)*Nfun(0,uu(kk))+d(j+1,1)*Nfun(1,uu(kk))+d(j+2,1)*Nfun(2,uu(kk))+d(j+3,1)*Nfun(3,uu(kk)); 
        y(kk)=d(j,2)*Nfun(0,uu(kk))+d(j+1,2)*Nfun(1,uu(kk))+d(j+2,2)*Nfun(2,uu(kk))+d(j+3,2)*Nfun(3,uu(kk));
        z(kk)=d(j,3)*Nfun(0,uu(kk))+d(j+1,3)*Nfun(1,uu(kk))+d(j+2,3)*Nfun(2,uu(kk))+d(j+3,3)*Nfun(3,uu(kk));
    end
    plot3(x,y,z,'k','LineWidth',1);
    temPoint = [x y z];
     if j ~= m-1
         temPoint = temPoint(1:end-1,:); %排除每段曲线之间重复连接点
     end
   
end
