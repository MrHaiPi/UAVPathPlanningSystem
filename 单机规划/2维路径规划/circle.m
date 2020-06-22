function circle(obj1,obj2,obj3)
a=size(obj1,2);
for r=1:a
alpha=0:pi/50:2*pi;
R=obj3(r); 
x=R*cos(alpha)+obj1(r); 
y=R*sin(alpha)+obj2(r); 
plot(x,y,'-','color','k') 
axis equal 
hold on;
end