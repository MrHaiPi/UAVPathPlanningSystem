function Z_value=FUNenvironment(obj)
SETenvironment;
Z_value=interp2(X,Y,Z,obj(:,1),obj(:,2),'cubic');

