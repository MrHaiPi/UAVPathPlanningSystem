function y = Nfun(i,t)
    %²ÎÊýi,
    y=0;
    for j=0:(3-i)
        y=y+(-1)^j*nchoosek(4,j)*(t+3-i-j)^3;
    end
    y=y/6;
end