function DrawSpline(n, k, P, NodeVector)
for u = 0 : 0.005 : 1-0.005
    for i = 0 : 1 : n
        Nik(i+1, 1) = BaseFunction(i, k , u, NodeVector);
    end
    p_u = P * Nik;
    if u == 0
        tempx = p_u(1,1);
        tempy = p_u(2,1);
        line([tempx p_u(1,1)], [tempy p_u(2,1)],...
            'Marker','.','LineStyle','-', 'Color','k', 'LineWidth',1.1);
    else
        line([tempx p_u(1,1)], [tempy p_u(2,1)],...
            'Marker','.','LineStyle','-', 'Color','k', 'LineWidth',1.1);
        tempx = p_u(1,1);
        tempy = p_u(2,1);
    end
end