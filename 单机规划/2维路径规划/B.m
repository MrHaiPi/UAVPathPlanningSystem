P =GlobalParams_s'; 
n = size(P,2)-1; k = 2;
NodeVector = U_quasi_uniform(n, k); % 准均匀B样条的节点矢量
DrawSpline(n, k, P, NodeVector);

