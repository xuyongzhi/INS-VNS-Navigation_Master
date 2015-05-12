function Rk = R_cov2(Pc,Pp,Q0,Rbb,Tbb)
% 函数功能
% 计算量测信息——相邻时刻小车本体坐标系相对旋转矩阵
% 和本体系下的平移矢量的测量误差方差阵
% 2013.3.25
N = size(Pp,2); % feature number
J = zeros(3*N,7);
% JJ = zeros(N,7);
q0 = Q0(1);
q1 = Q0(2);
q2 = Q0(3);
q3 = Q0(4);
Rbc = [1, 0, 0;
       0, 0,-1;
       0, 1, 0];
Rcb = Rbc';
for i = 1:N
    res = Pc(:,i) - (Rbc * Rbb * Rcb * Pp(:,i) + Rbc * Tbb);
    pp1 = Pp(1,i);
    pp2 = Pp(2,i);
    pp3 = Pp(3,i);
    J(3*i-2,1) = -2*pp2*q2 - 2*pp3*q3 - 2*pp1*q0;
    J(3*i-2,2) = 2*pp2*q3 - 2*pp3*q2 - 2*pp1*q1;
    J(3*i-2,3) = -2*pp2*q0 - 2*pp3*q1 + 2*pp1*q2;
    J(3*i-2,4) = 2*pp2*q1 - 2*pp3*q0 + 2*pp1*q3;
    J(3*i-2,5) = -1;
    J(3*i-2,6) = 0;
    J(3*i-2,7) = 0;
    
    J(3*i-1,1) = 2*pp1*q2 - 2*pp3*q1 - 2*pp2*q0;
    J(3*i-1,2) = 2*pp1*q3 - 2*pp3*q0 + 2*pp2*q1;
    J(3*i-1,3) = 2*pp1*q0 + 2*pp3*q3 + 2*pp2*q2;
    J(3*i-1,4) = 2*pp1*q1 + 2*pp3*q2 - 2*pp2*q3;
    J(3*i-1,5) = 0;
    J(3*i-1,6) = 0;
    J(3*i-1,7) = 1;
    
    J(3*i,1) = 2*pp1*q3 + 2*pp2*q1 - 2*pp3*q0;
    J(3*i,2) = -2*pp1*q2 + 2*pp2*q0 + 2*pp3*q1;
    J(3*i,3) = -2*pp1*q1 + 2*pp2*q3 - 2*pp3*q2;
    J(3*i,4) = 2*pp1*q0 + 2*pp2*q2 + 2*pp3*q3;
    J(3*i,5) = 0;
    J(3*i,6) = -1;
    J(3*i,7) = 0;
    J(3*i-2:3*i,:) = 2 * norm(res) * J(3*i-2:3*i,:);
%     JJ(i,:) = 2 * res' * J(3*i-2:3*i,:);
end
Rk = inv(J'*J);
% Rk = inv(JJ'*JJ);
% Rk = J'*J;
