function Rk = R_covEuler1(Pc,Pp,R,pos,T)
% 函数功能
% 计算量测信息——相邻时刻小车本体坐标系相对旋转矩阵
% 和本体系下的平移矢量的测量误差方差阵，姿态用欧拉角表示
% 2013.5.7
N = size(Pp,2); % feature number
J = zeros(3*N,6);
% JJ = zeros(N,6);
pitch = pos(1);
roll = pos(2);
head = pos(3);
Rbc = [1, 0, 0;
       0, 0,-1;
       0, 1, 0];
Rcb = Rbc';
for i = 1:N
    pp1 = Pp(1,i);
    pp2 = Pp(2,i);
    pp3 = Pp(3,i);
    J(3*i-2,1) = pp1*sin(head)*cos(pitch)*sin(roll) - pp3*cos(head)*cos(pitch)*sin(roll) + pp2*sin(pitch)*sin(roll);
    J(3*i-2,2) = - pp1*(-cos(head)*sin(roll) - sin(head)*sin(pitch)*cos(roll)) - pp3*(-sin(roll)*sin(head) + cos(head)*sin(pitch)*cos(roll)) - pp2*cos(pitch)*cos(roll);
    J(3*i-2,3) = - pp1*(-sin(head)*cos(roll) - cos(head)*sin(pitch)*sin(roll)) - pp3*(cos(roll)*cos(head) - sin(head)*sin(pitch)*sin(roll));
    J(3*i-2,4) = -1;
    J(3*i-2,5) = 0;
    J(3*i-2,6) = 0;
    
    J(3*i-1,1) = pp1*(cos(roll)*sin(head)*cos(pitch)) + pp3*( - cos(head)*cos(roll)*cos(pitch)) + pp2*sin(pitch)*cos(roll);
    J(3*i-1,2) = pp1*(cos(head)*cos(roll) - sin(roll)*sin(head)*sin(pitch)) + pp3*(sin(head)*cos(roll) + cos(head)*sin(roll)*sin(pitch)) + pp2*cos(pitch)*sin(roll);
    J(3*i-1,3) = pp1*(-sin(head)*sin(roll) + cos(roll)*cos(head)*sin(pitch)) + pp3*(cos(head)*sin(roll) + sin(head)*cos(roll)*sin(pitch));
    J(3*i-1,4) = 0;
    J(3*i-1,5) = 0;
    J(3*i-1,6) = 1;
    
    J(3*i,1) = pp2*cos(pitch) + pp3*cos(head)*sin(pitch) - pp1*sin(pitch)*sin(head);
    J(3*i,2) = 0;
    J(3*i,3) = pp3*sin(head)*cos(pitch) + pp1*cos(pitch)*cos(head);
    J(3*i,4) = 0;
    J(3*i,5) = -1;
    J(3*i,6) = 0;
    
%     r = norm(Pc(:,i) - (R * Pp(:,i) + T));
%     J(3*i-2:3*i,:) = 2 * r * J(3*i-2:3*i,:);
    
    res = Pc(:,i) - (Rbc * R * Rcb * Pp(:,i) + Rbc * T);
%     JJ(i,:) = 2 * res' * J(3*i-2:3*i,:);
    J(3*i-2:3*i,:) = 2 * norm(res) * J(3*i-2:3*i,:);
end
Rk = inv(J'*J);
% Rk = inv(JJ'*JJ);
