%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%日期：2012.12.22
%作者：白鑫贝
%程序说明：利用LM优化算法求解两时刻之间的旋转矩阵和平移矢量，
%         未知量为姿态四元数和三个方向的平移量。
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X = LMalgorithm1(Pc,Pp,Q0,T0)
n = size(Pc,2); % feature number
alpha = 0.01;
beta = 10;
delta = 1e-5;
X0 = [Q0;T0];
Ak = zeros(3*n,7);
fk = zeros(3*n,1);
while 1
    q0 = X0(1);
    q1 = X0(2);
    q2 = X0(3);
    q3 = X0(4);
    tx = X0(5);
    ty = X0(6);
    tz = X0(7);
    R = [q0^2+q1^2-q2^2-q3^2,    2*(q1*q2+q0*q3),        2*(q1*q3-q0*q2);
         2*(q1*q2-q0*q3),    q0*q0-q1*q1+q2*q2-q3*q3,    2*(q2*q3+q0*q1);
         2*(q1*q3+q0*q2),        2*(q2*q3-q0*q1),    q0*q0-q1*q1-q2*q2+q3*q3];
    T = [tx;ty;tz];
    for i = 1:n
        pp1 = Pp(1,i);
        pp2 = Pp(2,i);
        pp3 = Pp(3,i);
        pc1 = Pc(1,i);
        pc2 = Pc(2,i);
        pc3 = Pc(3,i);
        Ak(3*i-2,1) = - 2*pp1*q0 - 2*pp2*q3 + 2*pp3*q2;
        Ak(3*i-2,2) = - 2*pp1*q1 - 2*pp2*q2 - 2*pp3*q3;
        Ak(3*i-2,3) = 2*pp1*q2 - 2*pp2*q1 + 2*pp3*q0;
        Ak(3*i-2,4) = 2*pp1*q3 - 2*pp2*q0 - 2*pp3*q1;
        Ak(3*i-2,5) = - 1;
        Ak(3*i-2,6) = 0;
        Ak(3*i-2,7) = 0;
        
        Ak(3*i-1,1) = - 2*pp2*q0 + 2*pp1*q3 - 2*pp3*q1;
        Ak(3*i-1,2) = 2*pp2*q1 - 2*pp1*q2 - 2*pp3*q0;
        Ak(3*i-1,3) = - 2*pp2*q2 - 2*pp1*q1 - 2*pp3*q3;
        Ak(3*i-1,4) = 2*pp2*q3 + 2*pp1*q0 - 2*pp3*q2;
        Ak(3*i-1,5) = 0;
        Ak(3*i-1,6) = - 1;
        Ak(3*i-1,7) = 0;
        
        Ak(3*i,1) = - 2*pp3*q0 - 2*pp1*q2 + 2*pp2*q1;
        Ak(3*i,2) = 2*pp3*q1 - 2*pp1*q3 + 2*pp2*q0;
        Ak(3*i,3) = 2*pp3*q2 - 2*pp1*q0 - 2*pp2*q3;
        Ak(3*i,4) = - 2*pp3*q3 - 2*pp1*q1 - 2*pp2*q2;
        Ak(3*i,5) = 0;
        Ak(3*i,6) = 0;
        Ak(3*i,7) = - 1;
        fk(3*i-2:3*i) = - R * [pp1;pp2;pp3] - T + [pc1;pc2;pc3];
    end
    A = Ak' * Ak + alpha * eye(7);
    b = - Ak' * fk;
    % 解方程组
    dk = LineMainGauss(A,b);
    X = X0 + dk;
    X(1:4) = X(1:4) / norm(X(1:4));
    % 判断
    R1 = [X(1)^2+X(2)^2-X(3)^2-X(4)^2,    2*(X(2)*X(3)+X(1)*X(4)),        2*(X(2)*X(4)-X(1)*X(3));
         2*(X(2)*X(3)-X(1)*X(4)),    X(1)*X(1)-X(2)*X(2)+X(3)*X(3)-X(4)*X(4),    2*(X(3)*X(4)+X(1)*X(2));
         2*(X(2)*X(4)+X(1)*X(3)),        2*(X(3)*X(4)-X(1)*X(2)),    X(1)*X(1)-X(2)*X(2)-X(3)*X(3)+X(4)*X(4)];
    T1 = X(5:7);
    F0 = 0;
    F1 = 0;
    for i = 1:n
        F0 = F0 + norm( - R * Pp(:,i) - T + Pc(:,i));
        F1 = F1 + norm( - R1 * Pp(:,i) - T1 + Pc(:,i));
    end
%     if F1 < F0
%         cc = norm(Ak' * fk);
%         if cc <= delta
%             break;
%         else
%             X0 = X;
%             alpha = alpha / beta;
%             continue;
%         end
%     else
%         cc = norm(Ak' * fk);
%         if cc <= delta
%             X = X0;
%             break;
%         else
%             alpha = alpha * beta;
%             continue;
%         end
%     end
    if F1 < F0
        if abs((F1 - F0) / F0) <= delta
            break;
        else
            X0 = X;
            alpha = alpha / beta;
            continue;
        end
    else
        if abs((F1 - F0) / F0) <= delta
            X = X0;
            break;
        else
            alpha = alpha * beta;
            continue;
        end
    end
end
