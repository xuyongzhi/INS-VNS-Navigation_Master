% buaa xyz 2014.1.10

% 四元数 -> 方向余弦矩阵

function Cnb = FQtoCnb(Q)
format long
Q=Q/norm(Q);
q0 = Q(1) ;
q1 = Q(2) ;
q2 = Q(3) ;
q3 = Q(4) ;
Cnb=[   q0^2+q1^2-q2^2-q3^2,    2*(q1*q2+q0*q3),            2*(q1*q3-q0*q2);
        2*(q1*q2-q0*q3),        q0*q0-q1*q1+q2*q2-q3*q3,    2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2),        2*(q2*q3-q0*q1),            q0*q0-q1*q1-q2*q2+q3*q3     ];
