% buaa xyz 2014.4.17（纠正版）

% 方向余弦矩阵 -> 四元数
% Q为n到b
 
% FCnbtoQ_new 比 FCnbtoQ_old 精度略高

function Q = FCnbtoQ(Cnb)

Q = FCnbtoQ_new(Cnb);

function Q = FCnbtoQ_new(Cnb)

format long
% 根据初始姿态矩阵Cnb计算初始姿态四元数
% 参考《高钟毓P17》

q0 = 1/2*sqrt( 1+Cnb(1,1)+Cnb(2,2)+Cnb(3,3) ) ;
q1 = 1/(4*q0)*(Cnb(2,3)-Cnb(3,2));
q2 = 1/(4*q0)*(Cnb(3,1)-Cnb(1,3));
q3 = 1/(4*q0)*(Cnb(1,2)-Cnb(2,1));

Q=[q0;q1;q2;q3];
Q=Q/norm(Q);



function Q = FCnbtoQ_old(Cnb)
%% 这个是2014.4.17之前用的函数，是错的
format long
% 根据初始姿态矩阵Cnb计算初始姿态四元数
q1=1/2*sqrt(abs(1+Cnb(1,1)-Cnb(2,2)-Cnb(3,3)));
q2=1/2*sqrt(abs(1-Cnb(1,1)+Cnb(2,2)-Cnb(3,3)));
q3=1/2*sqrt(abs(1-Cnb(1,1)-Cnb(2,2)+Cnb(3,3)));


q0=1/2*sqrt(abs(1+Cnb(1,1)+Cnb(2,2)+Cnb(3,3)));     % xyz
%q0=sqrt(abs(1-q1^2-q2^2-q3^2));                    % ning


if Cnb(2,3)-Cnb(3,2)<0
    q1=-q1;
end
if Cnb(3,1)-Cnb(1,3)<0
    q2=-q2;
end
if Cnb(1,2)-Cnb(2,1)<0
    q3=-q3;
end
Q=[q0;q1;q2;q3];
Q=Q/norm(Q);


