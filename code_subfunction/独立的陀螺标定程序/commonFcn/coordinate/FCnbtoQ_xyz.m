% buaa xyz 2014.1.13

% 方向余弦矩阵 -> 四元数
% Q为n到b
function Q = FCnbtoQ_xyz(Cnb)
format long
% 根据初始姿态矩阵Cnb计算初始姿态四元数
% 参考《高钟毓P17》

q0 = 1/2*sqrt( 1+Cnb(1,1)+Cnb(2,2)+Cnb(3,3) ) ;
q1 = 1/(4*q0)*(Cnb(2,3)-Cnb(3,2));
q2 = 1/(4*q0)*(Cnb(3,1)-Cnb(1,3));
q3 = 1/(4*q0)*(Cnb(1,2)-Cnb(2,1));

Q=[q0;q1;q2;q3];
Q=Q/norm(Q);
