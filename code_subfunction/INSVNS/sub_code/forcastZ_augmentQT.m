%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%函数功能：量测预测
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Z_forcast = forcastZ_augmentQT(X_forcast)
q0 = X_forcast(17);
q1 = X_forcast(18);
q2 = X_forcast(19);
q3 = X_forcast(20);
Crbk0=[q0^2+q1^2-q2^2-q3^2,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2);
      2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3,2*(q2*q3-q0*q1);
      2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3]';     %calculate Cebk0
q0 = X_forcast(1);
q1 = X_forcast(2);
q2 = X_forcast(3);
q3 = X_forcast(4);
Crbk1=[q0^2+q1^2-q2^2-q3^2,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2);
      2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3,2*(q2*q3-q0*q1);
      2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3]';     %calculate Cebk1
Rk0k1 = Crbk1*Crbk0';
Tk0k1 = Crbk1*(X_forcast(21:23)-X_forcast(5:7));
% 根据相对姿态变化矩阵R计算姿态四元数
q1=1/2*sqrt(abs(1+Rk0k1(1,1)-Rk0k1(2,2)-Rk0k1(3,3)));
q2=1/2*sqrt(abs(1-Rk0k1(1,1)+Rk0k1(2,2)-Rk0k1(3,3)));
q3=1/2*sqrt(abs(1-Rk0k1(1,1)-Rk0k1(2,2)+Rk0k1(3,3)));
q0=sqrt(abs(1-q1^2-q2^2-q3^2));
if Rk0k1(2,3)-Rk0k1(3,2)<0
    q1=-q1;
end
if Rk0k1(3,1)-Rk0k1(1,3)<0
    q2=-q2;
end
if Rk0k1(1,2)-Rk0k1(2,1)<0
    q3=-q3;
end
Q0=[q0;q1;q2;q3];
Q0=Q0/norm(Q0);
Z_forcast = [Q0;Tk0k1];
