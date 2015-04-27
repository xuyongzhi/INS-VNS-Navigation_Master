% buaa xyz 2014.1.17
% 源至白

% 惯导力学状态方程（非线性时变）
% 输入：
%       Q为姿态四元数，gyroDrift为陀螺漂移，accDrift为加计漂移，输入时这三个从状态变量中提取
%                            X=[dangleEsm;dVel;dPos;gyroDrift;accDrift]
%       wibb:陀螺数据
%       fb：加计数据
%       wirr：惯导解算参考系自转角速度
% 输出：雅克比状态矩阵

function F = GetF_StatusSINS(Q,gyroDrift,accDrift,wibb,fb,wirr)
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);      % 姿态四元数

F1(1,1) = q1*(wirr(2)*q3 - wirr(3)*q2 + wirr(1)*q0) + q2*(-wirr(1)*q3 + wirr(3)*q1 + wirr(2)*q0) + q3*(wirr(1)*q2 - wirr(2)*q1 + wirr(3)*q0);
F1(1,2) = (gyroDrift(1)/2 - wibb(1)/2 + (wirr(2)*(2*q0*q3 + 2*q1*q2))/2 - (wirr(3)*(2*q0*q2 - 2*q1*q3))/2 + (wirr(1)*(q0^2 + q1^2 - q2^2 - q3^2))/2)...
           + q1*(wirr(2)*q2 + wirr(3)*q3 + wirr(1)*q1) + q2*(wirr(1)*q2 + wirr(3)*q0 - wirr(2)*q1) + q3*(wirr(1)*q3 - wirr(2)*q0 - wirr(3)*q1);
F1(1,3) = q1*(wirr(2)*q1 - wirr(3)*q0 - wirr(1)*q2) + (gyroDrift(2)/2 - wibb(2)/2 - (wirr(1)*(2*q0*q3 - 2*q1*q2))/2 + (wirr(3)*(2*q0*q1 + 2*q2*q3))/2 + (wirr(2)*(q0^2 - q1^2 + q2^2 - q3^2))/2)...
           + q2*(wirr(1)*q1 + wirr(3)*q3 + wirr(2)*q2) + q3*(wirr(1)*q0 + wirr(2)*q3 - wirr(3)*q2);
F1(1,4) = q1*(wirr(2)*q0 + wirr(3)*q1 - wirr(1)*q3) + q2*(-wirr(1)*q0 + wirr(3)*q2 - wirr(2)*q3) + q3*(wirr(1)*q1 + wirr(2)*q2 + wirr(3)*q3)...
           + (gyroDrift(3)/2 - wibb(3)/2 + (wirr(1)*(2*q0*q2 + 2*q1*q3))/2 - (wirr(2)*(2*q0*q1 - 2*q2*q3))/2 + (wirr(3)*(q0^2 - q1^2 - q2^2 + q3^2))/2);
F1(2,1) = q3*(-wirr(1)*q3 + wirr(3)*q1 + wirr(2)*q0) - q2*(wirr(1)*q2 - wirr(2)*q1 + wirr(3)*q0) - q0*(wirr(2)*q3 - wirr(3)*q2 + wirr(1)*q0)...
           - (gyroDrift(1)/2 - wibb(1)/2 + (wirr(2)*(2*q0*q3 + 2*q1*q2))/2 - (wirr(3)*(2*q0*q2 - 2*q1*q3))/2 + (wirr(1)*(q0^2 + q1^2 - q2^2 - q3^2))/2);
F1(2,2) = q3*(wirr(1)*q2 + wirr(3)*q0 - wirr(2)*q1) - q2*(wirr(1)*q3 - wirr(2)*q0 - wirr(3)*q1) - q0*(wirr(2)*q2 + wirr(3)*q3 + wirr(1)*q1);
F1(2,3) = q3*(wirr(1)*q1 + wirr(3)*q3 + wirr(2)*q2) - q2*(wirr(1)*q0 + wirr(2)*q3 - wirr(3)*q2) - q0*(wirr(2)*q1 - wirr(3)*q0 - wirr(1)*q2)...
           - (gyroDrift(3)/2 - wibb(3)/2 + (wirr(1)*(2*q0*q2 + 2*q1*q3))/2 - (wirr(2)*(2*q0*q1 - 2*q2*q3))/2 + (wirr(3)*(q0^2 - q1^2 - q2^2 + q3^2))/2);
F1(2,4) = (gyroDrift(2)/2 - wibb(2)/2 - (wirr(1)*(2*q0*q3 - 2*q1*q2))/2 + (wirr(3)*(2*q0*q1 + 2*q2*q3))/2 + (wirr(2)*(q0^2 - q1^2 + q2^2 - q3^2))/2)...
           + q3*(-wirr(1)*q0 + wirr(3)*q2 - wirr(2)*q3) - q2*(wirr(1)*q1 + wirr(2)*q2 + wirr(3)*q3) - q0*(wirr(2)*q0 + wirr(3)*q1 - wirr(1)*q3);
F1(3,1) = q1*(wirr(1)*q2 - wirr(2)*q1 + wirr(3)*q0) - q0*(-wirr(1)*q3 + wirr(3)*q1 + wirr(2)*q0) - q3*(wirr(2)*q3 - wirr(3)*q2 + wirr(1)*q0)...
           - (gyroDrift(2)/2 - wibb(2)/2 - (wirr(1)*(2*q0*q3 - 2*q1*q2))/2 + (wirr(3)*(2*q0*q1 + 2*q2*q3))/2 + (wirr(2)*(q0^2 - q1^2 + q2^2 - q3^2))/2);
F1(3,2) = (gyroDrift(3)/2 - wibb(3)/2 + (wirr(1)*(2*q0*q2 + 2*q1*q3))/2 - (wirr(2)*(2*q0*q1 - 2*q2*q3))/2 + (wirr(3)*(q0^2 - q1^2 - q2^2 + q3^2))/2)...
           + q1*(wirr(1)*q3 - wirr(2)*q0 - wirr(3)*q1) - q0*(wirr(1)*q2 + wirr(3)*q0 - wirr(2)*q1) - q3*(wirr(2)*q2 + wirr(3)*q3 + wirr(1)*q1);
F1(3,3) = q1*(wirr(1)*q0 + wirr(2)*q3 - wirr(3)*q2) - q0*(wirr(1)*q1 + wirr(3)*q3 + wirr(2)*q2) - q3*(wirr(2)*q1 - wirr(3)*q0 - wirr(1)*q2);
F1(3,4) = q1*(wirr(1)*q1 + wirr(2)*q2 + wirr(3)*q3) - q0*(-wirr(1)*q0 + wirr(3)*q2 - wirr(2)*q3) - q3*(wirr(2)*q0 + wirr(3)*q1 - wirr(1)*q3)...
           - (gyroDrift(1)/2 - wibb(1)/2 + (wirr(2)*(2*q0*q3 + 2*q1*q2))/2 - (wirr(3)*(2*q0*q2 - 2*q1*q3))/2 + (wirr(1)*(q0^2 + q1^2 - q2^2 - q3^2))/2);
F1(4,1) = q2*(wirr(2)*q3 - wirr(3)*q2 + wirr(1)*q0) - q1*(-wirr(1)*q3 + wirr(3)*q1 + wirr(2)*q0) - q0*(wirr(1)*q2 - wirr(2)*q1 + wirr(3)*q0)...
           - (gyroDrift(3)/2 - wibb(3)/2 + (wirr(1)*(2*q0*q2 + 2*q1*q3))/2 - (wirr(2)*(2*q0*q1 - 2*q2*q3))/2 + (wirr(3)*(q0^2 - q1^2 - q2^2 + q3^2))/2);
F1(4,2) = q2*(wirr(2)*q2 + wirr(3)*q3 + wirr(1)*q1) - q1*(wirr(1)*q2 + wirr(3)*q0 - wirr(2)*q1) - q0*(wirr(1)*q3 - wirr(2)*q0 - wirr(3)*q1)...
           - (gyroDrift(2)/2 - wibb(2)/2 - (wirr(1)*(2*q0*q3 - 2*q1*q2))/2 + (wirr(3)*(2*q0*q1 + 2*q2*q3))/2 + (wirr(2)*(q0^2 - q1^2 + q2^2 - q3^2))/2);
F1(4,3) = (gyroDrift(1)/2 - wibb(1)/2 + (wirr(2)*(2*q0*q3 + 2*q1*q2))/2 - (wirr(3)*(2*q0*q2 - 2*q1*q3))/2 + (wirr(1)*(q0^2 + q1^2 - q2^2 - q3^2))/2)...
           + q2*(wirr(2)*q1 - wirr(3)*q0 - wirr(1)*q2) - q1*(wirr(1)*q1 + wirr(3)*q3 + wirr(2)*q2) - q0*(wirr(1)*q0 + wirr(2)*q3 - wirr(3)*q2);
F1(4,4) = q2*(wirr(2)*q0 + wirr(3)*q1 - wirr(1)*q3) - q1*(-wirr(1)*q0 + wirr(3)*q2 - wirr(2)*q3) - q0*(wirr(1)*q1 + wirr(2)*q2 + wirr(3)*q3);
F2(1,1) = 1/2 * q1;
F2(1,2) = 1/2 * q2;
F2(1,3) = 1/2 * q3;
F2(2,1) = -1/2 * q0;
F2(2,2) = 1/2 * q3;
F2(2,3) = -1/2 * q2;
F2(3,1) = -1/2 * q3;
F2(3,2) = -1/2 * q0;
F2(3,3) = 1/2 * q1;
F2(4,1) = 1/2 * q2;
F2(4,2) = -1/2 * q1;
F2(4,3) = -1/2 * q0;
F3 = eye(3);
Fv = -2 * [    0    -wirr(3)  wirr(2);
            wirr(3)     0    -wirr(1);
           -wirr(2)  wirr(1)     0   ];
Cbr=[q0^2+q1^2-q2^2-q3^2,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3,2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3];
Fa = -Cbr;
F4(1,1) = 2 * q0 * (fb(1) - accDrift(1)) - 2 * q3 * (fb(2) - accDrift(2)) + 2 * q2 * (fb(3) - accDrift(3));
F4(1,2) = 2 * q1 * (fb(1) - accDrift(1)) + 2 * q2 * (fb(2) - accDrift(2)) + 2 * q3 * (fb(3) - accDrift(3));
F4(1,3) = - 2 * q2 * (fb(1) - accDrift(1)) + 2 * q1 * (fb(2) - accDrift(2)) + 2 * q0 * (fb(3) - accDrift(3));
F4(1,4) = - 2 * q3 * (fb(1) - accDrift(1)) - 2 * q0 * (fb(2) - accDrift(2)) + 2 * q1 * (fb(3) - accDrift(3));
F4(2,1) = 2 * q3 * (fb(1) - accDrift(1)) + 2 * q0 * (fb(2) - accDrift(2)) - 2 * q1 * (fb(3) - accDrift(3));
F4(2,2) = 2 * q2 * (fb(1) - accDrift(1)) - 2 * q1 * (fb(2) - accDrift(2)) - 2 * q0 * (fb(3) - accDrift(3));
F4(2,3) = 2 * q1 * (fb(1) - accDrift(1)) + 2 * q2 * (fb(2) - accDrift(2)) + 2 * q3 * (fb(3) - accDrift(3));
F4(2,4) = 2 * q0 * (fb(1) - accDrift(1)) - 2 * q3 * (fb(2) - accDrift(2)) + 2 * q2 * (fb(3) - accDrift(3));
F4(3,1) = - 2 * q2 * (fb(1) - accDrift(1)) + 2 * q1 * (fb(2) - accDrift(2)) + 2 * q0 * (fb(3) - accDrift(3));
F4(3,2) = 2 * q3 * (fb(1) - accDrift(1)) + 2 * q0 * (fb(2) - accDrift(2)) - 2 * q1 * (fb(3) - accDrift(3));
F4(3,3) = - 2 * q0 * (fb(1) - accDrift(1)) + 2 * q3 * (fb(2) - accDrift(2)) - 2 * q2 * (fb(3) - accDrift(3));
F4(3,4) = 2 * q1 * (fb(1) - accDrift(1)) + 2 * q2 * (fb(2) - accDrift(2)) + 2 * q3 * (fb(3) - accDrift(3));
% F = [   F1,     zeros(4,3),zeros(4,3),   F2,     zeros(4,3),zeros(4,7);
%      zeros(3,4),zeros(3,3),    F3,    zeros(3,3),zeros(3,3),zeros(3,7);
%         F4,     zeros(3,3),    Fv,    zeros(3,3),    Fa,    zeros(3,7);
%      zeros(6,23);
%      zeros(7,23)];
%-------  2013.3.1 modify  -------%
F = [   F1,     zeros(4,3),zeros(4,3),   F2,     zeros(4,3);
     zeros(3,4),zeros(3,3),    F3,    zeros(3,3),zeros(3,3);
        F4,     zeros(3,3),    Fv,    zeros(3,3),    Fa    ;
     zeros(6,16)];      % 15*16