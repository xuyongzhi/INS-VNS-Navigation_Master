%% 惯导误差方程(线性时变)
% X=[dangleEsm;dVel;dPos;gyroDrift;accDrift]
% 输入：SINS 解算坐标系为 r ,b 为当前本体系
% 输出：F：状态矩阵,G：噪声矩阵
function [F,G] = GetF_StatusErrorSINS(Cbr,Wirr,fb)
format long
F11 = -getCrossMarix(Wirr);
F14 = Cbr;
fr = Cbr * fb;
F21 = -getCrossMarix(fr);
F22 = -2 * getCrossMarix(Wirr);
F25 = Cbr;
F32 = eye(3);
% 得状态矩阵
F = [F11,zeros(3),zeros(3),     F14, zeros(3);
       F21,     F22,zeros(3),zeros(3),      F25;
       zeros(3),F32,zeros(3),zeros(3), zeros(3);
       zeros(3,15);zeros(3,15)  ];
G = [Cbr,zeros(3);zeros(3),Cbr;zeros(3,6);zeros(3,6);zeros(3,6)];