% buaa xyz
% 2014.5.12
% 计算SINS直接模型的 量测方程 雅克比矩阵，状态四元数形式，量测Qbb Tbb
%   16维：X = (q,r,v,gyro,acc,qlast,rlast) 
% 忽略了 gr 的变化

function augZhijieSINS_QbbTbb_syms_H = augZhijieSINS_QbbTbb_syms_H
% [q0,q1,q2,q3,q0_last,q1_last,q2_last,q3_last,rx,ry,rz,rx_last,ry_last,rz_last]
syms Qrb r v gyro acc
syms q0 q1 q2 q3 rx ry rz vx vy vz gyrox gyroy gyroz accx accy accz q0_last q1_last q2_last q3_last rx_last ry_last rz_last

syms wirrx wirry wirrz wibbx wibby wibbz fbx fby fbz wibbx wibby wibbz grx gry grz
% syms fb wibb wirr gr


qk = [q0;q1;q2;q3];
qk_last = [q0_last;q1_last;q2_last;q3_last];
rk = [rx;ry;rz];
rk_last = [rx_last;ry_last;rz_last];

v = [vx;vy;vz] ;
gyro = [gyrox;gyroy;gyroz] ;
acc = [accx;accy;accz] ;


qk_last_niv = [qk_last(1);-qk_last(2:4)];
Qbb = QuaternionMultiply(qk_last_niv,qk) ;


Tbb = FQtoCnb_forSym(qk)*(rk-rk_last) ;
Z = [Qbb;Tbb];
X = [qk;rk;v;gyro;acc;qk_last;rk_last];

augZhijieSINS_QbbTbb_syms_H = jacobian(Z,X);
save augZhijieSINS_QbbTbb_syms_H augZhijieSINS_QbbTbb_syms_H
symvar(augZhijieSINS_QbbTbb_syms_H)

function Cnb = FQtoCnb_forSym(Q)
q0 = Q(1) ;
q1 = Q(2) ;
q2 = Q(3) ;
q3 = Q(4) ;
Cnb=[   q0^2+q1^2-q2^2-q3^2,    2*(q1*q2+q0*q3),            2*(q1*q3-q0*q2);
        2*(q1*q2-q0*q3),        q0*q0-q1*q1+q2*q2-q3*q3,    2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2),        2*(q2*q3-q0*q1),            q0*q0-q1*q1-q2*q2+q3*q3     ];