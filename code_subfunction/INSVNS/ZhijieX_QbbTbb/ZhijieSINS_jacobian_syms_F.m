% buaa xyz
% 2014.5.10
% 计算SINS直接模型雅克比矩阵，四元数形式
%   16维：X = (q,r,v,gyro,acc)
% 忽略了 gr 的变化

function ZhijieSINS_jacobian_syms_F = ZhijieSINS_jacobian_syms_F
% 需要更新的参数[ accx, accy, accz, fbx, fby, fbz, gyrox, gyroy, gyroz, q0, q1, q2, q3, wibbx, wibby, wibbz, wirrx, wirry, wirrz]

syms Qrb r v gyro acc
syms q0 q1 q2 q3 rx ry rz vx vy vz gyrox gyroy gyroz accx accy accz

syms wirrx wirry wirrz wibbx wibby wibbz fbx fby fbz wibbx wibby wibbz grx gry grz
% syms fb wibb wirr gr


Qrb = [q0;q1;q2;q3];
r = [rx;ry;rz] ;
v = [vx;vy;vz] ;
gyro = [gyrox;gyroy;gyroz] ;
acc = [accx;accy;accz] ;

wirr = [wirrx; wirry; wirrz] ;
wibb = [wibbx; wibby; wibbz] ;
fb = [fbx;fby;fbz];
gr = [grx;gry;grz];

Crb = FQtoCnb_forSym(Qrb);
Wrbb = wibb-gyro-Crb*wirr ;
dqdt=0.5*[    0    ,-Wrbb(1),-Wrbb(2),-Wrbb(3);
            Wrbb(1),     0    , Wrbb(3),-Wrbb(2);
            Wrbb(2),-Wrbb(3),     0    , Wrbb(1);
            Wrbb(3), Wrbb(2),-Wrbb(1),     0    ]*Qrb ;
drdt = v;
dvdt = Crb'*(fb-acc)-2*getCrossMarix(wirr)*v+gr ;     

dXdt = [dqdt;drdt;dvdt;zeros(6,1)];
X = [Qrb;r;v;gyro;acc];
ZhijieSINS_jacobian_syms_F = jacobian(dXdt,X) ;

save ZhijieSINS_jacobian_syms_F ZhijieSINS_jacobian_syms_F

FF_11 = ZhijieSINS_jacobian_syms_F(1:4,1:4) ;

function Cnb = FQtoCnb_forSym(Q)
q0 = Q(1) ;
q1 = Q(2) ;
q2 = Q(3) ;
q3 = Q(4) ;
Cnb=[   q0^2+q1^2-q2^2-q3^2,    2*(q1*q2+q0*q3),            2*(q1*q3-q0*q2);
        2*(q1*q2-q0*q3),        q0*q0-q1*q1+q2*q2-q3*q3,    2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2),        2*(q2*q3-q0*q1),            q0*q0-q1*q1-q2*q2+q3*q3     ];