%% SINS力学方程（直接模型）
% 输入 X = (q,r,v,gyro,acc,q_last,r_last)
    % r:SINS解算导航系-世界坐标系  wirr,gr：,    IMU数据：wibb,fb
% 输出 dXdt（X的导数）
% 增广与不增广的都可以

function newX  = updateX_ZhiJie(X,wirr,gr,wibb,fb,T)
[dXdt,Wrbb] = dXdt_ZhiJie(X,wirr,gr,wibb,fb);

newX = X+dXdt*T ;   % 除四元数之外用这个方程计算

q = X(1:4);
q_new = QuaternionDifferential( q,Wrbb,T ) ;    % 四元数更新采用角增量方法

newX(1:4) = q_new;
if length(X)==23
	newX(17:23) = X(1:7);   % 增广的状态更新
end

function [dXdt,Wrbb] = dXdt_ZhiJie(X,wirr,gr,wibb,fb)

dXdt = zeros(size(X));      % 增广与不增广的都可以

Qrb = X(1:4);
r = X(5:7);
v = X(8:10);
gyro_const = X(11:13);
acc_const = X(14:16);

Crb = FQtoCnb(Qrb);
Wrbb = wibb-gyro_const-Crb*wirr ;
% Wrbb = wibb-Crb*wirr ;
Wrbb_Q = [0;Wrbb];
% dqdt = QuaternionMultiply(Qrb,Wrbb_Q)/2;

% Qrb_new = Qrb+dqdt*T ;
dqdt=0.5*[    0    ,-Wrbb(1),-Wrbb(2),-Wrbb(3);
            Wrbb(1),     0    , Wrbb(3),-Wrbb(2);
            Wrbb(2),-Wrbb(3),     0    , Wrbb(1);
            Wrbb(3), Wrbb(2),-Wrbb(1),     0    ]*Qrb;

drdt = v;

dvdt = Crb'*(fb-acc_const)-2*getCrossMatrix(wirr)*v+gr ;

dXdt(1:4) = dqdt;
dXdt(5:7) = drdt;
dXdt(8:10) = dvdt;


