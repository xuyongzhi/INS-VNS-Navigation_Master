%% EKF_augZhijie_QT 滤波
%   增广直接状态模型，Qbb和Tbb为量测，EKF滤波方法
%   16+7=23维：X = (q,r,v,gyro,acc,q_last,r_last)

% X_SINSPre： SINS递推状态值

function [X_new,P_new,X_pre,X_correct,Z_pre,Z,Zerror] = SINSEKF_augZhijie_QT(Rbb_VNS,Tbb_VNS,isTbb_last,T,X_last,X_SINSPre,P_last,Q,R,wirr,gr,wibb,fb,isSINSpre)
                                                
% 计算量测量
Qbb = FCnbtoQ(Rbb_VNS);

if isTbb_last==1
    Tbb_VNS = Rbb_VNS * Tbb_VNS ;
end

Z = [Qbb;Tbb_VNS];
% 一步状态预测
X_pre  = updateX_ZhiJie(X_last,wirr,gr,wibb,fb,T);

if isSINSpre==1
   % 采用SINS递推值作为状态一步预测
   X_pre(1:10) = X_SINSPre(1:10) ;
   % X_SINSPre(17:23)本就是一样的
end
% 量测量预测
Z_pre = hfun_QbbTbb_ZhijieX(X_pre) ;
% 计算状态转移矩阵 Fai
% dbstop in GetF_augSINSZhiJie
F = GetF_augSINSZhiJie(X_last(1:4),X_last(11:13),X_last(14:16),fb,wibb,wirr) ;
Fai = FtoFai(F,T);
% dbstop in GetH_augSINSZhiJie
H = GetH_augSINSZhiJie(X_last(1:4),X_last(5:7),X_last(17:20),X_last(21:23));
% 均方误差阵一步预测
P_pre = Fai*P_last*Fai'+Q ;
% 计算增益阵
K = P_pre*H'/(H*P_pre*H'+R);
% 状态估计
X_correct = K*(Z-Z_pre);
X_new = X_pre+X_correct ;
X_new(1:4) = X_new(1:4)/norm(X_new(1:4));
X_new(20:23) = X_new(20:23)/norm(X_new(20:23));
% 均方误差估计
temp = eye(length(X_last))-K*H ;
P_new = temp*P_pre*temp'+K*R*K' ;

Zerror = Z-Z_pre;

function F = GetF_augSINSZhiJie(Qrb,gyroDrift,accDrift,fb,wibb,wirr)
%   16+7=23维：X = (q,r,v,gyro,acc,q_last,r_last)

% 先计算非增广部分的状态雅克比矩阵：导入syms的雅克比形式，再更新参数

% 非增广部分
% syms_FF = importdata('ZhijieSINS_jacobian_syms_F');
% accx = accDrift(1);
% accy = accDrift(2);
% accz = accDrift(3);
% fbx=fb(1);
% fby=fb(2);
% fbz=fb(3);
% gyrox=gyroDrift(1);
% gyroy=gyroDrift(2);
% gyroz=gyroDrift(3);
q0=Qrb(1);
q1=Qrb(2);
q2=Qrb(3);
q3=Qrb(4);
% wibbx=wibb(1);
% wibby=wibb(2);
% wibbz=wibb(3);
% wirrx=wirr(1);
% wirry=wirr(2);
% wirrz=wirr(3);

%%%%%%%%  (1)  %%%%%%%%
% FF = subs(syms_F);

%%%%%%%%  (2)  %%%%%%%%
% syms_FF_23 = syms_FF(5:7,8:10);
% syms_FF_33 = syms_FF(8:10,8:10);
% syms_FF_35 = syms_FF(8:10,14:16);
% syms_FF_14 = syms_FF(1:4,11:13);
% 
% sFF_23 = subs(syms_FF_23) ;
% sFF_33 = subs(syms_FF_33) ;
% sFF_35 = subs(syms_FF_35) ;
% sFF_14 = subs(syms_FF_14) ;

% syms_FF_11 = syms_FF(1:4,1:4);
% sFF_11 = subs(syms_FF_11) ;

% syms_FF_31 = syms_FF(8:10,1:4);
% sFF_31 = subs(syms_FF_31) ;

% FF_11 = sFF_11;
% FF_14 = sFF_14 ;
% FF_31 = sFF_31 ;
%%%%%%%%  (3)  %%%%%%%%
Crb = FQtoCnb(Qrb);

FF_11 = Get_FF11_xyz(Qrb,wirr,gyroDrift,wibb);
% FF_11bai = Get_FF11_bai(Qrb,wirr,gyroDrift,wibb);
FF_14 = -1/2*[  -q1 -q2 -q3;
                q0  -q3  q2;
                q3  q0   -q1;
                -q2 q1   q0  ];        
FF_23 = eye(3) ;

FF_31(1:3,1) = [ 2*q0 2*q3 2*q2; 2*q3 2*q0 -2*q1 ; -2*q2 2*q1 2*q0]*(fb-accDrift) ;
FF_31(1:3,2) = [ 2*q1 2*q2 2*q3; 2*q2 -2*q1 -2*q0 ; 2*q3 2*q0 -2*q1]*(fb-accDrift) ;
FF_31(1:3,3) = [ -2*q2 2*q1 2*q0; 2*q1 2*q2 2*q3 ; -2*q0 2*q3 -2*q2]*(fb-accDrift) ;
FF_31(1:3,4) = [ -2*q3 -2*q0 2*q1; 2*q0 -2*q3 2*q2 ; 2*q1 2*q2 2*q3]*(fb-accDrift) ;
FF_33 = -2*getCrossMatrix(wirr) ;
FF_35 = -Crb' ;

FF = [FF_11 zeros(4,3) zeros(4,3) FF_14 zeros(4,3) ;
      zeros(3,4) zeros(3,3) FF_23 zeros(3,3) zeros(3,3);
      FF_31 zeros(3,3) FF_33 zeros(3,3) FF_35;
      zeros(6,16)
    ];

% b=FF_23-sFF_23;
% c=FF_33-sFF_33;
% d=FF_35-sFF_35;
% e=FF_14-sFF_14;
% f=FF_11-sFF_11;
% g=FF_11bai-FF_11;
% fh=FF_11bai-sFF_11;
% sd=FF_31-sFF_31;

F = [FF zeros(16,7); zeros(7,23) ];

function F11 = Get_FF11_xyz(Qrb,wirr,gyroDrift,wibb)
% xyz求 FF(1,1)
q0=Qrb(1);
q1=Qrb(2);
q2=Qrb(3);
q3=Qrb(4);
Crb = FQtoCnb(Qrb);
% Wrbb = wibb-gyroDrift-Crb*wirr ;
wirb = Crb*wirr ;
% 先求 wirb 对Q的雅克比矩阵 d_wirb_dQ (3*4)
d_wirb_dQ(1:3,1) = [ 2*q0 2*q3 -2*q2; -2*q3 2*q0 2*q1 ; 2*q2 -2*q1 2*q0]*wirr ;
d_wirb_dQ(1:3,2) = [ 2*q1 2*q2 2*q3; 2*q2 -2*q1 2*q0 ; 2*q3 -2*q0 -2*q1]*wirr ;
d_wirb_dQ(1:3,3) = [ -2*q2 2*q1 -2*q0; 2*q1 2*q2 2*q3 ; 2*q0 2*q3 -2*q2]*wirr ;
d_wirb_dQ(1:3,4) = [ -2*q3 2*q0 2*q1; -2*q0 -2*q3 2*q2 ; 2*q1 2*q2 2*q3]*wirr ;
F11tempB = zeros(4,4);
F11tempB(1,1) = -1/2*[-q1 -q2 -q3]*d_wirb_dQ(1:3,1) ;
F11tempB(2,1) = -1/2*wirb(1)-1/2*[ q0 -q3 q2 ]*d_wirb_dQ(1:3,1) ;
F11tempB(3,1) = -1/2*wirb(2)-1/2*[ q3 q0 -q1 ]*d_wirb_dQ(1:3,1) ;
F11tempB(4,1) = -1/2*wirb(3)-1/2*[ -q2 q1 q0 ]*d_wirb_dQ(1:3,1) ;

F11tempB(1,2) = 1/2*wirb(1)-1/2*[-q1 -q2 -q3]*d_wirb_dQ(1:3,2) ;
F11tempB(2,2) = -1/2*[ q0 -q3 q2 ]*d_wirb_dQ(1:3,2) ;
F11tempB(3,2) = 1/2*wirb(3)-1/2*[ q3 q0 -q1 ]*d_wirb_dQ(1:3,2) ;
F11tempB(4,2) = -1/2*wirb(2)-1/2*[ -q2 q1 q0 ]*d_wirb_dQ(1:3,2) ;

F11tempB(1,3) = 1/2*wirb(2)-1/2*[-q1 -q2 -q3]*d_wirb_dQ(1:3,3) ;
F11tempB(2,3) = -1/2*wirb(3)-1/2*[ q0 -q3 q2 ]*d_wirb_dQ(1:3,3) ;
F11tempB(3,3) = -1/2*[ q3 q0 -q1 ]*d_wirb_dQ(1:3,3) ;
F11tempB(4,3) = 1/2*wirb(1)-1/2*[ -q2 q1 q0 ]*d_wirb_dQ(1:3,3) ;

F11tempB(1,4) = 1/2*wirb(3)-1/2*[-q1 -q2 -q3]*d_wirb_dQ(1:3,4) ;
F11tempB(2,4) = 1/2*wirb(2)-1/2*[ q0 -q3 q2 ]*d_wirb_dQ(1:3,4) ;
F11tempB(3,4) = -1/2*wirb(1)-1/2*[ q3 q0 -q1 ]*d_wirb_dQ(1:3,4) ;
F11tempB(4,4) = -1/2*[ -q2 q1 q0 ]*d_wirb_dQ(1:3,4) ;

wA = wibb-gyroDrift ;
F11tempA = 0.5*[    0    ,-wA(1),-wA(2),-wA(3);
            wA(1),     0    , wA(3),-wA(2);
            wA(2),-wA(3),     0    , wA(1);
            wA(3), wA(2),-wA(1),     0    ] ;
        
F11 =    F11tempA + F11tempB;   
        
function F11 = Get_FF11_bai(Q,wirr,gyroDrift,wibb)
% 白师姐求 FF(1,1)
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);      % 姿态四元数
F11(1,1) = q1*(wirr(2)*q3 - wirr(3)*q2 + wirr(1)*q0) + q2*(-wirr(1)*q3 + wirr(3)*q1 + wirr(2)*q0) + q3*(wirr(1)*q2 - wirr(2)*q1 + wirr(3)*q0);
F11(1,2) = (gyroDrift(1)/2 - wibb(1)/2 + (wirr(2)*(2*q0*q3 + 2*q1*q2))/2 - (wirr(3)*(2*q0*q2 - 2*q1*q3))/2 + (wirr(1)*(q0^2 + q1^2 - q2^2 - q3^2))/2)...
           + q1*(wirr(2)*q2 + wirr(3)*q3 + wirr(1)*q1) + q2*(wirr(1)*q2 + wirr(3)*q0 - wirr(2)*q1) + q3*(wirr(1)*q3 - wirr(2)*q0 - wirr(3)*q1);
F11(1,3) = q1*(wirr(2)*q1 - wirr(3)*q0 - wirr(1)*q2) + (gyroDrift(2)/2 - wibb(2)/2 - (wirr(1)*(2*q0*q3 - 2*q1*q2))/2 + (wirr(3)*(2*q0*q1 + 2*q2*q3))/2 + (wirr(2)*(q0^2 - q1^2 + q2^2 - q3^2))/2)...
           + q2*(wirr(1)*q1 + wirr(3)*q3 + wirr(2)*q2) + q3*(wirr(1)*q0 + wirr(2)*q3 - wirr(3)*q2);
F11(1,4) = q1*(wirr(2)*q0 + wirr(3)*q1 - wirr(1)*q3) + q2*(-wirr(1)*q0 + wirr(3)*q2 - wirr(2)*q3) + q3*(wirr(1)*q1 + wirr(2)*q2 + wirr(3)*q3)...
           + (gyroDrift(3)/2 - wibb(3)/2 + (wirr(1)*(2*q0*q2 + 2*q1*q3))/2 - (wirr(2)*(2*q0*q1 - 2*q2*q3))/2 + (wirr(3)*(q0^2 - q1^2 - q2^2 + q3^2))/2);
F11(2,1) = q3*(-wirr(1)*q3 + wirr(3)*q1 + wirr(2)*q0) - q2*(wirr(1)*q2 - wirr(2)*q1 + wirr(3)*q0) - q0*(wirr(2)*q3 - wirr(3)*q2 + wirr(1)*q0)...
           - (gyroDrift(1)/2 - wibb(1)/2 + (wirr(2)*(2*q0*q3 + 2*q1*q2))/2 - (wirr(3)*(2*q0*q2 - 2*q1*q3))/2 + (wirr(1)*(q0^2 + q1^2 - q2^2 - q3^2))/2);
F11(2,2) = q3*(wirr(1)*q2 + wirr(3)*q0 - wirr(2)*q1) - q2*(wirr(1)*q3 - wirr(2)*q0 - wirr(3)*q1) - q0*(wirr(2)*q2 + wirr(3)*q3 + wirr(1)*q1);
F11(2,3) = q3*(wirr(1)*q1 + wirr(3)*q3 + wirr(2)*q2) - q2*(wirr(1)*q0 + wirr(2)*q3 - wirr(3)*q2) - q0*(wirr(2)*q1 - wirr(3)*q0 - wirr(1)*q2)...
           - (gyroDrift(3)/2 - wibb(3)/2 + (wirr(1)*(2*q0*q2 + 2*q1*q3))/2 - (wirr(2)*(2*q0*q1 - 2*q2*q3))/2 + (wirr(3)*(q0^2 - q1^2 - q2^2 + q3^2))/2);
F11(2,4) = (gyroDrift(2)/2 - wibb(2)/2 - (wirr(1)*(2*q0*q3 - 2*q1*q2))/2 + (wirr(3)*(2*q0*q1 + 2*q2*q3))/2 + (wirr(2)*(q0^2 - q1^2 + q2^2 - q3^2))/2)...
           + q3*(-wirr(1)*q0 + wirr(3)*q2 - wirr(2)*q3) - q2*(wirr(1)*q1 + wirr(2)*q2 + wirr(3)*q3) - q0*(wirr(2)*q0 + wirr(3)*q1 - wirr(1)*q3);
F11(3,1) = q1*(wirr(1)*q2 - wirr(2)*q1 + wirr(3)*q0) - q0*(-wirr(1)*q3 + wirr(3)*q1 + wirr(2)*q0) - q3*(wirr(2)*q3 - wirr(3)*q2 + wirr(1)*q0)...
           - (gyroDrift(2)/2 - wibb(2)/2 - (wirr(1)*(2*q0*q3 - 2*q1*q2))/2 + (wirr(3)*(2*q0*q1 + 2*q2*q3))/2 + (wirr(2)*(q0^2 - q1^2 + q2^2 - q3^2))/2);
F11(3,2) = (gyroDrift(3)/2 - wibb(3)/2 + (wirr(1)*(2*q0*q2 + 2*q1*q3))/2 - (wirr(2)*(2*q0*q1 - 2*q2*q3))/2 + (wirr(3)*(q0^2 - q1^2 - q2^2 + q3^2))/2)...
           + q1*(wirr(1)*q3 - wirr(2)*q0 - wirr(3)*q1) - q0*(wirr(1)*q2 + wirr(3)*q0 - wirr(2)*q1) - q3*(wirr(2)*q2 + wirr(3)*q3 + wirr(1)*q1);
F11(3,3) = q1*(wirr(1)*q0 + wirr(2)*q3 - wirr(3)*q2) - q0*(wirr(1)*q1 + wirr(3)*q3 + wirr(2)*q2) - q3*(wirr(2)*q1 - wirr(3)*q0 - wirr(1)*q2);
F11(3,4) = q1*(wirr(1)*q1 + wirr(2)*q2 + wirr(3)*q3) - q0*(-wirr(1)*q0 + wirr(3)*q2 - wirr(2)*q3) - q3*(wirr(2)*q0 + wirr(3)*q1 - wirr(1)*q3)...
           - (gyroDrift(1)/2 - wibb(1)/2 + (wirr(2)*(2*q0*q3 + 2*q1*q2))/2 - (wirr(3)*(2*q0*q2 - 2*q1*q3))/2 + (wirr(1)*(q0^2 + q1^2 - q2^2 - q3^2))/2);
F11(4,1) = q2*(wirr(2)*q3 - wirr(3)*q2 + wirr(1)*q0) - q1*(-wirr(1)*q3 + wirr(3)*q1 + wirr(2)*q0) - q0*(wirr(1)*q2 - wirr(2)*q1 + wirr(3)*q0)...
           - (gyroDrift(3)/2 - wibb(3)/2 + (wirr(1)*(2*q0*q2 + 2*q1*q3))/2 - (wirr(2)*(2*q0*q1 - 2*q2*q3))/2 + (wirr(3)*(q0^2 - q1^2 - q2^2 + q3^2))/2);
F11(4,2) = q2*(wirr(2)*q2 + wirr(3)*q3 + wirr(1)*q1) - q1*(wirr(1)*q2 + wirr(3)*q0 - wirr(2)*q1) - q0*(wirr(1)*q3 - wirr(2)*q0 - wirr(3)*q1)...
           - (gyroDrift(2)/2 - wibb(2)/2 - (wirr(1)*(2*q0*q3 - 2*q1*q2))/2 + (wirr(3)*(2*q0*q1 + 2*q2*q3))/2 + (wirr(2)*(q0^2 - q1^2 + q2^2 - q3^2))/2);
F11(4,3) = (gyroDrift(1)/2 - wibb(1)/2 + (wirr(2)*(2*q0*q3 + 2*q1*q2))/2 - (wirr(3)*(2*q0*q2 - 2*q1*q3))/2 + (wirr(1)*(q0^2 + q1^2 - q2^2 - q3^2))/2)...
           + q2*(wirr(2)*q1 - wirr(3)*q0 - wirr(1)*q2) - q1*(wirr(1)*q1 + wirr(3)*q3 + wirr(2)*q2) - q0*(wirr(1)*q0 + wirr(2)*q3 - wirr(3)*q2);
F11(4,4) = q2*(wirr(2)*q0 + wirr(3)*q1 - wirr(1)*q3) - q1*(-wirr(1)*q0 + wirr(3)*q2 - wirr(2)*q3) - q0*(wirr(1)*q1 + wirr(2)*q2 + wirr(3)*q3);

function H = GetH_augSINSZhiJie(Qrb,r,Qrb_last,r_last)
% syms_H = importdata('augZhijieSINS_QbbTbb_syms_H') ;

q0=Qrb(1);
q1=Qrb(2);
q2=Qrb(3);
q3=Qrb(4);
% q0_last=Qrb_last(1);
% q1_last=Qrb_last(2);
% q2_last=Qrb_last(3);
% q3_last=Qrb_last(4);
% rx = r(1);
% ry = r(2);
% rz = r(3);
% rx_last = r_last(1);
% ry_last = r_last(2);
% rz_last = r_last(3);

% %%%%%%%%  (a)  %%%%%%%%
% Ha = subs(syms_H);
% %%%%%%%%  (b)  %%%%%%%%
% sym_H11 = syms_H(1:4,1:4);
% sym_H14 = syms_H(1:4,17:20);
% sym_H21 = syms_H(5:7,1:4);
% sym_H22 = syms_H(5:7,5:7);
% sym_H25 = syms_H(5:7,21:23);
% sH11 = subs(sym_H11);
% sH14 = subs(sym_H14);
% sH21 = subs(sym_H21);
% sH22 = subs(sym_H22);
% sH25 = subs(sym_H25);
% 
% Hb = [  sH11 zeros(4,3) zeros(4,9) sH14 zeros(4,3) ;
%         sH21    sH22    zeros(3,9) zeros(3,4)  sH25 ];
%%%%%%%%  (c)  %%%%%%%%
Qrb_last_niv = [Qrb_last(1);-Qrb_last(2:4)];
v0 = Qrb_last_niv(1) ;
v = Qrb_last_niv(2:4);
H11 = [ v0  -v' ;
        v   v0*eye(3)+getCrossMatrix(v) ];
p0 = Qrb(1);
p = Qrb(2:4);
H14 = [ p0 p';
        p -p0*eye(3)+getCrossMatrix(p) ];
Crb = FQtoCnb(Qrb);    
H22 =  Crb ;
H25 = -Crb ;

dr = r-r_last ;
H21(1:3,1) = [ 2*q0 2*q3 -2*q2; -2*q3 2*q0 2*q1 ; 2*q2 -2*q1 2*q0]*dr ;
H21(1:3,2) = [ 2*q1 2*q2 2*q3; 2*q2 -2*q1 2*q0 ; 2*q3 -2*q0 -2*q1]*dr ;
H21(1:3,3) = [ -2*q2 2*q1 -2*q0; 2*q1 2*q2 2*q3 ; 2*q0 2*q3 -2*q2]*dr ;
H21(1:3,4) = [ -2*q3 2*q0 2*q1; -2*q0 -2*q3 2*q2 ; 2*q1 2*q2 2*q3]*dr ;

Hc = [  H11 zeros(4,3) zeros(4,9) H14 zeros(4,3) ;
        H21    H22    zeros(3,9) zeros(3,4)  H25 ];
H = Hc;    
% H11-sH11
% H14-sH14
% H22-sH22
% H25-sH25
% H21-sH21

    
function Zpre = hfun_QbbTbb_ZhijieX(X)
%% 量测量：Qbb,Tbb
%   状态量：SINS直接模型
qk = X(1:4);
qk = qk/norm(qk);
qk_last = X(17:20);
qk_last = qk_last/norm(qk_last);
qk_last_niv = [qk_last(1);-qk_last(2:4)];
Qbb = QuaternionMultiply(qk_last_niv,qk) ;
Qbb = Qbb/norm(Qbb);
rk = X(5:7) ;
rk_last = X(21:23);
Tbb = FQtoCnb(qk)*(rk-rk_last) ;
Zpre = [Qbb;Tbb];
