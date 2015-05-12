%% DRRbb标定陀螺常值漂移法：两个时刻的Rbb相乘
function [gyroDriftEsmDRR_new,P_gyroDRbbKFDRR_new] = gyroDRRUKF(R_INS_last,R_VNS_last,R_INS,R_VNS,T,gyroDriftEsmDRR_old,P_gyroDRbbKFDRR_old,Q_gyroDRbbKFDRR,R_gyroDRbbKFDRR,Wrbb_last,Wrbb)

Z_gyroDRbbDRR = FCnbtoQ(R_INS_last*R_VNS*R_VNS_last'*R_INS') ;   % 量测量
%   Z_gyroDRbbDRR = FCnbtoQ(R_INS_last*R_VNS_last*R_VNS'*R_INS') ;   % 量测量

Fai = eye(3);
[gyroDriftEsmDRR_new,P_gyroDRbbKFDRR_new] = UKF(gyroDriftEsmDRR_old,P_gyroDRbbKFDRR_old,Q_gyroDRbbKFDRR,R_gyroDRbbKFDRR,Z_gyroDRbbDRR,Wrbb_last,Wrbb,Fai,3,-3,1,T);


function Zpre_gyroDRbbDRR = hfun_DRRbbDRR(gyroDrift,T,Wrbb_last,Wrbb)
%% 由状态计算量测
% Wrbb = [0 0 0];
OWrbb = GetdCnbdtK(Wrbb) ;   % Wrbb对应的姿态转移矩阵变化率
OWrbb_last = GetdCnbdtK(Wrbb_last) ;   % Wrbb对应的姿态转移矩阵变化率
OgyroDrift = GetdCnbdtK(gyroDrift) ; % 陀螺常值漂移对应的姿态转移矩阵变化率

Rbb_INS_last = eye(3)-(OgyroDrift+OWrbb_last)*T ;
Rbb_INS = eye(3)-(OgyroDrift+OWrbb)*T ;
Rbb_VNS_last = eye(3) - OWrbb_last*T ;
Rbb_VNS = eye(3) - OWrbb*T ;
DRR = Rbb_INS_last * Rbb_VNS * Rbb_VNS_last' * Rbb_INS' ;
%   DRR = Rbb_INS_last * Rbb_VNS_last * Rbb_VNS' * Rbb_INS' ;
Zpre_gyroDRbbDRR = FCnbtoQ(DRR);

function CnbV = GetdCnbdtK(Wrbb)
%% 由角速率计算姿态转移矩阵变化率
CnbV = [        0       -Wrbb(3)    Wrbb(2)
            Wrbb(3)         0       -Wrbb(1)
            -Wrbb(2)    Wrbb(1)        0   ];

function [Xe,Pe]=UKF(X0,P0,Q,R,Z,Wrbb_last,Wrbb,Fai,nn,mn,tao,T)
% station：
% Fai：
% nn： 状态维数
% mn
% tao：

if(det(P0)>0)
    U=chol(P0);
else
    U=chol(P0+eye(nn,nn)*1e-3);
end


% cakculate sigma point
Vp=zeros(nn,nn);
Vn=zeros(nn,nn);

V0=X0;
for n=1:nn
    Vp(:,n)=V0+sqrt(nn+tao)*U(n,:)';
end
for n=1:nn
    Vn(:,n)=V0-sqrt(nn+tao)*U(n,:)';
end

% Time update
Xp=zeros(nn,nn);
Xn=zeros(nn,nn);

Xm=Fai*V0;
for n=1:nn
    tempx=Fai*Vp(:,n);
    Xp(:,n)=tempx;
end
for n=1:nn
    tempx=Fai*Vn(:,n);
    Xn(:,n)=tempx;
end

Xall = [Xp;Xn];
stdXall = std(Xall,0,2) ;

w0=tao/(nn+tao);
X10=w0*Xm;
for n=1:nn
    wp=1/(2*(nn+tao));
    X10=X10+Xp(:,n)*wp;
end
for n=1:nn
    wn=1/(2*(nn+tao));
    X10=X10+Xn(:,n)*wn;
end

P10=(Xm-X10)*(Xm-X10)'*w0+Q;
for n=1:nn
    P10=P10+(Xp(:,n)-X10)*(Xp(:,n)-X10)'*wp;
end
for n=1:nn
    P10=P10+(Xn(:,n)-X10)*(Xn(:,n)-X10)'*wn;
end


yp=zeros(4,nn);
yn=zeros(4,nn);
ym=hfun_DRRbbDRR(Xm,T,Wrbb_last,Wrbb) ;
Z1=w0*ym;
for n=1:nn
    yp(:,n)=hfun_DRRbbDRR(Xp(:,n),T,Wrbb_last,Wrbb) ;
    Z1=Z1+yp(:,n)*wp;
end
for n=1:nn
    yn(:,n)=hfun_DRRbbDRR(Xn(:,n),T,Wrbb_last,Wrbb) ;
    Z1=Z1+yn(:,n)*wn;
end


% Measurement update equations
Pyy=w0*(ym-Z1)*(ym-Z1)'+R;
for n=1:nn
    Pyy=Pyy+(yp(:,n)-Z1)*(yp(:,n)-Z1)'*wp;
end
for n=1:nn
    Pyy=Pyy+(yn(:,n)-Z1)*(yn(:,n)-Z1)'*wn;
end
Pxy=w0*(Xm-X10)*(ym-Z1)';
for n=1:nn
    Pxy=Pxy+(Xp(:,n)-X10)*(yp(:,n)-Z1)'*wp;
end
for n=1:nn
    Pxy=Pxy+(Xn(:,n)-X10)*(yn(:,n)-Z1)'*wn;
end
Kk=Pxy/Pyy;
Xe=X10+Kk*(Z-Z1);
Pe=P10-Kk*Pyy*Kk';