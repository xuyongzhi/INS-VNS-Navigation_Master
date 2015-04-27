%% DRbb标定陀螺常值漂移法B：Rbb相乘
function [gyroDriftEsmB_new,P_gyroDRbbKFB_new] = gyroDRbbBUKF(R_INS,R_VNS,T,gyroDriftEsmB_old,P_gyroDRbbKFB_old,Q_gyroDRbbKFB,R_gyroDRbbKFB,Wrbb)

Z_gyroDRbbB = FCnbtoQ(R_INS*R_VNS') ;   % 量测量

Fai = eye(3);
[gyroDriftEsmB_new,P_gyroDRbbKFB_new] = UKF(gyroDriftEsmB_old,P_gyroDRbbKFB_old,Q_gyroDRbbKFB,R_gyroDRbbKFB,Z_gyroDRbbB,Wrbb,Fai,3,-3,1,T);


function Zpre_gyroDRbbB = hfun_DRbbB(gyroDrift,T,Wrbb)
%% 由状态计算量测
% Wrbb = [0 0 0];
OWrbb = GetCnbVelcity(Wrbb) ;   % Wrbb对应的姿态转移矩阵变化率
OgyroDrift = GetCnbVelcity(gyroDrift) ; % 陀螺常值漂移对应的姿态转移矩阵变化率
DR = eye(3)-OgyroDrift*T-(OWrbb^2+OgyroDrift*OWrbb)*T^2 ;   % RINS * RVNS'
Zpre_gyroDRbbB = FCnbtoQ(DR);

function CnbV = GetCnbVelcity(Wrbb)
%% 由角速率计算姿态转移矩阵变化率
CnbV = [        0       -Wrbb(3)    Wrbb(2)
            Wrbb(3)         0       -Wrbb(1)
            -Wrbb(2)    Wrbb(1)        0   ];

function [Xe,Pe]=UKF(X0,P0,Q,R,Z,Wrbb,Fai,nn,mn,tao,T)
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
ym=hfun_DRbbB(Xm,T,Wrbb) ;
Z1=w0*ym;
for n=1:nn
    yp(:,n)=hfun_DRbbB(Xp(:,n),T,Wrbb) ;
    Z1=Z1+yp(:,n)*wp;
end
for n=1:nn
    yn(:,n)=hfun_DRbbB(Xn(:,n),T,Wrbb) ;
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