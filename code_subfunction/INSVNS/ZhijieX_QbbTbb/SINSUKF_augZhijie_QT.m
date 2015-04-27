%% UKF_augZhijie_QT 滤波
%   增广直接状态模型，Qbb和Tbb为量测，UKF滤波方法
%   16+7=23维：X = (q,r,v,gyro,acc,q_last,r_last)

function [X_new,P_new,Xpre,X_correct,Zpre,Z,Zerror] = SINSUKF_augZhijie_QT(Rbb_VNS,Tbb_VNS,T,X_last,X_SINSPre,P_last,Q,R,wirr,gr,wibb,fb,isSINSpre)

Qbb = FCnbtoQ(Rbb_VNS);
Z = [Qbb;Tbb_VNS];

[X_new,P_new,Xpre,X_correct,Zpre] = UKF(X_last,X_SINSPre,P_last,Q,R,Z,wirr,gr,wibb,fb,T,isSINSpre);
Zerror = Z-Zpre;

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


function [Xe,Pe,X10,X_correct,Z1]=UKF(X0,X_SINSPre,P0,Q,R,Z,wirr,gr,wibb,fb,T,isSINSpre)
% station：

% tao：

% %% 纯状态递推
% % 
% Xpre  = updateX_ZhiJie(X0,wirr,gr,wibb,fb,T) ;
% % 前10位取X_SINSPre的
% Xe=Xpre;
% 
% if isSINSpre==1     % 采用SINS解算结果作为状态一步预测
%     Xe(1:10)=X_SINSPre(1:10);
%     Xe(17:23)=X_SINSPre(17:23);
% end
% X10=Xe;
% X_correct=zeros(size(X10));
% Z1=zeros(7,1);
% Pe=P0;
% return;

nn=length(X0);
tao = 3-nn ;
if(det(P0)>0)
    U=chol(P0);
else
    P0=ones(23,23)*1e-8;
    U=chol(P0+eye(nn,nn)*1e-3);
end

% cakculate sigma point
Vp=zeros(nn,nn);
Vn=zeros(nn,nn);

V0=X0;
for n=1:nn
    Vp(:,n)=V0+sqrt(nn+tao)*U(n,:)';
    Vp(:,n)=NormXQ(Vp(:,n)) ;
    Vp(1:4,n)=V0(1:4);
    Vp(17:23,n)=V0(17:23);
end
for n=1:nn
    Vn(:,n)=V0-sqrt(nn+tao)*U(n,:)';
    Vn(:,n)=NormXQ(Vn(:,n)) ;
    Vn(1:4,n)=V0(1:4);
    Vn(17:23,n)=V0(17:23);
end

% Time update
Xp=zeros(nn,nn);
Xn=zeros(nn,nn);

Xpre  = updateX_ZhiJie(X0,wirr,gr,wibb,fb,T) ;

% Xpre=X0+dXdt_ZhiJie(X0,wirr,gr,wibb,fb)*T;
Xm=Xpre;
% Xm=NormXQ(Xm) ;

if isSINSpre==1     % 采用SINS解算结果作为状态一步预测
    Xm(1:10)=X_SINSPre(1:10);
    Xm(17:23)=X_SINSPre(17:23);
end
for n=1:nn
%     tempx=Vp(:,n)+dXdt_ZhiJie(Vp(:,n),wirr,gr,wibb,fb)*T;
%     tempx=NormXQ(tempx) ;
    tempx  = updateX_ZhiJie(Vp(:,n),wirr,gr,wibb,fb,T) ;
    Xp(:,n)=tempx;                                               
%     Xp(:,n)=Xm;
end
for n=1:nn
%     tempx=Vn(:,n)+dXdt_ZhiJie(Vn(:,n),wirr,gr,wibb,fb)*T;
%     tempx=NormXQ(tempx) ;
    tempx  = updateX_ZhiJie(Vn(:,n),wirr,gr,wibb,fb,T) ;
    Xn(:,n)=tempx;                                               
%     Xn(:,n)=Xm;
end

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
X10=NormXQ(X10) ;

P10=(Xm-X10)*(Xm-X10)'*w0+Q;
for n=1:nn
    P10=P10+(Xp(:,n)-X10)*(Xp(:,n)-X10)'*wp+Q;
end
for n=1:nn
    P10=P10+(Xn(:,n)-X10)*(Xn(:,n)-X10)'*wn+Q;
end

yp=zeros(7,nn);
yn=zeros(7,nn);
ym=hfun_QbbTbb_ZhijieX(Xm) ;
Z1=w0*ym;
for n=1:nn
    yp(:,n)=hfun_QbbTbb_ZhijieX(Xp(:,n)) ;
    Z1=Z1+yp(:,n)*wp;
end
for n=1:nn
    yn(:,n)=hfun_QbbTbb_ZhijieX(Xn(:,n)) ;
    Z1=Z1+yn(:,n)*wn;
end
Z1(1:4)=Z1(1:4)/norm(Z1(1:4));

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
Xe=NormXQ(Xe) ;
Pe=P10-Kk*Pyy*Kk';

X_correct = Kk*(Z-Z1);
%Xe=X10;
% Xe=Xm;

function X=NormXQ(X)
X(1:4)=X(1:4)/norm(X(1:4));
X(17:20)=X(17:20)/norm(X(17:20));