%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          buaa xyz 
%                          2014 10 9
%               SINS误差状态模型，观测量：特征点三维相机系坐标
%                         状态更新函数
%  不考虑视觉常值误差：  X=[dat dv dr gyroDrift accDrift  ] 
%  考虑视觉常值误差：  X=[dat dv dr gyroDrift accDrift beita  dTcd_c ] beita是左右相机间角度误差
% UKF   % 状态方程线性，量测方程非线性   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ newX,newP,X_correct,X_pre ] = updateX_SINSerr_FPc...
        ( X0,P0,Q,R,Wirr,fb,cycleTvns,Crb,position,FP0,FP1,FPpixel_2time,Rbc,Tcb_c,calibData,...
          Crb_SINSpre,position_SINSpre,isDebudMode,calZMethod )
% Z：下一时刻的特征点相机系坐标
% FP0：当前时刻的特征点相机系坐标

[F_INS,G_INS] = GetF_StatusErrorSINS(Crb',Wirr,fb);  % 注意取滤波周期，而不是惯导解算周期
F_k = [F_INS zeros(15,6);zeros(6,21)];
% G_k = [G_INS;zeros(6,6)];
Fai_k = FtoFai(F_k,cycleTvns);
x_n = 21 ;   % 输入系统的状态维数是21，多出的部分是视觉常漂（为了统一在一个程序中），此方法中不要。此方法中实际有效的维数是15
Z = FP1 ;        
switch calZMethod.Z_method
    case {'FPc_UKF','FPc_VnsErr_UKF'}
        Z_method = calZMethod.Z_method ;
        [newX,newP,X_correct,X_pre,Zinteg_error,Zinteg_pre] = update_FPc_UKF...
            (Z_method,X0,P0,Q,R,Z,Fai_k,FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c,FPpixel_2time,calibData,x_n,-8) ;
end
if isfield(isDebudMode,'onlyStateFcn') && isDebudMode.onlyStateFcn==1
    % 纯状态递推，不作量测的修正
    newX = X_pre;
end


function [newX,newP,X_correct,X_pre,Zinteg_error,Zinteg_pre] = update_FPc_UKF...
    (Z_method,X0,P0,Q,R,Z,Fai,FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c,FPpixel_2time,calibData,x_n,tao)
% X0（前一时刻状态）,P0（前一时刻协方差）,Q（Q阵）,R（R阵）,Z（量测）,Fai（状态转移矩阵）,FP0（前一时刻特征点相机系位置）
%  UKF
FP_n = length(FP0);   % 特征点个数*3
R = R*eye(FP_n);

if(det(P0)>0)
    U=chol(P0);
else
    U=chol(P0+eye(x_n,x_n)*1e-3);
end

% cakculate sigma point
Vp=zeros(x_n,x_n);
Vn=zeros(x_n,x_n);

V0=X0;
for n=1:x_n
    Vp(:,n)=V0+sqrt(x_n+tao)*U(n,:)';
end
for n=1:x_n
    Vn(:,n)=V0-sqrt(x_n+tao)*U(n,:)';
end

% Time update
Xp=zeros(x_n,x_n);
Xn=zeros(x_n,x_n);

Xm=Fai*V0;
for n=1:x_n
    tempx=Fai*Vp(:,n);
    Xp(:,n)=tempx;
end
for n=1:x_n
    tempx=Fai*Vn(:,n);
    Xn(:,n)=tempx;
end

w0=tao/(x_n+tao);
X10=w0*Xm;
for n=1:x_n
    wp=1/(2*(x_n+tao));
    X10=X10+Xp(:,n)*wp;
end
for n=1:x_n
    wn=1/(2*(x_n+tao));
    X10=X10+Xn(:,n)*wn;
end
% 得到 X10

P10=(Xm-X10)*(Xm-X10)'*w0+Q;
for n=1:x_n
    P10=P10+(Xp(:,n)-X10)*(Xp(:,n)-X10)'*wp+Q;
end
for n=1:x_n
    P10=P10+(Xn(:,n)-X10)*(Xn(:,n)-X10)'*wn+Q;
end
% 得到 P10

yp=zeros(FP_n,x_n);
yn=zeros(FP_n,x_n);
% ym=hfun1(Xm,statione,T0,Cbne,ICen,ICbn,Cbc,Re,msize,P0);
ym = hfun_FPcpre(  Z_method,Xm,FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c,FPpixel_2time,calibData ) ;
Z1=w0*ym;
for n=1:x_n
%     yp(:,n)=hfun1(Xp(:,n),statione,T0,Cbne,ICen,ICbn,Cbc,Re,msize,P0);
    yp(:,n) = hfun_FPcpre(  Z_method,Xp(:,n),FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c,FPpixel_2time,calibData ) ;
    Z1=Z1+yp(:,n)*wp;
end
for n=1:x_n
%     yn(:,n)=hfun1(Xn(:,n),statione,T0,Cbne,ICen,ICbn,Cbc,Re,msize,P0);
    yn(:,n) = hfun_FPcpre(  Z_method,Xn(:,n),FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c,FPpixel_2time,calibData ) ;
    Z1=Z1+yn(:,n)*wn;
end

% Measurement update equations
Pyy=w0*(ym-Z1)*(ym-Z1)'+R;
for n=1:x_n
    Pyy=Pyy+(yp(:,n)-Z1)*(yp(:,n)-Z1)'*wp;
end
for n=1:x_n
    Pyy=Pyy+(yn(:,n)-Z1)*(yn(:,n)-Z1)'*wn;
end
Pxy=w0*(Xm-X10)*(ym-Z1)';
for n=1:x_n
    Pxy=Pxy+(Xp(:,n)-X10)*(yp(:,n)-Z1)'*wp;
end
for n=1:x_n
    Pxy=Pxy+(Xn(:,n)-X10)*(yn(:,n)-Z1)'*wn;
end
Kk=Pxy/Pyy;
newX=X10+Kk*(Z-Z1);
newP=P10-Kk*Pyy*Kk';

% 输出一些中间值
X_pre = X10 ;
X_correct = Kk*(Z-Z1) ;
Zinteg_pre = Z1 ;
Zinteg_error = Z-Z1 ;

%% 运动估计函数 ：计算下一时刻特征点相机系坐标
function FP1_pre = hfun_FPcpre(  Z_method,X,FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c,FPpixel_2time,calibData )
switch Z_method
    case 'FPc_UKF'
        FP1_pre = hfun_nonVnsErr_FPcpre(  X,FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c ) ;
    case 'FPc_VnsErr_UKF'
%         dbstop in hfun_VnsErr_FPcpre
        FP1_pre = hfun_VnsErr_FPcpre(  X,FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c,FPpixel_2time,calibData ) ;
end


%% 运动估计函数（不考虑双目外参误差）：计算下一时刻特征点相机系坐标
function FP1_pre = hfun_nonVnsErr_FPcpre(  X,FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c )
% (IX1,statione,T0,Cbne,ICen,ICbn,Cbc,Re,msize,P0)

% FP0：上一时刻的相机系坐标
% Crb_SINSpre、position_SINSpre：惯性预测的 姿态矩阵 和 位置
% Crb、position：上一时刻估计的 姿态矩阵 和 位置
% Rbc：本体系到相机系旋转矩阵
% Tcb_c：本体系原点在相机系坐标
% X：状态量（用于补偿预测的位置姿态）

% FP1_pre ：根据 上一时刻相机坐标FP0、惯导预测的 Crb_SINSpre、上一时刻组合估计的位置姿态 -> 预测的当前时刻特征点相机系坐标

Crc = FCbn(X(1:3)); % X(1:3)是平台坐标系p（SINS计算用r），转动到真实r坐标系的角度
Crb_pre = Crb_SINSpre*Crc ;     % 在 Crb_SINSpre 中 补偿 平台失准角
Rbb_INS = Crb_pre*Crb' ;

position_pre = position_SINSpre - X(7:9);  % 在 position_SINSpre 中 补偿 位置误差
Tbb_INS = Crb*(position_pre-position) ; % 注意：在上一时刻分解
Rcc_pre = Rbc*Rbb_INS*Rbc' ;
Tcc_pre = Rbc* Tbb_INS+(eye(3)-Rcc_pre')*Tcb_c  ;   % 注意：在上一时刻分解

FP_n  =length(FP0);
match_n = FP_n/3 ;
FP1_pre = zeros(FP_n,1);
for i=1:match_n
    FP1_pre(3*i-2:3*i,1) = Rcc_pre*(FP0(3*i-2:3*i,1)-Tcc_pre) ;
end

%% 运动估计函数（考虑双目外参误差）：计算下一时刻特征点相机系坐标
function FP1_pre = hfun_VnsErr_FPcpre(  X,FP0,Crb,position,Crb_SINSpre,position_SINSpre,Rbc,Tcb_c,FPpixel_2time,calibData )
% (IX1,statione,T0,Cbne,ICen,ICbn,Cbc,Re,msize,P0)

% FP0：上一时刻的相机系坐标
% Crb_SINSpre、position_SINSpre：惯性预测的 姿态矩阵 和 位置
% Crb、position：上一时刻估计的 姿态矩阵 和 位置
% Rbc：本体系到相机系旋转矩阵
% Tcb_c：本体系原点在相机系坐标
% X：状态量（用于补偿预测的位置姿态）
% FPpixel_2time：前后两个时刻左右相机的特征点像素坐标
% c：左相机系，d：右相机系。Cd2c_inexact：d到c旋转矩阵（含误差）。Tcd_c_inexact：d原点在c的坐标（含误差）
% FP1_pre ：根据 上一时刻相机坐标FP0、惯导预测的 Crb_SINSpre、上一时刻组合估计的位置姿态 -> 预测的当前时刻特征点相机系坐标

% 提取双目标定参数：fc_left,fc_right,Cd2c_calib,Tcd_c_calib
fc_left = (calibData.fc_left(1)+calibData.fc_left(2))/2 ;
fc_right = (calibData.fc_right(1)+calibData.fc_right(2))/2 ;
Cd2c_calib = FCbn(calibData.om) ;   % om是从左到右
Tcd_c_calib = -Cd2c_calib*calibData.T/1000 ;        % T是 Tdc_d，单位mm

Crc = FCbn(X(1:3)); % X(1:3)是平台坐标系p（SINS计算用r），转动到真实r坐标系的角度
Crb_pre = Crb_SINSpre*Crc ;     % 在 Crb_SINSpre 中 补偿 平台失准角
Rbb_INS = Crb_pre*Crb' ;

position_pre = position_SINSpre - X(7:9);  % 在 position_SINSpre 中 补偿 位置误差
Tbb_INS = Crb*(position_pre-position) ; % 注意：在上一时刻分解
Rcc_pre = Rbc*Rbb_INS*Rbc' ;
Tcc_pre = Rbc* Tbb_INS+(eye(3)-Rcc_pre')*Tcb_c  ;   % 注意：在上一时刻分解

FP_n  =length(FP0);
match_n = FP_n/3 ;
FP1_pre = zeros(FP_n,1);
beita = X(16:18);
dTcd_c = X(19:21);
da_current_save = zeros(match_n,1);
db_current_save = zeros(match_n,1);
for i=1:match_n
    % 前一时刻的三维重建比例因子误差
    L_pixel_0 = FPpixel_2time.FPpixel_leftCurrent(i,:) ;
    R_pixel_0 = FPpixel_2time.FPpixel_rightCurrent(i,:) ;
    [da_current db_current] = cal_da_db_reconstruction( L_pixel_0,R_pixel_0,fc_left,fc_right,Cd2c_calib,Tcd_c_calib,beita,dTcd_c ) ;
    % 后一时刻的三维重建比例因子误差
    L_pixel_1 = FPpixel_2time.FPpixel_leftNext(i,:) ;
    R_pixel_1 = FPpixel_2time.FPpixel_rightNext(i,:) ;
    [da_next db_next] = cal_da_db_reconstruction( L_pixel_1,R_pixel_1,fc_left,fc_right,Cd2c_calib,Tcd_c_calib,beita,dTcd_c ) ;
    FP1_pre(3*i-2:3*i,1) = da_next*Rcc_pre*(FP0(3*i-2:3*i,1)/da_current-Tcc_pre) ;
    
    test_f = FP1_pre(3*i-2:3*i,1) ;
    if isnan(test_f(1))||isnan(test_f(2))||isnan(test_f(3))
       disp('wrong') 
    end
    da_current_save(i) = da_current;
    db_current_save(i) = db_current;
end
disp('')

%% 计算双目外参导致的三维重建比例因子误差
function [ da,db ] = cal_da_db_reconstruction( L_pixel,R_pixel,fc_left,fc_right,Cd2c_inexact,Tcd_c_inexact,beita,dTcd_c )
% L_pixel：左成像点象素坐标。R_pixel：右成像点象素坐标。fc：象素焦距
% c：左相机系，d：右相机系。Cd2c_inexact：d到c旋转矩阵（含误差）。Tcd_c_inexact：d原点在c的坐标（含误差）
% beita：Cd2c的误差角。dTcd_c：Tcd_c的误差

% 计算含误差的比例因子： a_du_inexact,b_dv_inexact
[ a_du_inexact,b_dv_inexact ] = cal_a_b_reconstruction( L_pixel,R_pixel,fc_left,fc_right,Cd2c_inexact,Tcd_c_inexact ) ;
% 计算补偿后的（认为不含误差）的比例因子： 
Cd2c_err = FCbn(beita)' ;
Cd2c = Cd2c_err*Cd2c_inexact ;
Tcd_c = Tcd_c_inexact-dTcd_c ;
[ a_du,b_dv ] = cal_a_b_reconstruction( L_pixel,R_pixel,fc_left,fc_right,Cd2c,Tcd_c ) ;
% 计算比例因子误差
da = a_du_inexact/a_du ;
db = b_dv_inexact/b_dv ;

%% 计算三维重建比例因子
function [ a_du,b_dv ] = cal_a_b_reconstruction( L_pixel,R_pixel,fc_left,fc_right,Cd2c,Tcd_c )
% L_pixel：左成像点象素坐标。R_pixel：右成像点象素坐标。fc：象素焦距
% c：左相机系，d：右相机系。Cd2c：d到c旋转矩阵。Tcd_c：d原点在c的坐标
% 输出：a_du=a*du，a是比例因子=|OP|/|OL|，du是横向象元尺寸
%       b_du=b*du，b是比例因子=|OrP|/|OrR|，dv是纵向象元尺寸

% 可解得2组 a 和 b ，取均值
L = [ L_pixel(1);L_pixel(2);fc_left ];
R = Cd2c*[ R_pixel(1);R_pixel(2);fc_right ];
% 第1组：第1行+第2行
M1_2 = [  L(1) -R(1)
        L(2) -R(2)    ];
T1_2 = [ Tcd_c(1);Tcd_c(2) ];
Y1_2 = M1_2\T1_2 ;
% 第2组：第1行+第3行
M1_3 = [  L(1) -R(1)
        L(3) -R(3)    ];
T1_3 = [ Tcd_c(1);Tcd_c(3) ];
Y1_3 = M1_3\T1_3 ;
% 取均值
Y = (Y1_2+Y1_3)/2 ;
a_du = Y(1) ;
b_dv = Y(2) ;

%% 根据 相机系三维特征点坐标 投影得到 二维像素坐标
function TouYing()