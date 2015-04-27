%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.3
% 作者：xyz
% 功能：惯性/视觉组合函数：
%% 输入
%   integMethod（组合导航方法）：simple_dRdT,
%   visualInputData（视觉输入），所需成员：VisualRT,frequency
%   imuInputData（惯导输入），所需成员：wib_INSm,f_INSm,imu_fre
%% 输出：INS_VNS_NavResult
%   按ResultDisplay特定格式存储的当结果，不同导航方法的结果仅方案名不同
%% 程序功能说明 
% 导航解算规则：
%       IMU数据频率远大于VO，以纯惯导为基础，按频率进行周期性的组合
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [INS_VNS_NavResult,check,recordStr,NavFilterParameter] = main_INS_VNS_Q(integMethod,visualInputData,imuInputData,trueTrace,NavFilterParameter,isTrueX0)
format long
if ~exist('integMethod','var')
    % 独立运行
    clc
    clear all 
    close all
    %% 参数选择1 ： 再此更改所需添加的数据名称：对应相关的参数设置方案    
    % load([pwd,'\gyro_norm.mat']);  % 只有陀螺随机噪声
     load([pwd,'\10s.mat']);
    % load([pwd,'\SimGenRT-R_Std0.002rad.mat']);                % 简称 R-20
    % load([pwd,'\SimGenRT-R_Std0.002rad_Const0.0002rad.mat']);
    % load([pwd,'\SimGenRT-T_Std0.02m.mat']);                   % 简称 T-20
    % load([pwd,'\SimGenRT-T_Std0.02rad_Const0.002rad.mat']);
    
    % load([pwd,'\ForwardVelNonIMUNoise.mat'])
    % load([pwd,'\trueVision40m.mat']);
    % load([pwd,'\visonScence40m.mat']);
    % load([pwd,'\仿真生成RT-静止-2S-陀螺噪声-Tbb常值.mat']);
    %% 参数选择2 ： 组合方法
    integMethod =  'simple_dRdT';
   % integMethod =  'augment_dRdT';
    isAlone = 1;
else
    isAlone = 0;
end

format long
disp('函数 INS_VNS_ZdRdT 开始运行')
addpath([pwd,'\sub_code']);
oldfolder=cd([GetUpperPath(pwd),'\commonFcn']);
add_CommonFcn_ToPath;
cd(oldfolder);
addpath([GetUpperPath(pwd),'\ResultDisplay']);

%% 导入数据

% (1) 导入纯视觉导航仿真解算的的中间结果，包括两个数据:Rbb[例3*3*127]、Tbb[例3*127]
VisualOut_RT=visualInputData.VisualRT;
RbbVision = VisualOut_RT.Rbb;
TbbVision = VisualOut_RT.Tbb;
frequency_VO = visualInputData.frequency;
% （2）IMU数据
wib_INSm = imuInputData.wib;
f_INSm = imuInputData.f;
imu_fre = imuInputData.frequency; % Hz

% 真实轨迹的参数
if ~exist('trueTrace','var')
    trueTrace = [];
end
resultPath = [pwd,'\navResult'];
if isdir(resultPath)
    delete([resultPath,'\*.*'])
else
   mkdir(resultPath) 
end
[planet,isKnowTrue,initialPosition_e,initialVelocity_r,initialAttitude_r,trueTraeFre,true_position,true_attitude,true_velocity,true_acc_r] = GetFromTrueTrace( trueTrace );
% true_position=-true_position;true_velocity=-true_velocity;initialVelocity_r=-initialVelocity_r;
%% 星体常数
if strcmp(planet,'m')
    moonConst = getMoonConst;   % 得到月球常数
    gp = moonConst.g0 ;     % 用于导航解算
    wip = moonConst.wim ;
    Rp = moonConst.Rm ;
    e = moonConst.e;
    gk1 = moonConst.gk1;
    gk2 = moonConst.gk2;
    disp('轨迹发生器：月球')
else
    earthConst = getEarthConst;   % 得到地球常数
    gp = earthConst.g0 ;     % 用于导航解算
    wip = earthConst.wie ;
    Rp = earthConst.Re ;
    e = earthConst.e;
    gk1 = earthConst.gk1;
    gk2 = earthConst.gk2;
    disp('轨迹发生器：地球')
end
%% 月球模型参数
Wipp=[0;0;wip];

%% sample period
validLenth_INS_VNS = GetValidLength([size(f_INSm,2),size(TbbVision,2)],[imu_fre,frequency_VO]); % 进行组合处理时，INS和VNS数据有效个数
imuNum = validLenth_INS_VNS(1); % 有效的IMU数据长度
%integnum = floor(imuNum/(imu_fre/frequency_VO))+1; % 组合导航数据个数 = 有效的VNS数据个数+1
integnum = validLenth_INS_VNS(2); % 组合导航数据个数 = 有效的VNS数据个数+1
integFre = frequency_VO;
cycleT_INS = 1/imu_fre;  % 捷联解算周期
cycleT_VNS = 1/frequency_VO;  % 视觉数据周期/滤波周期
OneIntegT_IMUtime = fix(imu_fre/integFre);  % 一个组合周内，IMU解算的次数
%% SINS导航参数
% 由IMU噪声确定滤波PQ初值的选取
    % 仿真时噪声已知，存储在imuInputData中，实验室噪声未知，手动输入 常值偏置 和 随机标准差
[pa,na,pg,ng,~] = GetIMUdrift( imuInputData,planet ) ; % pa(加计常值偏置),na（加计随机漂移）,pg(陀螺常值偏置),ng（陀螺随机漂移）
%初始位置误差 m 
dinit_pos = trueTrace.InitialPositionError;
%初始姿态误差 rad
dinit_att = trueTrace.InitialAttitudeError;

% 组合导航参数
INTGatt = zeros(3,integnum);  % 欧拉角姿态
INTGvel = zeros(3,integnum);  % 速度
INTGpos = zeros(3,integnum);  % 位置

% 捷联惯导解算导航参数
QrbSave = zeros(4,imuNum);      % 四元数记录
CrbSave = zeros(3,3,imuNum);    % 姿态矩阵记录

SINSatt = zeros(3,imuNum);  % 欧拉角姿态
SINSvel = zeros(3,imuNum);  % 速度
SINSposition = zeros(3,imuNum);  % 位置 米
SINSacc_r = zeros(3,imuNum);  % 加速度
SINSpositionition_d = zeros(3,imuNum);% 大地坐标系 经纬度

% SINSposition(:,1)=[1;1;0];
%% SINS初始条件
SINSpositionition_d(:,1) = initialPosition_e;  % 经度 纬度 高度
SINSatt(:,1) = initialAttitude_r;         % 初始姿态 sita ,gama ,fai （rad）

positionr = FJWtoZJ(SINSpositionition_d(:,1),planet);  %地固坐标系中的初始位置
positionr = positionr+dinit_pos ;   % 叠加初始位置误差
SINSpositionition_d(:,1) = FZJtoJW(positionr,planet);
Cen=FCen(SINSpositionition_d(1,1),SINSpositionition_d(2,1));       %calculate Cen

Cbn = FCbn(SINSatt(:,1));
Cbn = Cbn*FCbn(dinit_att);  % 叠加初始姿态误差
opintions.headingScope = 180;
SINSatt(:,1) = GetAttitude(Cbn','rad',opintions) ;

Cnb = Cbn';
Cer = Cen; % 世界坐标系相对于初始时刻地固系的旋转矩阵
Cre = Cer';
Crb = Cnb;
Cbr = Crb';
Wirr = Cer * Wipp;
SINSvel(:,1) = Cbr * initialVelocity_r;
INTGvel(:,1) = Cbr * initialVelocity_r;
INTGatt(:,1) = SINSatt(:,1);

% 根据初始姿态矩阵Crb计算初始姿态四元数
Qrb = FCnbtoQ(Crb);
QrbSave(:,1)  = Qrb ;
CrbSave(:,:,1) = Crb ;
%% 组合导航估计的误差
dangleEsm = zeros(3,integnum);          % 平台失准角估计值
dVelocityEsm = zeros(3,integnum);       % 速度误差估计值
dPositionEsm = zeros(3,integnum);       % 位置误差估计值
gyroDrift = zeros(3,integnum);          % 陀螺漂移估计值
accDrift = zeros(3,integnum);           % 加计漂移估计值

dangleEsmP = zeros(3,integnum);       	% 平台失准角估计均方误差
dVelocityEsmP = zeros(3,integnum);      % 速度误差估计均方误差
dPositionEsmP = zeros(3,integnum);      % 位置误差估计均方误差
gyroDriftP = zeros(3,integnum);         % 陀螺漂移估计均方误差
accDriftP = zeros(3,integnum);          % 加计漂移估计均方误差
% 中间参数
R_INS_save = zeros(3,3,integnum);
T_INS_save = zeros(3,integnum);
R_VNS_save = zeros(3,3,integnum);
T_VNS_save = zeros(3,integnum);

%% 2014.4.15 宁老师的想法：利用直觉Rbb直接估计陀螺常值漂移
%%%%%%%%%%%  A：相减法
gyroDriftEsmA = zeros(3,integnum);          % 陀螺漂移估计值
gyroDriftEsmAError = zeros(3,integnum);          % 陀螺漂移估计值误差
P_gyroDRbbKFA = zeros(3,3,integnum);
Q_gyroDRbbKFA = diag([1 1 1]*0);
R_gyroDRbbKFA = diag([1 1 1]*1e-7);
if isTrueX0==1
    gyroDriftEsmA(:,1) = pg ;
    P_gyroDRbbKFA(:,:,1) = diag([ (pg(1))^2+1e-12,(pg(2))^2+1e-12,(pg(3))^2+1e-12 ]);
else
    pgError0 = [0.1;0.1;0.1]*pi/180/3600 ;  % 陀螺常值漂移 状态分量初值误差
    gyroDriftEsmA(:,1)=  pg-pgError0 ;
    P_gyroDRbbKFA(:,:,1) = diag([ (pg(1))^2+1e-7,(pg(2))^2+1e-7,(pg(3))^2+1e-7 ]);
end
%%%%%%%%%%%  B：相乘法
gyroDriftEsmB = zeros(3,integnum);          % 陀螺漂移估计值
gyroDriftEsmBError = zeros(3,integnum);          % 陀螺漂移估计值误差
P_gyroDRbbUKFB = zeros(3,3,integnum);
Q_gyroDRbbUKFB = diag([1 1 1]*0);
R_gyroDRbbUKFB = diag([1 1 1 1]*1e-7);
if isTrueX0==1
    gyroDriftEsmB(:,1) = pg ;
    P_gyroDRbbUKFB(:,:,1) = diag([ (pg(1))^2+1e-8,(pg(2))^2+1e-8,(pg(3))^2+1e-6 ]);
else
    pgError0 = [0.1;0.1;0.1]*pi/180/3600 ;  % 陀螺常值漂移 状态分量初值误差
    gyroDriftEsmB(:,1)=  pg-pgError0 ;
    P_gyroDRbbUKFB(:,:,1) = diag([ (pg(1))^2+1e-7,(pg(2))^2+1e-7,(pg(3))^2+1e-7 ]);
end

%% 组合导航参数（随不同组合方法不同）
projectName = integMethod;  % 存储在结果中，绘图时显示
switch integMethod
    case 'simple_dRdT'  
        %% 简化的状态模型（不扩维），dRdT作为量测
            % X=[dangleEsm;dVel;dPos;gyroDrift;accDrift]，将上一时刻的状态估计值作为真值
%         projectName = 'simple_dRdT';    % 存储在结果中，绘图时显示
        XNum = 15;
        ZNum = 6; % 量测信息维数
        X = zeros(XNum,integnum);       % 状态向量
        P = zeros(XNum,XNum,integnum); % 滤波P阵s
        
        X_pre_error = zeros(XNum,integnum);       % 状态向量一步预测误差
        X_correct = zeros(XNum,integnum);       % 状态向量滤波修正值
        Z_Integ = zeros(ZNum,integnum);
        newInformation = zeros(ZNum,integnum);  % 新息
        if isTrueX0==1
            X(:,1) = [zeros(9,1);pg;pa];  
        else            
            pgError0 = [0.1;0.1;0.1]*pi/180/3600 ;  % 陀螺常值漂移 状态分量初值误差
            paError0 = [10;10;10]*gp/1e6  ;         % 加计常值漂移 状态分量初值误差
            X(:,1) = [zeros(9,1);pg-pgError0;pa-paError0]; 
        end
        [ P(:,:,1),Q_ini,R,NavFilterParameter ] = GetFilterParameter_simple_dRdT( pg,ng,pa,na,NavFilterParameter );
        
        waitbarTitle = 'simple\_dRdT组合导航计算';  
        dangleEsm(:,1) = X(1:3,1); 
        dVelocityEsm(:,1) = X(4:6,1);
        dPositionEsm(:,1) = X(7:9,1);  
        gyroDrift(:,1) = X(10:12,1) ;
        accDrift(:,1) = X(13:15,1) ;
        P0_diag = sqrt(diag(P(:,:,1))) ;  % P0阵对角元素
        dangleEsmP(:,1) = P0_diag(1:3);
        dVelocityEsmP(:,1) = P0_diag(4:6);
        dPositionEsmP(:,1) = P0_diag(7:9);
        gyroDriftP(:,1) = P0_diag(10:12);
        accDriftP(:,1) = P0_diag(13:15);
    case 'augment_ZhiJie_QT'
        for n=1:1  % 仅为了将代码叠加起来
            XNum = 23;
            ZNum = 7; % 量测信息维数
            X = zeros(XNum,integnum);       % 状态向量
            P = zeros(XNum,XNum,integnum); % 滤波P阵s
            
            X_pre_error = zeros(XNum,integnum);       % 状态向量一步预测误差
            X_correct = zeros(XNum,integnum);       % 状态向量滤波修正值
            Z_Integ = zeros(ZNum,integnum);
            newInformation = zeros(ZNum,integnum);  % 新息
    %        X(:,1) = [zeros(10,1);pg;pa;zeros(7,1)];
            [ P(:,:,1),Q_ini,R,NavFilterParameter ] = GetFilterParameter_augment_ZhiJie_QT( pg,ng,pa,na,NavFilterParameter ) ;
            waitbarTitle = 'augment_ZhiJie_QT 组合导航计算';
            
            gyroDrift(:,1) = X(11:13,1) ;
            accDrift(:,1) = X(14:16,1) ;
        end
    case 'augment_dRdT'
        %% 增广状态方程，dRdT为量测
        for n=1:1
            XNum = 21;
            ZNum = 6; % 量测信息维数
            X = zeros(XNum,integnum);       % 状态向量
            if isTrueX0==1
                X(:,1) = [zeros(9,1);pg;pa;zeros(6,1)];
            else
                pgError0 = [0.1;0.1;0.1]*pi/180/3600 ;  % 陀螺常值漂移 状态分量初值误差
                paError0 = [10;10;10]*gp/1e6   ;         % 加计常值漂移 状态分量初值误差
                X(:,1) = [zeros(9,1);pg-pgError0;pa-paError0;zeros(6,1)]; 
            end
            Z_Integ = zeros(ZNum,integnum);
            P = zeros(XNum,XNum,integnum); % 滤波P阵s
            [ P(:,:,1),Q_ini,R,NavFilterParameter ] = GetFilterParameter_augment_dRdT( pg,ng,pa,na,NavFilterParameter ) ;
           
            waitbarTitle = 'augment\_dRdT组合导航计算';
            dangleEsm(:,1) = X(1:3,1); 
            dVelocityEsm(:,1) = X(4:6,1);
            dPositionEsm(:,1) = X(7:9,1);   
            gyroDrift(:,1) = X(10:12,1) ;
            accDrift(:,1) = X(13:15,1) ;
            P0_diag = sqrt(diag(P(:,:,1))) ;  % P0阵对角元素
            dangleEsmP(:,1) = P0_diag(1:3);
            dVelocityEsmP(:,1) = P0_diag(4:6);
            dPositionEsmP(:,1) = P0_diag(7:9);
            gyroDriftP(:,1) = P0_diag(10:12);
            accDriftP(:,1) = P0_diag(13:15);
        end
end

%% 开始导航解算
% 记录上一滤波时刻的姿态和位置

waitbar_h=waitbar(0,waitbarTitle);
for t_imu = 1:imuNum-1
    if mod(t_imu,ceil((imuNum-1)/200))==0
        waitbar(t_imu/(imuNum-1))
    end
    %% 世界坐标系SINS导航解算
    Wrbb = wib_INSm(:,t_imu) - Crb * Wirr;
    % 角增量法解四元数微分方程（简化的）
    Qrb=Qrb+0.5*cycleT_INS*[      0    ,-Wrbb(1,1),-Wrbb(2,1),-Wrbb(3,1);
                            Wrbb(1,1),     0    , Wrbb(3,1),-Wrbb(2,1);
                            Wrbb(2,1),-Wrbb(3,1),     0    , Wrbb(1,1);
                            Wrbb(3,1), Wrbb(2,1),-Wrbb(1,1),     0    ]*Qrb;
    Qrb=Qrb/norm(Qrb);      % 单位化四元数
    % 四元数->方向余弦矩阵
    Crb = FQtoCnb(Qrb);
    Cbr = Crb';
    % 更新当地加速度
    g = gp * (1+gk1*sin(SINSpositionition_d(2,t_imu))^2-gk2*sin(2*SINSpositionition_d(2,t_imu))^2);
    gn = [0;0;-g];
    % 更新姿态旋转矩阵
    Cen = FCen(SINSpositionition_d(1,t_imu),SINSpositionition_d(2,t_imu));
    Cnr = Cer * Cen';
    Cnb = Crb * Cnr;
    gb = Cnb * gn;
    gr = Cbr * gb;
  	 %%%%%%%%%%% 速度方程 %%%%%%%%%%            
    a_rbr = Cbr * f_INSm(:,t_imu) - getCrossMarix( 2*Wirr )*SINSvel(:,t_imu) + gr;      
    SINSacc_r(:,t_imu) = a_rbr;
    % 更新速度和位置
        % 捷联解算的导航坐标系：世界坐标系
    SINSvel(:,t_imu+1) = SINSvel(:,t_imu) + a_rbr * cycleT_INS;
    SINSposition(:,t_imu+1) = SINSposition(:,t_imu) + SINSvel(:,t_imu+1) * cycleT_INS;
    positione0 = Cre * SINSposition(:,t_imu+1) + positionr; % 将发射点惯性系中的位置转化到初始时刻地固系
    SINSpositionition_d(:,t_imu+1) = FZJtoJW(positione0,planet);
    
    %% KF滤波
    % 判断当前IMU数据是否为与某个图像最近的IMU数据，是则进行一次组合。
    % 保证一点：信息融合的时刻所用的视觉信息与惯导信息同一时刻。并注意视觉是两个时刻的相对运动信息的问题
    %       t_imu=1000 开始第一次信息融合
    %       用f_INSm(1000) 更新了 SINSposition(1001)，然后通过Rbb(1)Tbb(1)去修正SINSposition(1001)
    %       用f_INSm(1001)计算F_k,G_k,Fai_k
    %       用SINSposition(1001)，SINSposition(1)，Crb（1001）,Crb（1）,RbbVision(1),TbbVision(1)计算R_INS,T_INS,R_VNS,T_VNS
    %       用F_k,G_k,Fai_k，R_INS,T_INS,R_VNS,T_VNS计算组合的位置、速度、姿态<误差>：X(2)->dPositionEsm(2),dPositionEsm(2)...
    %       用组合估计的姿态位置等误差dangleEsm(2),dPositionEsm(2)...修正惯导姿态位置SINSposition(1001),Crb
    t_vision = (t_imu)/imu_fre*frequency_VO ;   % t_imu对应的t_vision（带小数点的个数）
    isIntegrate = 0 ;   % isIntegrate代表是否进行滤波采样并信息组合，允许imu_fre与frequency_VO之间不是整数倍
    if t_vision>=1
        num_vision_rem = abs( round(t_vision)-t_vision ) ; % 取小数部分        
        if num_vision_rem < frequency_VO/imu_fre/2  % 当前 t_imu离视觉采样最近
           isIntegrate = 1 ; 
        end
        if num_vision_rem == frequency_VO/imu_fre/2 && round(t_vision)-t_vision < 0 % 当t_imu正好在两帧视觉中间时取前面的
            isIntegrate = 1 ; 
        end        
    end
    if isIntegrate == 1     % 滤波采样
        %% 信息融合
        k_integ = fix(t_vision); 
        switch integMethod
            case 'simple_dRdT'  
                %% 惯导误差状态模型（简化-不扩维），dRdT作为量测
                    % 状态方程线性，量测方程线性，FK
                for i_non=1:1   % 无实际意义循环：讲所包含代码收起
                    % F,G,Fai：直接调用惯导误差方程即可
                    [F_k,G_k] = GetF_StatusErrorSINS(Cbr,Wirr,f_INSm(:,t_imu+1));  % 注意取滤波周期，而不是惯导解算周期
                    Fai_k = FtoFai(F_k,cycleT_VNS);
                    H = [eye(3),zeros(3,12);
                            zeros(3,6),-eye(3),zeros(3,6)];        % 量测矩阵为常量
                    % Q：系统噪声方差阵
                    Q_k = calQ( Q_ini,F_k,cycleT_VNS,G_k );
                  %  Q_k = [ Q_ini ];
                   % Q_k = G_k*Q_ini*G_k';
                     % 量测信息
                    [R_INS,T_INS,R_VNS,T_VNS] = calRT_INS_VS ( SINSposition(:,t_imu+1),SINSposition(:,t_imu+1-OneIntegT_IMUtime),Crb,CrbSave(:,:,t_imu+1-OneIntegT_IMUtime),RbbVision(:,:,k_integ),TbbVision(:,k_integ) );
                     % 得到量测量
                    opintions.headingScope=180;
                    Z = [GetAttitude(R_INS*R_VNS','rad',opintions);T_INS-T_VNS];  % 误差=INS-VNS，=> INS_true=INS-error_estimate         
                    
                    % KF滤波
                    X_pre = Fai_k * X(:,k_integ);   % 状态一步预测
                    P_pre = Fai_k * P(:,:,k_integ) * Fai_k' + Q_k;   % 均方误差一步预测

                    K_t = P_pre * H' / (H * P_pre * H' + R);   % 滤波增益
                    X(:,k_integ+1) = X_pre + K_t * (Z - H * X_pre);   % 状态估计

                    P_new = (eye(XNum) - K_t * H) * P_pre * (eye(XNum) - K_t * H)' + K_t * R * K_t';   % 估计均方误差
                    P(:,:,k_integ+1) = P_new;
                    % Rbb直接估计陀螺漂移
                  %  gyroDriftEsmA(:,k_integ+1) = GetGyroDriftEsmA(R_INS,R_VNS,cycleT_VNS);
                    % DRbb标定陀螺常值漂移法A
                    [gyroDriftEsmA(:,k_integ+1),P_gyroDRbbKFA(:,:,k_integ+1)] = gyroDRbbAKF(R_INS,R_VNS,cycleT_VNS,gyroDriftEsmA(:,k_integ),P_gyroDRbbKFA(:,:,k_integ),Q_gyroDRbbKFA,R_gyroDRbbKFA) ;
                    % DRbb标定陀螺常值漂移法B
                    [gyroDriftEsmB(:,k_integ+1),P_gyroDRbbUKFB(:,:,k_integ+1)] = gyroDRbbBUKF(R_INS,R_VNS,cycleT_VNS,gyroDriftEsmB(:,k_integ),P_gyroDRbbUKFB(:,:,k_integ),Q_gyroDRbbUKFB,R_gyroDRbbUKFB,Wrbb) ;
                    % KF 校核，存储中间量（检查调试时看）
                    if isKnowTrue==1
                        R_INS_save(:,:,k_integ) = R_INS;    T_INS_save(:,k_integ) = T_INS;   R_VNS_save(:,:,k_integ) = R_VNS;     T_VNS_save(:,k_integ) = T_VNS;
                        opintions.headingScope=180 ;
                        Cbr_true = FCbn(true_attitude(:,t_imu+1));  % 真实的姿态矩阵
                        Ccb = Crb ;     % 计算的姿态矩阵
                        Ccr_true = Cbr_true*Ccb ;    % 从计算机系到导航系，-> 真实的失准角
                        
%                         Crc_pre = FCbn(X_pre(1:3));
                        % 得到真实的平台失准角，此时认为计算机系 c 为参考系
                        platform_error_true = GetAttitude(Ccr_true,'rad',opintions) ;  
                        X_true = [platform_error_true;SINSvel(:,t_imu+1);SINSposition(:,t_imu+1);pg;pa]- [zeros(3,1);true_velocity(:,t_imu+1);true_position(:,t_imu+1);zeros(6,1)] ;
                        
                        X_pre_error(:,k_integ+1) = X_true-X_pre ;
                        
                        X_correct(:,k_integ+1) = K_t * (Z - H * X_pre);  % 当 X_pre_error 与 X_correct同号时具有修正作用
                        newInformation(:,k_integ+1) = Z - H * X_pre; % 新息
                    end
                    
                    % 保存误差估计值
                    dangleEsm(:,k_integ+1) = X(1:3,k_integ+1); 
                    dVelocityEsm(:,k_integ+1) = X(4:6,k_integ+1);
                    dPositionEsm(:,k_integ+1) = X(7:9,k_integ+1);       
                    gyroDrift(:,k_integ+1) = X(10:12,k_integ+1) ;
                    accDrift(:,k_integ+1) = X(13:15,k_integ+1) ;
                    % 保存估计均方误差
                    P_new_diag = sqrt(diag(P_new)) ;  % P阵对角元素
                    dangleEsmP(:,k_integ+1) = P_new_diag(1:3);
                    dVelocityEsmP(:,k_integ+1) = P_new_diag(4:6);
                    dPositionEsmP(:,k_integ+1) = P_new_diag(7:9);
                    gyroDriftP(:,k_integ+1) = P_new_diag(10:12);
                    accDriftP(:,k_integ+1) = P_new_diag(13:15);
                    % 由计算世界坐标系到真实世界坐标系的旋转矩阵
                    Crc = FCbn(dangleEsm(:,k_integ+1));           % 认为X(1:3,k_integ+1)是从计算机系c（SINS计算用r），转动到真实r坐标系的角度
                    X(1:9,k_integ+1) = 0;       % 修正状态
                end
            case 'simple_RT'
                %% 惯导力学状态模型（简化-非增广），R,T为量测
                    % 状态方程非线性，量测方程线性，EFK
                
            case 'augment_ZhiJie_QT'
                %% 增广，直接法（惯导力学状态模型），Q,T为量测
                    % 状态方程非线性，量测方程非线性，EFK
                for i_non=1:1
                    % F,Fai
                    Qrb_last = QrbSave(:,t_imu+1-OneIntegT_IMUtime) ;    % 取上一组合时刻的，已经修正的四元数
                    F_k = GetF_StatusSINS(Qrb_last,gyroDrift(:,k_integ),accDrift(:,k_integ),wib_INSm(:,t_imu+1),f_INSm(:,t_imu+1),Wirr) ;
                    Fai_k = FtoFai(F_k,cycleT_VNS);
                    % Q：系统噪声方差阵
                    G_k = [eye(4),zeros(4,3);
                         zeros(3,4),zeros(3);
                         zeros(3,4),eye(3);
                         zeros(6,7)]; 
                    Q_k = calQ( Q_ini,F_k,cycleT_VNS,G_k );
                    % H
                    dPosition = SINSposition(:,t_imu+1-OneIntegT_IMUtime) - SINSposition(:,t_imu+1) ;
                    H = GetH_augment_RT(Qrb,Qrb_last,dPosition);
                    % 计算真实量测：视觉输出
                    % 量测信息:直接就是Rbb和Tbb  在 b(k+1)下的分量
                    R_VNS_save(:,:,k_integ) = zeros(3);     T_VNS_save(:,k_integ) = zeros(3,1);
                    QVision = FCnbtoQ(RbbVision(:,:,k_integ)) ;
                    Z_vision = [QVision;TbbVision(:,k_integ)] ;
                    %%% EKF
                    % 状态一步预测:预测是通过惯导力学方程，因此，选用当前惯导解算出的姿态和位置作为状态预测值时可行的
                    X_pre = [Qrb;SINSposition(:,t_imu+1);SINSvel(:,t_imu+1);gyroDrift(:,k_integ);accDrift(:,k_integ);Qrb_last;INTGpos(:,k_integ)];
                    % 均方差一步预测
                    P_k = P(:,:,k_integ);
                    Poo = P_k(1:16,1:16);
                    Pod = P_k(1:16,17:23);
                    PodT = P_k(17:23,1:16);
                    Pdd = P_k(17:23,17:23);
                    Poo10 = Fai_k(1:16,1:16) * Poo * Fai_k(1:16,1:16)' + Q_k;
                    Pod10 = Fai_k(1:16,1:16) * Pod;
                    PodT10 = PodT * Fai_k(1:16,1:16)';
                    Pdd10 = Pdd;
                    P_pre = [Poo10 Pod10;PodT10 Pdd10];
                    % 增益矩阵
                    K = P_pre * H' / (H * P_pre * H' + R);   
                    % 状态估计
                    Qrb_last_inv = [Qrb_last(1);-Qrb_last(2:4)];         
                    Qbb_pre = QuaternionMultiply(Qrb_last_inv,Qrb) ;    % 一步预测四元数
                    Tbbpre = FQtoCnb(Qrb)*dPosition;
                    Z_pre = [Qbb_pre;Tbbpre];
                    Z_H_pre = H*X_pre ;
                    X(:,k_integ+1) = X_pre + K * (Z_vision - Z_pre);   
                    X(1:4,k_integ+1) = X(1:4,k_integ+1)/norm(X(1:4,k_integ+1)); % 单位化四元数
                    X(17:20,k_integ+1) = X(1:4,k_integ+1);
                    X(21:23,k_integ+1) = X(5:7,k_integ+1);
                    % 方差估计
                    P(:,:,k_integ+1)=(eye(XNum,XNum)-K*H)*P_pre*(eye(XNum,XNum)-K*H)'+K*R*K';
                    P(:,:,k_integ+1) = Ts * P(1:16,1:16,k_integ+1) * Ts';
                    % end of EKF
                    % 以惯导力学方程为状态模型时，估计出的是位置姿态，但为了统一，还是输出估计出的误差
                    % 计算 误差估计值
                        % 由计算世界坐标系到真实世界坐标系的旋转矩阵
                    Crc = FQtoCnb(X(1:4,k_integ+1));           % 认为X(1:4,k_integ+1)是从计算机系c（SINS计算用r），转动到真实r坐标系的角度
                    opintions.headingScope=180;
                    dangleEsm(:,k_integ+1) = GetAttitude(Crc,'rad',opintions); 
                    dVelocityEsm(:,k_integ+1) = SINSvel(:,t_imu+1)-X(8:10,k_integ+1);
                    dPositionEsm(:,k_integ+1) = SINSposition(:,t_imu+1)-X(5:7,k_integ+1);   % 估计的位置误差 = 惯导解算的位置-估计的位置   
                    gyroDrift(:,k_integ+1) = X(11:13,k_integ+1) ;
                    accDrift(:,k_integ+1) = X(14:16,k_integ+1) ;
                    
                    X(2:10,k_integ+1) = 0;       % 修正状态
                    X(1,k_integ+1) = 1;       % 修正状态
                end
            case 'augment_dRdT'
                %% 增广惯导误差状态方程，dRdT为量测
                    % 状态方程线性，量测方程非线性，EFK
                for i_non=1:1
                    % F,G,Fai：扩展调用惯导误差方程的结果
                    [F_INS,G_INS] = GetF_StatusErrorSINS(Cbr,Wirr,f_INSm(:,t_imu+1));  % 注意取滤波周期，而不是惯导解算周期
                    F_k = [F_INS zeros(15,6);zeros(6,21)];
                    G_k = [G_INS;zeros(6,6)];
                    Fai_k = FtoFai(F_k,cycleT_VNS);
                    % Q：系统噪声方差阵
                    Q_k = calQ( Q_ini,F_k,cycleT_VNS,G_k );
                  %  Q_k = Q_ini ;
                    % 量测矩阵 H  （雅克比求）
                    H1 = 1 / 2 * eye(3);
                    H2 = - 1 / 2 * Crb * CrbSave(:,:,t_imu+1-OneIntegT_IMUtime)';
            %         H1 = eye(3);
            %         H2 = - Crb * CrbSave(:,:,t_imu+1-OneIntegT_IMUtime)' * eye(3);%zeros(3)
                    H = [H1,zeros(3,12),H2,zeros(3);
                         zeros(3,6),-eye(3),zeros(3,6),eye(3),zeros(3)];
                     % 量测信息               
                    [Z,R_INS,T_INS,R_VNS,T_VNS] = calZ_augment_dRdT( SINSposition(:,t_imu+1),SINSposition(:,t_imu+1-OneIntegT_IMUtime),Crb,CrbSave(:,:,t_imu+1-OneIntegT_IMUtime),RbbVision(:,:,k_integ),TbbVision(:,k_integ) );
                    % EKF                    
                    X_pre = Fai_k * X(:,k_integ);   % 状态一步预测
                    % 均方差一步预测
                    P_k = P(:,:,k_integ);
            %         P_pre = Fai_k*P_k*Fai_k' + P_new_diag;
                    Poo = P_k(1:15,1:15);
                    Pod = P_k(1:15,16:21);
                    PodT = P_k(16:21,1:15);
                    Pdd = P_k(16:21,16:21);
                    Poo10 = Fai_k(1:15,1:15) * Poo * Fai_k(1:15,1:15)' + Q_k(1:15,1:15);
                    Pod10 = Fai_k(1:15,1:15) * Pod;
                    PodT10 = PodT * Fai_k(1:15,1:15)';
                    Pdd10 = Pdd;
                    P_pre = [Poo10 Pod10;PodT10 Pdd10];   % 均方误差一步预测
            %         P10a = Fai_k * P_k * Fai_k' + [P_new_diag,zeros(15,6);zeros(6,15),1e-8*eye(6,6)];
                    % 滤波增益
                    K_t = P_pre * H' / (H * P_pre * H' + R);   
                    % 状态估计
                    X(:,k_integ+1) = X_pre + K_t * (Z - H * X_pre);   
                    
                    P(:,:,k_integ+1)=(eye(XNum,XNum)-K_t*H)*P_pre*(eye(XNum,XNum)-K_t*H)'+K_t*R*K_t';
            %         P(16:21,16:21,k_integ+1) = Ts(16:21,:) * P(1:15,1:15,k_integ+1) * Ts(16:21,:)';
                    Ts = [eye(15);eye(3),zeros(3,12);zeros(3,6),eye(3),zeros(3,6)];
                    P_new = Ts * P(1:15,1:15,k_integ+1) * Ts';
                    P(:,:,k_integ+1) = P_new;
                    % end of EKF
                    % DRbb标定陀螺常值漂移法A
                    [gyroDriftEsmA(:,k_integ+1),P_gyroDRbbKFA(:,:,k_integ+1)] = gyroDRbbAKF(R_INS,R_VNS,cycleT_VNS,gyroDriftEsmA(:,k_integ),P_gyroDRbbKFA(:,:,k_integ),Q_gyroDRbbKFA,R_gyroDRbbKFA) ;
                    % DRbb标定陀螺常值漂移法B
                    [gyroDriftEsmB(:,k_integ+1),P_gyroDRbbUKFB(:,:,k_integ+1)] = gyroDRbbBUKF(R_INS,R_VNS,cycleT_VNS,gyroDriftEsmB(:,k_integ),P_gyroDRbbUKFB(:,:,k_integ),Q_gyroDRbbUKFB,R_gyroDRbbUKFB,Wrbb) ;
                    % KF 校核，存储中间量（检查调试时看）
                    if isKnowTrue==1
                        R_INS_save(:,:,k_integ) = R_INS;    T_INS_save(:,k_integ) = T_INS;   R_VNS_save(:,:,k_integ) = R_VNS;     T_VNS_save(:,k_integ) = T_VNS;
                        opintions.headingScope=180 ;
                        %%% 求当前时刻的平台失准角真值
                        Cbr_true = FCbn(true_attitude(:,t_imu+1));  % 真实的姿态矩阵
                        Ccb = Crb ;     % 计算的姿态矩阵
                        Ccr_true = Cbr_true*Ccb ;    % 从计算机系到导航系，-> 真实的失准角
                        % 得到真实的平台失准角，此时认为计算机系 c 为参考系
                        platform_error_true = GetAttitude(Ccr_true,'rad',opintions) ;  % 从 c 到 r
                        %%% 求上一时刻的平台失准角真值（校正后）
                        lastCbr_true = FCbn(true_attitude(:,t_imu+1-fix(imu_fre/frequency_VO)));
                        lastCcb = FCbn(INTGatt(:,k_integ))';
                        lastCcr_true = lastCbr_true*lastCcb ;
                        last_platform_error_true = GetAttitude(lastCcr_true,'rad',opintions) ;
                        
                        X_true = [platform_error_true;SINSvel(:,t_imu+1);SINSposition(:,t_imu+1);pg;pa;last_platform_error_true;INTGpos(:,k_integ)]- [zeros(3,1);true_velocity(:,t_imu+1);true_position(:,t_imu+1);zeros(6,1);zeros(3,1);true_position(:,t_imu+1-fix(imu_fre/frequency_VO))] ;
                        
                        X_pre_error(:,k_integ+1) = X_true-X_pre ;
                        
                        X_correct(:,k_integ+1) = K_t * (Z - H * X_pre);  % 当 X_pre_error 与 X_correct同号时具有修正作用
                        newInformation(:,k_integ+1) = Z - H * X_pre; % 新息
                    end                    

                    % 保存估计值
                    % 保存误差估计值
                    dangleEsm(:,k_integ+1) = X(1:3,k_integ+1); 
                    dVelocityEsm(:,k_integ+1) = X(4:6,k_integ+1);
                    dPositionEsm(:,k_integ+1) = X(7:9,k_integ+1);       
                    gyroDrift(:,k_integ+1) = X(10:12,k_integ+1) ;
                    accDrift(:,k_integ+1) = X(13:15,k_integ+1) ;
                    % 保存估计均方误差
                    P_new_diag = sqrt(diag(P_new)) ;  % P阵对角元素
                    dangleEsmP(:,k_integ+1) = P_new_diag(1:3);
                    dVelocityEsmP(:,k_integ+1) = P_new_diag(4:6);
                    dPositionEsmP(:,k_integ+1) = P_new_diag(7:9);
                    gyroDriftP(:,k_integ+1) = P_new_diag(10:12);
                    accDriftP(:,k_integ+1) = P_new_diag(13:15);
                    % 由计算世界坐标系到真实世界坐标系的旋转矩阵
                    Crc = FCbn(dangleEsm(:,k_integ+1));           % 认为X(1:3,k_integ+1)是从计算机系c（SINS计算用r），转动到真实r坐标系的角度
%                     X(16:18,k_integ+1) = X(1:3,k_integ+1);%zeros(3,1)
%                     X(19:21,k_integ+1) = X(4:6,k_integ+1);%zeros(3,1)
                    X(16:18,k_integ+1) = zeros(3,1);%
                    X(19:21,k_integ+1) = zeros(3,1);% 
                    X(1:9,k_integ+1) = 0;       % 修正状态
                end
        end
        %% 修正位置和速度
        SINSposition(:,t_imu+1) = SINSposition(:,t_imu+1) - dPositionEsm(:,k_integ+1);          
        SINSvel(:,t_imu+1) = SINSvel(:,t_imu+1) - dVelocityEsm(:,k_integ+1);
        positione0 = Cre * SINSposition(:,t_imu+1) + positionr; % 将发射点惯性系中的位置转化到初始时刻地固系
        SINSpositionition_d(:,t_imu+1) = FZJtoJW(positione0,planet);
        Cen = FCen(SINSpositionition_d(1,t_imu+1),SINSpositionition_d(2,t_imu+1));
  
        Cnr = Cer * Cen';
        
        % 修正方向余弦矩阵和姿态四元数(修正姿态) 
%          Cbr= Ccr * Cbr;  % Cbr=Ccr*Cbc  ---> Cbc=Cbr，即未更新前的r是c
%          Crb = Cbr';
         Crb = Crb*Crc;     
%         Crb = Crc' * Crb;  % 师姐用
%        Cnb = Crb * Cnr;
        Qrb = FCnbtoQ(Crb);
        QrbSave(:,t_imu+1)  = Qrb ;
        CrbSave(:,:,t_imu+1)  = Crb ;
        
        % 组合导航参数
        INTGpos(:,k_integ+1) = SINSposition(:,t_imu+1);
        INTGvel(:,k_integ+1) = SINSvel(:,t_imu+1);
        % 由方向余弦矩阵求姿态角
        opintions.headingScope=180;
        INTGatt(:,k_integ+1) = GetAttitude(Crb,'rad',opintions);
        
    end
end
close(waitbar_h)

%% 已知真实：计算导航误差
% 计算各维误差
if  isKnowTrue==1
    % 计算组合数据有效长度
    lengthArrayOld = [length(INTGpos),length(true_position)];
    frequencyArray = [integFre,trueTraeFre];
    [~,~,combineLength,combineFre] = GetValidLength(lengthArrayOld,frequencyArray);
    INTGPositionError = zeros(3,combineLength); % 组合导航的位置误差
    INTGAttitudeError = zeros(3,combineLength); % 组合导航的姿态误差
    INTGVelocityError = zeros(3,combineLength); % 组合导航的速度误差
    for k=1:combineLength
        k_true = fix((k-1)*(trueTraeFre/combineFre))+1 ;
        k_integ = fix((k-1)*(integFre/combineFre))+1;
        INTGPositionError(:,k) = INTGpos(:,k_integ)-true_position(:,k_true) ;
        INTGAttitudeError(:,k) = INTGatt(:,k_integ)-true_attitude(:,k_true);
        INTGAttitudeError(3,k) = YawErrorAdjust(INTGAttitudeError(3,k),'rad') ;
        INTGVelocityError(:,k) = INTGvel(:,k_integ)-true_velocity(:,k_true);  
    end    
    SINS_accError  =SINSacc_r-true_acc_r(:,1:length(SINSacc_r)) ; % SINS的加速度误差
    accDriftError = accDrift-repmat(pa,1,integnum) ;        % 组合导航的加计估计误差
    gyroDriftError = gyroDrift-repmat(pg,1,integnum) ;      % 组合导航的陀螺估计误差
    gyroDriftEsmAError = gyroDriftEsmA-repmat(pg,1,integnum) ;
    gyroDriftEsmBError = gyroDriftEsmB-repmat(pg,1,integnum) ;
    % 计算空间二维/三维位置误差及相对值
% dbstop in CalPosErrorIndex
    errorStr = CalPosErrorIndex( true_position,INTGPositionError,INTGAttitudeError*180/pi*3600 );
else
    errorStr = '\n真实未知';
end
accDriftStartErrorStr = sprintf('%0.3g  ',accDriftError(:,1)/(gp*1e-6));
accDriftEndErrorStr = sprintf('%0.3g  ',accDriftError(:,length(accDriftError))/(gp*1e-6));
gyroDriftStartErrorStr = sprintf('%0.3g  ',gyroDriftError(:,1)*180/pi*3600);
gyroDriftEndErrorStr = sprintf('%0.3g  ',gyroDriftError(:,length(gyroDriftError))*180/pi*3600);
recordStr = sprintf('%s\n\t初始、最终加计估计误差：(%s)、(%s) ug\n\t初始、最终陀螺估计误差：(%s)、(%s) °/h\n',errorStr,accDriftStartErrorStr,accDriftEndErrorStr,gyroDriftStartErrorStr,gyroDriftEndErrorStr) ;

X0str = sprintf('%0.3g  ',X(:,1));
P0str = sprintf('%0.3g  ',diag(P(:,:,1))');
Qstr = sprintf('%0.3g  ',diag(Q_k)');
R0str = sprintf('%0.3g  ',diag(R)');
recordStr = sprintf('%s\n滤波参数：\n\tX(0)=( %s )\n\tP(0)=( %s )\n\tQk=( %s )\n\tR(0)=( %s )\n',...
    recordStr,X0str,P0str,Qstr,R0str);

gyroDRbbAKFDriftEndErrorStr = sprintf('%0.3g  ',gyroDriftEsmAError(:,length(gyroDriftEsmAError))*180/pi*3600);
recordStr = sprintf('%s\nAKF法陀螺常值漂移标定\n\tAKF陀螺常值漂移标定误差：(%s) °/h\n',recordStr,gyroDRbbAKFDriftEndErrorStr) ;
Q_gyroDRbbKFAstr = sprintf('%0.3g  ',diag(Q_gyroDRbbKFA)');
R_gyroDRbbKFAstr = sprintf('%0.3g  ',diag(R_gyroDRbbKFA)');
P_gyroDRbbKFA0str = sprintf('%0.3g  ',diag(P_gyroDRbbKFA(:,:,1))');
gyroDriftEsmA0str = sprintf('%0.3g  ',gyroDriftEsmA(:,1));
recordStr = sprintf('%s\tAKF滤波参数：\n\tgyroDriftEsmA(0)=( %s )\n\tP_gyroDRbbUKFB_0=( %s )\n\tQ_gyroDRbbKFA=( %s )\n\tR_gyroDRbbKFA=( %s )\n',...
    recordStr,gyroDriftEsmA0str,P_gyroDRbbKFA0str,Q_gyroDRbbKFAstr,R_gyroDRbbKFAstr);

gyroDRbbBUKFDriftEndErrorStr = sprintf('%0.3g  ',gyroDriftEsmBError(:,length(gyroDriftEsmBError))*180/pi*3600);
recordStr = sprintf('%s\nBUKF法陀螺常值漂移标定\n\tBUKF陀螺常值漂移标定误差：(%s) °/h\n',recordStr,gyroDRbbBUKFDriftEndErrorStr) ;
Q_gyroDRbbUKFBstr = sprintf('%0.3g  ',diag(Q_gyroDRbbUKFB)');
R_gyroDRbbUKFBstr = sprintf('%0.3g  ',diag(R_gyroDRbbUKFB)');
P_gyroDRbbUKFB0str = sprintf('%0.3g  ',diag(P_gyroDRbbUKFB(:,:,1))');
gyroDriftEsmB0str = sprintf('%0.3g  ',gyroDriftEsmB(:,1));
recordStr = sprintf('%s\tBUKF滤波参数：\n\tgyroDriftEsmB(0)=( %s )\n\tP_gyroDRbbUKFB_0=( %s )\n\tQ_gyroDRbbUKFB=( %s )\n\tR_gyroDRbbUKFB=( %s )\n',...
    recordStr,gyroDriftEsmB0str,P_gyroDRbbUKFB0str,Q_gyroDRbbUKFBstr,R_gyroDRbbUKFBstr);

time=zeros(1,integnum);
for i=1:integnum
    time(i)=(i-1)/frequency_VO/60;
end
diagP_gyroDRbbKFA = zeros(3,length(P_gyroDRbbKFA));
diagP_gyroDRbbUKFB = zeros(3,length(P_gyroDRbbUKFB));
for k=1:length(P_gyroDRbbKFA)
    diagP_gyroDRbbKFA(:,k) = diag(P_gyroDRbbKFA(:,:,k)) ;
    diagP_gyroDRbbUKFB(:,k) = diag(P_gyroDRbbUKFB(:,:,k)) ;
end
%% 保存结果为特定格式
INS_VNS_NavResult = saveINS_VNS_NavResult_subplot(integFre,combineFre,imu_fre,projectName,gp,isKnowTrue,trueTraeFre,...
    INTGpos,INTGvel,INTGatt,dPositionEsm,dVelocityEsm,dangleEsm,accDrift,gyroDrift,INTGPositionError,true_position,...
    INTGAttitudeError,true_attitude,INTGVelocityError,accDriftError,gyroDriftError,dangleEsmP,dVelocityEsmP,dPositionEsmP,...
    gyroDriftP,accDriftP,SINS_accError,X_pre_error,X_correct,gyroDriftEsmA,gyroDriftEsmAError,gyroDriftEsmB,gyroDriftEsmBError,diagP_gyroDRbbKFA,diagP_gyroDRbbUKFB);

save([resultPath,'\INS_VNS_NavResult.mat'],'INS_VNS_NavResult')
disp('INS_VNS_ZdRdT 函数运行结束')

disp('滤波调试信息已输出到基本工作空间')

if exist('R_INS','var')
    check.R_INS=R_INS;  check.T_INS=T_INS;  check.R_VNS=R_VNS;  check.R_INS=R_INS;  check.T_VNS=T_VNS;  check.newInformation=newInformation;
    check.X_correct=X_correct;  check.X_pre_error=X_pre_error;
else
    check=[];
end

global projectDataPath 
if isAlone==1
   % 查看结果
    projectDataPath = pwd;
    [ResultDisplayPath,~] = GetUpperPath(pwd) ;
%     oldFloder = cd([ResultPath,'\ResultDispaly']) ; % 进入结果查看路径
    
%    copyfile([ResultDisplayPath,'\ResultDisplay\ResultDisplay.exe'],[pwd,'\navResult\ResultDisplay.exe']);
    
    % 记录实验参数
    fid = fopen([pwd,'\navResult\实验笔记(INS_VNS独立运行).txt'], 'w+');
    RecodeInput (fid,visualInputData,imuInputData,trueTrace);
    fprintf(fid,'\nINS_VNS_%s 解算误差：\n',integMethod);
    fprintf(fid,'%s',recordStr);
    fclose(fid);
    open([pwd,'\navResult\实验笔记(INS_VNS独立运行).txt'])
    if exist('VOResult','var')
       save([pwd,'\navResult\VOResult.mat'],'VOResult'); 
    end
    if exist('SINS_Result','var')
       save([pwd,'\navResult\SINS_Result.mat'],'SINS_Result'); 
    end
	ResultDisplay()
end

function Q = calQ( Q_ini,F,cycleT,G )
% Q：系统噪声方差阵
format long
Fi = F * cycleT;
Q = G*Q_ini*G';
tmp1 = Q * cycleT;
Q = tmp1;
for i = 2:11
    tmp2 = Fi * tmp1;
    tmp1 = (tmp2 + tmp2')/i;
    Q = Q + tmp1;
end
Q1=Q;

function gyroDrift = GetGyroDriftEsmA(R_INS,R_VNS,cycleT_VNS)
%% 利用 R_VNS-R_INS 直接估计陀螺常值漂移
dR = R_VNS-R_INS ;
gyroDrift(1) = (dR(3,2)-dR(2,3))/2/cycleT_VNS ;
gyroDrift(2) = (dR(1,3)-dR(3,1))/2/cycleT_VNS ;
gyroDrift(3) = (dR(2,1)-dR(1,2))/2/cycleT_VNS ;

function [ P_ini,Q_ini,R_ini,NavFilterParameter ] = GetFilterParameter_augment_ZhiJie_QT( pg,ng,pa,na,NavFilterParameter )
%% 导入初始滤波参数 P、Q、R

% if isfield(NavFilterParameter,'P_ini_augment_ZhiJie_QT')
%     P_ini = NavFilterParameter.P_ini_augment_ZhiJie_QT ;
% else
%     
% end
    szj1 = 0;
    szj2 = 0;
    szj3 = 0;
    szj4 = 0;
    P1_temp = diag([(szj1)^2,(szj2)^2,(szj3)^2,(szj4)^2,(0.001)^2,(0.001)^2,(0.001)^2,1e-9,1e-9,1e-9,...
                    (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2]);
    Ts = [eye(16);eye(4),zeros(4,12);zeros(3,4),eye(3),zeros(3,9)];
    P_ini = Ts * P1_temp * Ts';
%     NavFilterParameter.P_ini_augment_ZhiJie_QT =  ;
    
% if isfield(NavFilterParameter,'Q_ini_augment_ZhiJie_QT')
%     Q_ini = NavFilterParameter.Q_ini_augment_ZhiJie_QT ;
% else
% end
    Q_ini = diag([(ng(1))^2,(ng(2))^2,(ng(3))^2,(ng(3))^2,(na(1))^2,(na(2))^2,(na(3))^2]);      % ???
  %%%  Q_ini = diag([(ng(1))^2+1e-18,(ng(2))^2+1e-18,(ng(3))^2+1e-18,(na(1))^2+1e-14,(na(2))^2+1e-14,(na(3))^2+1e-14]);
%     NavFilterParameter.Q_ini_augment_ZhiJie_QT =  ;

if isfield(NavFilterParameter,'R_ini_augment_ZhiJie_QT')
    R_list_input = {NavFilterParameter.R_ini_augment_ZhiJie_QT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[4e-004,4e-004,4e-004,6e-007,6e-007,6e-007  ]',...     % 圆周360m Rbb 206"
                        '[1e-004,5e-004,8e-004,6e-007,6e-007,6e-007  ]',...
                        '[1e-005,1e-005,1e-003,6e-007,6e-007,6e-006  ]'....
                        '[1e-006,1e-006,8e-006,6e-004,6e-004,6e-004 ]',...      % 向前360m Tbb 0.02m
                        '[1e-0010,1e-0010,1e-0010,1e-0010,1e-0010,1e-0010  ]',...   % 无噪声
                        '[4e-004,4e-004,4e-004,6e-007,6e-007,6e-007  ]'}];      % 圆周360m Rbb 20.6"

[Selection,ok] = listdlg('PromptString','R_ini(前R后T):','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0
    Selection = 1;
end
answer = inputdlg('噪声方程阵R(前R后T)                                  .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R
NavFilterParameter.R_ini_augment_ZhiJie_QT = answer{1} ;

function [ P_ini,Q_ini,R_ini,NavFilterParameter ] = GetFilterParameter_simple_dRdT( pg,ng,pa,na,NavFilterParameter )
%% 导入初始滤波参数 P、Q、R
% if isfield(NavFilterParameter,'P_ini_simple_dRdT')
%     P_ini = NavFilterParameter.P_ini_simple_dRdT ;
% else
%     
% end
    szj1 = 1/3600*pi/180 * 0;
    szj2 = 1/3600*pi/180 * 0;
    szj3 = 1/3600*pi/180 * 0;
    P_ini = diag([(szj1)^2,(szj2)^2,(szj3)^2,(0.001)^2,(0.001)^2,(0.001)^2,1e-9,1e-9,1e-9,...
                    (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2]); %  15*15
%      P_ini = diag([(szj1)^2,(szj2)^2,(szj3)^2,(0.001*0)^2,(0.001*0)^2,(0.001*0)^2,1e-9*0,1e-9*0,1e-9*0,...
%                                 (pg(1))^2+1e-8*0,(pg(2))^2+1e-8*0,(pg(3))^2+1e-8*0,(pa(1))^2+1e-12*0,(pa(2))^2+1e-12*0,(pa(3))^2+1e-12*0]);
     NavFilterParameter.P_ini_simple_dRdT = sprintf('%1.1e ',P_ini) ;
    
% if isfield(NavFilterParameter,'Q_ini_simple_dRdT')
%     Q_ini = NavFilterParameter.Q_ini_simple_dRdT ;
% else
% end
% 0.01°/h=4.8e-8 rad/s        1ug=1.62*1e-6 m/s^2
    Q_ini = diag([(ng(1))^2,(ng(2))^2,(ng(3))^2,(na(1))^2,(na(2))^2,(na(3))^2]);
  %%%  Q_ini = diag([(ng(1))^2+1e-19,(ng(2))^2+1e-19,(ng(3))^2+1e-19,(na(1))^2+1e-15,(na(2))^2+1e-15,(na(3))^2+1e-15]);
%     Q_ini = diag([  10e-12 10e-12 10e-10 ...         % 失准角微分方程
%                     10e-12 10e-12 10e-14...         % 速度微分方程
%                     10e-17 10e-17 10e-17...         % 位置微分方程
%                     0 0 0 ...         % 陀螺常值微分方程
%                     0 0 0  ]);       % 加计常值微分方程
                
%     Q_ini = diag([  2e-13 2e-13 2e-13 ...         % 失准角微分方程
%                     2e-12 2e-12 2e-12...         % 速度微分方程
%                     2e-12 2e-12 2e-12 ...         % 位置微分方程
%                     0 0 0 ...         % 陀螺常值微分方程
%                     0 0 0  ]);       % 加计常值微分方程
% Q_ini = diag([      10e-12 10e-10 10e-10 ...         % 失准角微分方程
%                     10e-15 10e-15 10e-24...         % 速度微分方程
%                     10e-17 10e-17 10e-27...         % 位置微分方程
%                     0 0 0 ...         % 陀螺常值微分方程
%                     0 0 0  ]);       % 加计常值微分方程
	NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_ini) ;

if isfield(NavFilterParameter,'R_ini_simple_dRdT')
    R_list_input = {NavFilterParameter.R_ini_simple_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e-5  [1 1 1]*1e-5]'...
                        '[4e-004,4e-004,4e-004,6e-007,6e-007,6e-007  ]',...     % 圆周360m Rbb 206"
                        '[1e-004,5e-004,8e-004,6e-007,6e-007,6e-007  ]',...
                        '[1e-005,1e-005,1e-003,6e-007,6e-007,6e-006  ]'....
                        '[1e-006,1e-006,8e-006,6e-004,6e-004,6e-004 ]',...      % 向前360m Tbb 0.02m
                        '[4e-004,4e-004,4e-004,6e-007,6e-007,6e-007  ]'}];      % 圆周360m Rbb 20.6"

[Selection,ok] = listdlg('PromptString','R_ini(前R后T)_simple_dRdT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0
    Selection = 1;
end
answer = inputdlg('噪声方程阵R(前R后T)_simple_dRdT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R
NavFilterParameter.R_ini_simple_dRdT = answer{1} ;

    %    R = diag([20e-1,20e-1,20e-1,1e-6,1e-6,1e-6]*1e-3);
    % defaultR = {'[1e-005,1e-005,1e-003,6e-007,6e-007,6e-006  ]'};    % 前R后T "R-30"
    % defaultR = {'[1e-004,1e-004,8e-004,1e-007,1e-007,1e-007  ]'};    % 前R后T "R-1"
    % defaultR = {'[1e-0010,1e-0010,1e-0010,1e-0010,1e-0010,1e-0010  ]'};    
    % defaultR = {'[1e-004,1e-004,1e-004,6e-007,6e-007,6e-007  ]'};    % 前R后T "R-20"
    % defaultR = {'[8e-004,8e-004,1e-003,1e-007,1e-007,1e-006  ]'};    % 前R后T "R-10-10"
    % defaultR = {'[4e-004,4e-004,4e-004,6e-007,6e-007,6e-007  ]'};    % 前R后T "R-10"
    % defaultR = {'[4e-003,4e-003,4e-003,6e-006,6e-006,6e-006  ]'};    % 前R后T "R-10"
    % defaultR = {'[4e-003,4e-003,4e-003,6e-008,6e-008,6e-008  ]'};    % 前R后T "R-10"
    % defaultR = {'[8e-005,8e-005,8e-005,6e-007,6e-007,6e-007]'};       % 前R后T "R-50,T-50"
    % defaultR = {'[1e-5,1e-5,1e-5,0.008,0.008,0.008 ]'};             % 前R后T ，T-50
    % defaultR = {'[8e-006,8e-006,8e-006,6e-004,6e-004,6e-004 ]'};    %前R后T "T-20" 第1种（正常方差设置）
    % defaultR = {'[1e-004,1e-004,1e-004,6e-007,6e-007,6e-007 ]'};    %前R后T "T-20" 第2种 （奇怪，但效果也还行）
    % defaultR = {'[6e-004,6e-004,6e-004,6e-007,6e-007,6e-007  ]'};    % 前R后T "R-50"
    % defaultR = {'[8e-005,8e-005,8e-005,6e-007,6e-007,6e-007]'};      % 前R后T "R-50,T-50"
    % defaultR = {'[1e-009,1e-009,4e-007,1e-005,1e-005,1e-005 ]'};    %前R后T "T-10" 
    % defaultR = {'[1e-008,1e-008,8e-008,1e-005,1e-005,1e-005 ]'};    %前R后T "T-20" 圆周
    % defaultR = {'[1e-006,1e-006,8e-006,6e-004,6e-004,6e-004 ]'};    %前R后T "T-20"


    % display(P)
    % display(Q_ini)
    % R = diag(1e0*[1.6e-5,3.4e-7,9.9e-6,2.1e-5,3.4e-5,5.6e-5]); % with noise: 0.5 pixel
    % R = diag([1e-12*ones(1,3),2.1e-5,3.4e-5,5.6e-5]);
    % R = diag([5.9e-6,6.2e-8,3.1e-6,5.9e-5,1.5e-5,1.0e-4]); % line60
    % 0.5pixel
    % R = diag([4.3e-6,1.5e-7,6.5e-6,1.3e-4,1.1e-5,7.7e-5]); % arc 0.5pixel
    % R = diag([2.5e-6,8.5e-8,3.9e-6,7.4e-5,1.1e-5,4.3e-5]); % zhx
    % 0.5pixel

function [ P_ini,Q_ini,R_ini,NavFilterParameter ] = GetFilterParameter_augment_dRdT( pg,ng,pa,na,NavFilterParameter )
%% 导入初始滤波参数 P、Q、R
% if isfield(NavFilterParameter,'P_ini_augment_dRdT')
%     P_ini = NavFilterParameter.P_ini_augment_dRdT ;
% else
%     
% end
    szj1 = 1/3600*pi/180 * 6;
    szj2 = 1/3600*pi/180 * 6;
    szj3 = 1/3600*pi/180 * 6;
    P1_temp = diag([(szj1)^2,(szj2)^2,(szj3)^2,(0.001)^2,(0.001)^2,(0.001)^2,1e-9,1e-9,1e-9,...
                    (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2]); %  15*15
%     P1_temp = diag([(szj1)^2,(szj2)^2,(szj3)^2,(0.001)^2,(0.001)^2,(0.001)^2,1e-9,1e-9,1e-9,...
%                                     (pg(1))^2+1e-8,(pg(2))^2+1e-8,(pg(3))^2+1e-8,(pa(1))^2+1e-12,(pa(2))^2+1e-12,(pa(3))^2+1e-12]);
%     P1_temp = diag([(szj1)^2,(szj2)^2,(szj3)^2,(0.001)^2,(0.001)^2,(0.001)^2,1e-9,1e-9,1e-9,...
%                                 (pg(1))^2+1e-6,(pg(2))^2+1e-6,(pg(3))^2+1e-6,(pa(1))^2+1e-10,(pa(2))^2+1e-10,(pa(3))^2+1e-10]);
    Ts = [eye(15);eye(3),zeros(3,12);zeros(3,6),eye(3),zeros(3,6)]; % 21*15
    P_ini = Ts * P1_temp * Ts';    
     NavFilterParameter.P_ini_augment_dRdT =  sprintf('%1.1e ',P_ini) ;
    
% if isfield(NavFilterParameter,'Q_ini_augment_dRdT')
%     Q_ini = NavFilterParameter.Q_ini_augment_dRdT ;
% else
% end
    Q_ini = diag([(ng(1))^2,(ng(2))^2,(ng(3))^2,(na(1))^2,(na(2))^2,(na(3))^2]);      % ???
  %  Q_ini = diag([(ng(1))^2+1e-19,(ng(2))^2+1e-19,(ng(3))^2+1e-19,(na(1))^2+1e-15,(na(2))^2+1e-15,(na(3))^2+1e-15]);
%     Q_ini = diag([  2e-13 2e-13 2e-13 ...         % 失准角微分方程
%                     2e-12 2e-12 2e-12...         % 速度微分方程
%                     0 0 0 ...         % 位置微分方程
%                     0 0 0 ...         % 陀螺常值微分方程
%                     0 0 0  ]);       % 加计常值微分方程
% Q_ini = diag([      10e-12 10e-10 10e-10 ...         % 失准角微分方程
%                     10e-15 10e-15 10e-24...         % 速度微分方程
%                     10e-17 10e-17 10e-27...         % 位置微分方程
%                     0 0 0 ...         % 陀螺常值微分方程
%                     0 0 0  ]);       % 加计常值微分方程
     NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_ini) ;



if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e-5  [1 1 1]*1e-5]'...
                        '[4e-004,4e-004,4e-004,6e-007,6e-007,6e-007  ]',...     % 圆周360m Rbb 206"
                        '[1e-004,5e-004,8e-004,6e-007,6e-007,6e-007  ]',...
                        '[1e-005,1e-005,1e-003,6e-007,6e-007,6e-006  ]'....
                        '[1e-006,1e-006,8e-006,6e-004,6e-004,6e-004 ]',...      % 向前360m Tbb 0.02m
                        '[4e-004,4e-004,4e-004,6e-007,6e-007,6e-007  ]'}];      % 圆周360m Rbb 20.6"

[Selection,ok] = listdlg('PromptString','R_ini(前R后T)-augment_dRdT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R后T)-augment_dRdT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;
