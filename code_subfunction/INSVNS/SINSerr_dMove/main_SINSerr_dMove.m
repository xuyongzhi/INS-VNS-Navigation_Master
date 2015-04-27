%% INS_VNS组合，惯导误差状态方程 dQbb dTbb
%　　　　　　　　buzz xyz
%               2014.5.23
% 6.7 21:39 发现 在 Crc补偿之前Crb没有更新的大问题
%% 
% X=[dat dv dr gyroDrift accDrift dat_last dr_last] 
% isTrueX0=1 ： 采用准确初值
% Z_method = 'sub_QT'(传统误差定义方法，subQ subT 量测)  'd_RT'
% integMethodDisplay : 此方法绘图时显示的名字
% timeShorted =0.5 : 计算的时长倍数

function [INS_VNS_NavResult,check,recordStr,NavFilterParameter] = main_SINSerr_dMove(visualInputData,imuInputData,trueTrace,NavFilterParameter,isTrueX0,Z_method,integMethodDisplay,timeShorted)

format long

%%
if ~exist('visualInputData','var')
    % 独立运行
    clc
    clear all 
    close all
    load('SINSerror_subQbbsubTbb.mat')
    isAlone = 1;
 %   Z_method='d_RTw';
     Z_method='sub_QTb';
%   Z_method='d_RTb';
    
     integMethodDisplay='sub_QTb';
%    integMethodDisplay='d_RTw';
%    integMethodDisplay='d_RTb';
    
    timeShorted = 1;
%     isTrueX0=1;
else
    isAlone = 0;
%     isTrueX0=1;
end

%% 参数设置
isCompensateDrift = 0 ; % （0/1）是否用 陀螺和加计漂移 补偿IMU数据
isComensateAccDrift=[1;1;1];
isCompensateGyroDrift=[1;1;1];
%% 调试模式 isDebudMode ：正常情况下全部是0
isDebugMode.debugEnable = 0 ;
isDebudMode.trueRbb = isDebugMode.debugEnable* 0  ;
isDebudMode.trueTbb = isDebugMode.debugEnable* 0 ;
isDebudMode.trueGyroDrift = isDebugMode.debugEnable* 0;
isDebudMode.trueAccDrift = isDebugMode.debugEnable* 0;
isDebudMode.onlySINS = isDebugMode.debugEnable* 0;       % 纯惯导递推
isDebudMode.onlyStateFcn = isDebugMode.debugEnable* 0 ;  % 纯状态方程递推，用此时得到的 位置姿态等误差 进行补偿，不利用量测，这样可以观察 状态方程 和 量测方程
isDebudMode.isResetDrift = isDebugMode.debugEnable* 0 ;  % 每次滤波后重置状态漂移量为0
isDebudMode.isTrueIMU = isDebugMode.debugEnable* 0 ;     % 采用 trueTrace 中未加噪声的 IMU 数据
display(isDebudMode)
%% 量测量计算参数
calZMethod.Z_method = Z_method ;  % 'sub_QT'  'd_RT'
calZMethod.Z_subQT_methodFlag = 0;  % 量测量Z的集中计算方法 0/1/2
display(calZMethod)
if isAlone==0
%     isTrueX0=1;
    save SINSerror_subQbbsubTbb visualInputData  imuInputData  trueTrace  NavFilterParameter  isTrueX0 isCompensateDrift Z_method integMethodDisplay
end
format long
disp('函数 INS_VNS_ZdRdT 开始运行')
% addpath([pwd,'\sub_code']);
% oldfolder=cd([getUpperPath(pwd),'\commonFcn']);
% add_CommonFcn_ToPath;
% cd(oldfolder);
% addpath([getUpperPath(pwd),'\ResultDisplay']);

%% 导入数据
% (1) 导入纯视觉导航仿真解算的的中间结果，包括两个数据:Rbb[例3*3*127]、Tbb[例3*127]
VisualOut_RT=visualInputData.VisualRT;
RbbVision = VisualOut_RT.Rbb;
if isfield(VisualOut_RT,'Tbb_last')
    isTbb_last=1;
    TbbVision = VisualOut_RT.Tbb_last;
else
    isTbb_last=0;
    TbbVision = VisualOut_RT.Tbb;
end
if isDebudMode.trueRbb==1
    RbbVision = VisualOut_RT.trueRbb;
end
if isDebudMode.trueTbb==1
    TbbVision = VisualOut_RT.trueTbb;
end
frequency_VO = visualInputData.frequency;
% （2）IMU数据
if isDebudMode.isTrueIMU == 0
    wib_INS = imuInputData.wib;
    fb_INS = imuInputData.f;
    imu_fre = imuInputData.frequency;   % Hz
else
    % 采用未加噪声的IMU
    wib_INS = trueTrace.wib_IMU;
    fb_INS = trueTrace.f_IMU;
    imu_fre = trueTrace.frequency;      % Hz
end


% 真实轨迹的参数
if ~exist('trueTrace','var')
    trueTrace = [];
end
resultPath = [pwd,'\navResult'];
if isdir(resultPath)
    delete([resultPath,'\*.mat'])
else
   mkdir(resultPath) 
end
[planet,isKnowTrue,initialPosition_e,initialVelocity_r,initialAttitude_r,trueTraeFre,true_position,...
    true_attitude,true_velocity,true_acc_r,runTime_IMU,runTime_image] = GetFromTrueTrace( trueTrace );
% true_position=-true_position;true_velocity=-true_velocity;initialVelocity_r=-initialVelocity_r;
% 星体常数
if strcmp(planet,'m')
    moonConst = getMoonConst;     	% 得到月球常数
    gp = moonConst.g0 ;             % 用于导航解算
    wip = moonConst.wim ;
    Rp = moonConst.Rm ;
    e = moonConst.e;
    gk1 = moonConst.gk1;
    gk2 = moonConst.gk2;
    disp('轨迹发生器：月球')
else
    earthConst = getEarthConst;     % 得到地球常数
    gp = earthConst.g0 ;            % 用于导航解算
    wip = earthConst.wie ;
    Rp = earthConst.Re ;
    e = earthConst.e;
    gk1 = earthConst.gk1;
    gk2 = earthConst.gk2;
    disp('轨迹发生器：地球')
end
% 月球模型参数
Wipp=[0;0;wip];
% sample period
imuNum = size(fb_INS,2);
integnum1 = size(TbbVision,2)+1 ;
integnum2 = fix(imuNum*frequency_VO/imu_fre)+1 ;    % 组合导航位置姿态数据个数
integnum = min(integnum1,integnum2);
integnum = fix(integnum*timeShorted) ;      %  截取一部分数据
imuNum = min(imuNum,fix((integnum-1)*imu_fre/frequency_VO)) ;

integFre = frequency_VO;

if isempty(runTime_IMU)
    cycleT_INS = 1/imu_fre*ones(imuNum,1);      % 捷联解算周期  sec
else
    cycleT_INS = runTime_to_setpTime(runTime_IMU) ;
end
if isempty(runTime_image)
    cycleT_VNS = 1/frequency_VO*ones(imuNum,1);      % 视觉数据周期/滤波周期  sec
else
    cycleT_VNS = runTime_to_setpTime(runTime_image) ;
end
OneIntegT_IMUtime = fix(imu_fre/integFre);  % 一个组合周内，IMU解算的次数

%% SINS导航参数
% 由IMU噪声确定滤波PQ初值的选取
    % 仿真时噪声已知，存储在imuInputData中，实验室噪声未知，手动输入 常值偏置 和 随机标准差
[pa,na,pg,ng,~] = GetIMUdrift( imuInputData,planet ) ; % pa(加计常值偏置),na（加计随机漂移）,pg(陀螺常值偏置),ng（陀螺随机漂移）
%初始位置误差 m 
dinit_pos = trueTrace.InitialPositionError ;
%初始姿态误差 rad
dinit_att = trueTrace.InitialAttitudeError ;
% 捷联惯导解算导航参数
% CrbSave = zeros(3,3,imuNum+1);    % 姿态矩阵记录
SINSatt = zeros(3,imuNum+1);        % 欧拉角姿态
SINSQ = zeros(4,imuNum+1);          % 姿态四元数
SINSvel = zeros(3,imuNum+1);        % 速度
SINSposition = zeros(3,imuNum+1);   % 位置 米
SINSacc_r = zeros(3,imuNum);        % 加速度
SINSpositionition_d = zeros(3,imuNum+1);% 大地坐标系 经纬度
%% SINS初始条件
SINSpositionition_d(:,1) = initialPosition_e;   % 经度 纬度 高度
SINSatt(:,1) = initialAttitude_r;               % 初始姿态 sita ,gama ,fai （rad）

positionr = FJWtoZJ(SINSpositionition_d(:,1),planet);  %地固坐标系中的初始位置
% positionr = positionr+dinit_pos ;             % 叠加初始位置误差
% SINSpositionition_d(:,1) = FZJtoJW(positionr,planet);
Cer=FCen(SINSpositionition_d(1,1),SINSpositionition_d(2,1));  % 世界坐标系相对于初始时刻地固系的旋转矩阵
Cre = Cer';
Cbr = FCbn(SINSatt(:,1));
Cbr = Cbr*FCbn(dinit_att);                      % 叠加初始姿态误差
opintions.headingScope = 180;
SINSatt(:,1) = GetAttitude(Cbr','rad',opintions) ;
Crb = Cbr';

Wirr = Cer * Wipp;
SINSvel(:,1) =  initialVelocity_r;
% 根据初始姿态矩阵Crb计算初始姿态四元数
SINSQ(:,1) = FCnbtoQ(Crb);
% CrbSave(:,:,1) = Crb ;
%% 组合导航参数
INTGatt = zeros(3,integnum);  % 欧拉角姿态
INTGvel = zeros(3,integnum);  % 速度
INTGpos = zeros(3,integnum);  % 位置
INTGacc = zeros(3,integnum);  % 加速度

INTGvel(:,1) = SINSvel(:,1);
INTGatt(:,1) = SINSatt(:,1);
% 组合导航估计的误差
dAngleEsm = zeros(3,integnum);          % 平台失准角估计值
dVelocityEsm = zeros(3,integnum);       % 速度误差估计值
dPositionEsm = zeros(3,integnum);       % 位置误差估计值
gyroDrift = zeros(3,integnum);          % 陀螺漂移估计值
accDrift = zeros(3,integnum);           % 加计漂移估计值

dAngleEsmP = zeros(3,integnum);       	% 平台失准角估计均方误差
dVelocityEsmP = zeros(3,integnum);      % 速度误差估计均方误差
dPositionEsmP = zeros(3,integnum);      % 位置误差估计均方误差
gyroDriftP = zeros(3,integnum);         % 陀螺漂移估计均方误差
accDriftP = zeros(3,integnum);          % 加计漂移估计均方误差
% 中间参数
R_INS_save = zeros(3,3,integnum-1);
T_INS_save = zeros(3,integnum-1);
R_VNS_save = zeros(3,3,integnum-1);
T_VNS_save = zeros(3,integnum-1);
projectName = integMethodDisplay;  % 存储在结果中，绘图时显示
%% 组合导航参数
XNum = 21;
ZNum = 6; % 量测信息维数
X = zeros(XNum,integnum);       % 状态向量
X_pre_error = zeros(XNum,integnum);   
X_correct = zeros(XNum,integnum);
if isTrueX0==1
    X(:,1) = [zeros(9,1);pg;pa;zeros(6,1)]; 
    warning('isTrueX0=1')
else 
%     pgError0 = [0.1;0.1;0.1]*pi/180/3600 ;  % 陀螺常值漂移 状态分量初值误差
%     paError0 = [10;10;10]*gp/1e6   ;         % 加计常值漂移 状态分量初值误差
%     X(:,1) = [zeros(9,1);pg-pgError0;pa-paError0;zeros(6,1)]; 
    X(:,1) = zeros(XNum,1);
    warning('isTrueX0=0')
end
Zinteg = zeros(ZNum,integnum-1);
Zinteg_error = zeros(ZNum,integnum-1);
Zinteg_pre = zeros(ZNum,integnum-1);
newInformation = zeros(ZNum,integnum);
P = zeros(XNum,XNum,integnum); % 滤波P阵s
[ P(:,:,1),Q_const,R_const,NavFilterParameter ] = GetFilterParameter_SINSerror_dMove( pg,ng,pa,na,NavFilterParameter ) ;

waitbarTitle = sprintf('%s:INS\_VNS组合导航解算',Z_method);

dAngleEsm(:,1) = X(1:3,1); 
dVelocityEsm(:,1) = X(4:6,1);
dPositionEsm(:,1) = X(7:9,1);   
gyroDrift(:,1) = X(10:12,1) ;
accDrift(:,1) = X(13:15,1) ;
P0_diag = sqrt(diag(P(:,:,1))) ;  % P0阵对角元素
dAngleEsmP(:,1) = P0_diag(1:3);
dVelocityEsmP(:,1) = P0_diag(4:6);
dPositionEsmP(:,1) = P0_diag(7:9);
gyroDriftP(:,1) = P0_diag(10:12);
accDriftP(:,1) = P0_diag(13:15);

%% 开始导航解算
% 记录上一滤波时刻的姿态和位置
k_integ=0;
waitbar_h=waitbar(0,strToDis(waitbarTitle));
for t_imu = 1:imuNum
    if mod(t_imu,ceil((imuNum-1)/10))==0
        waitbar(t_imu/(imuNum-1))
    end
    %% 世界坐标系SINS导航解算
    % 用捷联解算计算状态一步预测：四元数、速度、位置
    Crb = FQtoCnb(SINSQ(:,t_imu));
    wib_t_imu = wib_INS(:,t_imu) ;
    if isCompensateDrift==1
         wib_t_imu = wib_t_imu-gyroDrift(:,k_integ+1).* isCompensateGyroDrift;
     end
    Wrbb = wib_t_imu - Crb * Wirr;
    % 角增量法解四元数微分方程（简化的）
%     SINSQ(:,t_imu+1)=SINSQ(:,t_imu)+0.5*cycleT_INS(t_imu)*[      0    ,-Wrbb(1,1),-Wrbb(2,1),-Wrbb(3,1);
%                                                         Wrbb(1,1),     0    , Wrbb(3,1),-Wrbb(2,1);
%                                                         Wrbb(2,1),-Wrbb(3,1),     0    , Wrbb(1,1);
%                                                         Wrbb(3,1), Wrbb(2,1),-Wrbb(1,1),     0    ]*SINSQ(:,t_imu);
%     SINSQ(:,t_imu+1)=SINSQ(:,t_imu+1)/norm(SINSQ(:,t_imu+1));      % 单位化四元数    
    SINSQ(:,t_imu+1)  = QuaternionDifferential( SINSQ(:,t_imu),Wrbb,cycleT_INS(t_imu) ) ;
    % 更新当地加速度
    g = gp * (1+gk1*sin(SINSpositionition_d(2,t_imu))^2-gk2*sin(2*SINSpositionition_d(2,t_imu))^2);
    gn = [0;0;-g];
    % 更新Cen：只在计算 gr 的时候需要
    Cen = FCen(SINSpositionition_d(1,t_imu),SINSpositionition_d(2,t_imu));
    Cnr = Cer * Cen';
    gr = Cnr * gn;
  	 %%%%%%%%%%% 速度方程 %%%%%%%%%%  
    fb_t_imu = fb_INS(:,t_imu); 
    if isCompensateDrift==1
      	fb_t_imu = fb_t_imu-accDrift(:,k_integ+1).*isComensateAccDrift ;
    end
    a_rbr = Crb' * fb_t_imu - getCrossMatrix( 2*Wirr )*SINSvel(:,t_imu) + gr;      
    SINSacc_r(:,t_imu) = a_rbr;
    % 更新 Crb ，滤波后对这个 Crb 进行修正,并重新更新到 SINSQ(:,t_imu+1)
    Crb = FQtoCnb(SINSQ(:,t_imu+1));
    % 更新速度和位置
        % 捷联解算的导航坐标系：世界坐标系
    SINSvel(:,t_imu+1) = SINSvel(:,t_imu) + a_rbr * cycleT_INS(t_imu);
    SINSposition(:,t_imu+1) = SINSposition(:,t_imu) + SINSvel(:,t_imu) * cycleT_INS(t_imu);
    positione0 = Cre * SINSposition(:,t_imu+1) + positionr; % 将发射点惯性系中的位置转化到初始时刻地固系
    SINSpositionition_d(:,t_imu+1) = FZJtoJW(positione0,planet);    % 用于更新 gr 的计算
    
    %% 信息融合 EKF滤波
    % t_imu=100 20 300 ...    
    % t_imu=100,k_integ=1,用“RbbVision(:,:,1),wib_INS(:,1),wib_INS(:,-99),X(:,1),SINSvel(:,101)”算“X(:,2)”
    % t_imu=200,k_integ=2,用“RbbVision(:,:,2),wib_INS(:,101),wib_INS(:,1),X(:,2),SINSvel(:,201)”算“X(:,3)”
    % RbbVision(:,:,1)：t_imu=1到t_imu=101。RbbVision(:,:,2)：t_imu=101到t_imu=201。
    % RbbVision(:,:,1) 对应 X(:,1)产生的量测Qbb
    
    if mod(t_imu,imu_fre/frequency_VO)==0
        isIntegrate = 1 ;   
        if isDebudMode.onlySINS==1
            isIntegrate = 0 ;   
            k_integ = round((t_imu)*frequency_VO/imu_fre) ; 
            INTGpos(:,k_integ+1) = SINSposition(:,t_imu+1) ;  
            INTGvel(:,k_integ+1)  = SINSvel(:,t_imu+1) ;
            opintions.headingScope=180;
            INTGatt(:,k_integ+1) = GetAttitude(Crb,'rad',opintions);
        end
    else
        isIntegrate = 0 ;
    end   
    if isIntegrate == 1     % 滤波采样
        
        k_integ = round((t_imu)*frequency_VO/imu_fre) ; 
        
        fb_k = fb_INS(:,fix(t_imu+1-OneIntegT_IMUtime/2));
        if isCompensateDrift==1
             fb_k = fb_k-accDrift(:,k_integ).*isComensateAccDrift ;
         end
        % SINS解算的（姿态、位置）要用于求量测量中的SINS部分
        position_integ = INTGpos(:,k_integ) ;
        position_SINSpre = SINSposition(:,t_imu+1) ;
        Crb_SINSpre = FQtoCnb(SINSQ(:,t_imu+1));    
        Crb_k_integ = FCbn(INTGatt(:,k_integ))';    % 上一时刻滤波估计得到的结果        
%         if isCompensateDrift==1     % 如果该维常值漂移被补偿，则在状态预测前，该维的漂移置0
%             X(10:12,k_integ) = (~isCompensateGyroDrift).*X(10:12,k_integ) ;
%             X(13:15,k_integ) = (~isComensateAccDrift).*X(13:15,k_integ) ;
%         end        
        [ X(:,k_integ+1),P(:,:,k_integ+1),X_correct(:,k_integ+1),X_pre,Zinteg_error(:,k_integ),Zinteg(:,k_integ),Zinteg_pre(:,k_integ),R_INS,T_INS,R_VNS,T_VNS ] = updateX_SINSerror_subQbbsubTbb...
            ( X(:,k_integ),P(:,:,k_integ),Q_const,R_const,Wirr,fb_k,cycleT_VNS(k_integ),Crb_k_integ,position_integ,RbbVision(:,:,k_integ),TbbVision(:,k_integ),isTbb_last,Crb_SINSpre,position_SINSpre,isDebudMode,calZMethod );
        
        if isDebudMode.trueGyroDrift==1
            X(10:12,k_integ+1) = pg ;
        end
        if isDebudMode.trueAccDrift==1
            X(13:15,k_integ+1) = pa ;
        end        
         % EKF 校核，存储中间量（检查调试时看）
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
        end                    

        % 保存估计值
        % 保存误差估计值
        % X=[dat dv dr gyroDrift accDrift dat_last dr_last] 
        dAngleEsm(:,k_integ+1) = X(1:3,k_integ+1); 
        dVelocityEsm(:,k_integ+1) = X(4:6,k_integ+1);
        dPositionEsm(:,k_integ+1) = X(7:9,k_integ+1);       
        gyroDrift(:,k_integ+1) = X(10:12,k_integ+1) ;
        accDrift(:,k_integ+1) = X(13:15,k_integ+1) ;
        % 保存估计均方误差
        P_new_diag = sqrt(diag(P(:,:,k_integ+1))) ;  % P阵对角元素
        dAngleEsmP(:,k_integ+1) = P_new_diag(1:3);
        dVelocityEsmP(:,k_integ+1) = P_new_diag(4:6);
        dPositionEsmP(:,k_integ+1) = P_new_diag(7:9);
        gyroDriftP(:,k_integ+1) = P_new_diag(10:12);
        accDriftP(:,k_integ+1) = P_new_diag(13:15);
        % 由计算世界坐标系到真实世界坐标系的旋转矩阵
        Crc = FCbn(dAngleEsm(:,k_integ+1));           % 认为X(1:3,k_integ+1)是从计算机系c（SINS计算用r），转动到真实r坐标系的角度
        %%%%%%%%%%%%%%%%%%%%%%%  状态重置  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 姿态误差、速度误差、位置误差 均已补偿，由于它们代表的是误差，因此，将相应的状态量置0
        X(1:9,k_integ+1) = 0;       % 修正状态
        % 陀螺和加计漂移 就算补偿了，状态量也还不变
        if isDebudMode.isResetDrift
           X(10:15,k_integ+1) = 0 ;
        end
        % 增广状态的定义是上一时刻的 位置误差和姿态误差，为0，根据定义重置
        X(16:18,k_integ+1) = zeros(3,1);%
        X(19:21,k_integ+1) = zeros(3,1);% 
        
        %% 更新轨迹：用状态量修正 SINS解析递推 的姿态、速度、位置

        INTGpos(:,k_integ+1) = SINSposition(:,t_imu+1) - dPositionEsm(:,k_integ+1);  
        INTGvel(:,k_integ+1)  = SINSvel(:,t_imu+1) - dVelocityEsm(:,k_integ+1);
        Ccb = Crb ;         % 上一时刻的 r 实际为 c
        Crb = Ccb*Crc ;     
        opintions.headingScope=180;
        INTGatt(:,k_integ+1) = GetAttitude(Crb,'rad',opintions);
        
      %  q=SINSQ(:,t_imu+1)-FCnbtoQ(Crb)
        % 更新SINS的轨迹
        SINSQ(:,t_imu+1)  = FCnbtoQ(Crb);
        SINSvel(:,t_imu+1) = INTGvel(:,k_integ+1) ;
        SINSposition(:,t_imu+1) = INTGpos(:,k_integ+1) ;
        
%         positione0 = Cre * SINSposition(:,t_imu+1) + positionr; % 将发射点惯性系中的位置转化到初始时刻地固系
%         SINSpositionition_d(:,t_imu+1) = FZJtoJW(positione0,planet);
%         Cen = FCen(SINSpositionition_d(1,t_imu+1),SINSpositionition_d(2,t_imu+1));
        % 不需要更新 gr 则不需要更新 Cen
        
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
    if ~isempty(true_acc_r)
        SINS_accError  =SINSacc_r-true_acc_r(:,1:length(SINSacc_r)) ; % SINS的加速度误差
    else
        SINS_accError = [] ;
    end
    accDriftError = accDrift-repmat(pa,1,integnum) ;        % 组合导航的加计估计误差
    gyroDriftError = gyroDrift-repmat(pg,1,integnum) ;      % 组合导航的陀螺估计误差
    % 计算空间二维/三维位置误差及相对值
% dbstop in CalPosErrorIndex_route
    errorStr = CalPosErrorIndex_route( true_position(:,1:imuNum),INTGPositionError,INTGAttitudeError*180/pi*3600,INTGpos );
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
Qstr = sprintf('%0.3g  ',diag(Q_const)');
R0str = sprintf('%0.3g  ',diag(R_const)');

if isCompensateDrift==1
    isCompensateGyroDriftStr = sprintf('%d ',isCompensateGyroDrift);
    isCompensateAccDriftStr = sprintf('%d ',isComensateAccDrift);
    isCompensateDriftStr = sprintf('进行：IMU数据的常值漂移补偿。 陀螺:%s\t加计:%s',isCompensateGyroDriftStr,isCompensateAccDriftStr);
else
    isCompensateDriftStr = '不进行：IMU数据的常值漂移补偿';
end

recordStr = sprintf('%s\n滤波参数：\n\tX(0)=( %s )\n\tP(0)=( %s )\n\tQk=( %s )\n\tR(0)=( %s )\n%s\n',...
    recordStr,X0str,P0str,Qstr,R0str,isCompensateDriftStr);
if isTrueX0==1
    recordStr = sprintf('%s IMU常值漂移初值给 真值 （仿真真值/实验经验值）: pa=%d na=%d (ug), pg=%0.3f ng=%0.3f (°/h)\n',recordStr,pa(1)/(gp*1e-6),na(1)/(gp*1e-6),pg(1)*180/pi*3600,ng(1)*180/pi*3600 ) ;
else
    recordStr = sprintf('%s IMU常值漂移初值 置0\n',recordStr) ;
end
time=zeros(1,integnum);
for i=1:integnum
    time(i)=(i-1)/frequency_VO/60;
end
newInformation = Zinteg-Zinteg_pre;
%% 保存结果为特定格式
% INS_VNS_NavResult = saveINS_VNS_NavResult_subplot(integFre,combineFre,imu_fre,projectName,gp,isKnowTrue,trueTraeFre,...
%     INTGpos,INTGvel,INTGatt,dPositionEsm,dVelocityEsm,dAngleEsm,accDrift,gyroDrift,INTGPositionError,true_position,...
%     INTGAttitudeError,true_attitude,INTGVelocityError,accDriftError,gyroDriftError,dAngleEsmP,dVelocityEsmP,dPositionEsmP,...
%     gyroDriftP,accDriftP,SINS_accError,X_pre_error,X_correct,Zinteg_error,Zinteg_pre,Zinteg);
INS_VNS_NavResult = saveResult_SINSerror_subQbbsubTbb(integFre,combineFre,imu_fre,projectName,gp,isKnowTrue,trueTraeFre,...
    INTGpos,INTGvel,INTGacc,INTGatt,accDrift,gyroDrift,INTGPositionError,true_position,...
    INTGAttitudeError,true_attitude,INTGVelocityError,[],accDriftError,gyroDriftError,dAngleEsmP,dVelocityEsmP,dPositionEsmP,...
    gyroDriftP,accDriftP,SINS_accError,X_correct,Zinteg_error,Zinteg_pre,Zinteg ) ;
save([resultPath,'\INS_VNS_',projectName,'result.mat'],'INS_VNS_NavResult')
disp('INS_VNS_ZdRdT 函数运行结束')
disp('滤波调试信息已输出到基本工作空间')

if exist('R_INS','var')
    check.R_INS=R_INS_save;  check.T_INS=T_INS_save;  check.R_VNS=R_VNS_save;  check.R_INS=R_INS_save;  check.T_VNS=T_VNS_save;  check.newInformation=newInformation;
    check.X_correct=X_correct;  check.X_pre_error=X_pre_error;
    save check check
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
    noteStr = [pwd,'\(单独)SINSerror_subQ_subT.txt'];
    noteStr_old = [pwd,'\old(单独)_SINSerror_subQ_subT.txt'];
    if exist(noteStr,'file')
       copyfile(noteStr,noteStr_old); 
       open(noteStr_old);
    end
    fid = fopen(noteStr, 'w+');
    RecodeInput (fid,visualInputData,imuInputData,trueTrace);
    fprintf(fid,'\nINS_VNS_%s 解算误差：\n',integMethodDisplay);
    fprintf(fid,'%s',recordStr);
    fclose(fid);
    open(noteStr)
    if exist('VOResult','var')
       save([pwd,'\navResult\VOResult.mat'],'VOResult'); 
    end
    if exist('SINS_Result','var')
       save([pwd,'\navResult\SINS_Result.mat'],'SINS_Result'); 
    end
    
    disp('调出 ResultDisplay ，直接在 base 空间执行 ResultDisplay()')
	%ResultDisplay()
        
end
    figure('name','组合导航轨迹')
    INTGpos_length = length(INTGpos);
    trueTraceValidLength = fix((INTGpos_length-1)*trueTraeFre/frequency_VO) +1 ;
    true_position_valid = true_position(:,1:trueTraceValidLength);
    hold on
    plot(true_position_valid(1,:),true_position_valid(2,:),'--r');
    plot(INTGpos(1,:),INTGpos(2,:),'-.g');
   
    legend('trueTrace','INTGpos');
    saveas(gcf,'组合导航轨迹.fig')

       
function [ P_ini,Q_const,R_ini,NavFilterParameter ] = GetFilterParameter_SINSerror_subQsubT( pg,ng,pa,na,NavFilterParameter )
%% 导入初始滤波参数 P、Q、R_const
% if isfield(NavFilterParameter,'P_ini_augment_dRdT')
%     P_ini = NavFilterParameter.P_ini_augment_dRdT ;
% else
%     
% end
    szj1 = 1/3600*pi/180 * 2;
    szj2 = 1/3600*pi/180 * 2;
    szj3 = 1/3600*pi/180 * 2;
    pg = [ 1 1 1 ]*pi/180/3600 * 0.1 ;        % 
    pa = [ 1 1 1 ]*1e-6*9.8 *0.1 ;
    
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
%     Q_const = NavFilterParameter.Q_ini_augment_dRdT ;
% else
% end
   %%% Q_const = diag([(ng(1))^2,(ng(2))^2,(ng(3))^2,(na(1))^2,(na(2))^2,(na(3))^2]);      % ???
  %  Q_const = diag([(ng(1))^2+1e-19,(ng(2))^2+1e-19,(ng(3))^2+1e-19,(na(1))^2+1e-15,(na(2))^2+1e-15,(na(3))^2+1e-15]);
    
%   Q_const = diag([  2e-19 2e-19 2e-19 ...         % 失准角微分方程
%                     2e-8 2e-8 2e-8...            % 速度微分方程
%                     0 0 0 ...                       % 位置微分方程
%                     1e-37 1e-37 1e-37 ...           % 陀螺常值微分方程
%                     0 0 0  ]);                      % 加计常值微分方程
   
   %%% kitti
   Q_const = diag([  2e-19 2e-19 2e-19 ...         % 失准角微分方程
                    2e-8 2e-8 2e-8...            % 速度微分方程
                    0 0 0 ...                       % 位置微分方程
                    1e-37 1e-37 1e-37 ...           % 陀螺常值微分方程
                    0 0 0  ]);                      % 加计常值微分方程
                
% Q_const = diag([      10e-12 10e-10 10e-10 ...         % 失准角微分方程
%                     10e-15 10e-15 10e-24...         % 速度微分方程
%                     10e-17 10e-17 10e-27...         % 位置微分方程
%                     0 0 0 ...         % 陀螺常值微分方程
%                     0 0 0  ]);       % 加计常值微分方程
     NavFilterParameter.Q_ini_augment_dRdT = sprintf('%1.1e ',Q_const) ;



if isfield(NavFilterParameter,'R_ini_augment_dRdT')
    R_list_input = {NavFilterParameter.R_ini_augment_dRdT} ;
else
    R_list_input = [];
end
R_list = [R_list_input,{'[[1 1 1]*1e-5  [1 1 1]*1e-5]'...
                        '[[1 1 1]*1e-1  [1 1 1]*1e-12]'...  % kitti
                        '[4e-004,4e-004,4e-004,6e-007,6e-007,6e-007  ]',...     % 圆周360m Rbb 206"
                        '[1e-004,5e-004,8e-004,6e-007,6e-007,6e-007  ]',...
                        '[1e-005,1e-005,1e-003,6e-007,6e-007,6e-006  ]'....
                        '[1e-006,1e-006,8e-006,6e-004,6e-004,6e-004 ]',...      % 向前360m Tbb 0.02m
                        '[4e-004,4e-004,4e-004,6e-007,6e-007,6e-007  ]'}];      % 圆周360m Rbb 20.6"

[Selection,ok] = listdlg('PromptString','噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT:','SelectionMode','single','ListSize',[350,100],'ListString',R_list);
if ok==0    
    Selection = 1 ;
end
answer = inputdlg('噪声方程阵R(前R[3x3]后T[3x1])-subQ\_subT                     .','R_ini',1,R_list(Selection));
R_ini = diag(eval(answer{1})) ;   % R_const
NavFilterParameter.R_ini_augment_dRdT = answer{1} ;

function newStr = strToDis(Str)
% 在 _的前面加 \
k_new=1;
newStr='';
for k=1:length(Str)
    if strcmp(Str(k),'_')==1
        newStr(k_new)='\';
        k_new = k_new+1 ;
        newStr(k_new)=Str(k) ;
        k_new = k_new+1 ;
    else
        newStr(k_new)=Str(k) ;
        k_new = k_new+1 ;
    end
end
