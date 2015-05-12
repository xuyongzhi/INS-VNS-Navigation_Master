% buaa xyz 2014.1.9

% 月球表面 世界坐标系下纯惯导解算：以初始时刻的xyz系作为导航系
% 源至 白鑫贝（2013.1.21-SINSr_addnoise）

function [SINS_Result,errorStr] = main_SINSNav_i( imuInputData,trueTrace,visualInputData )
format long
disp('函数 main_SINSNav 开始运行')
tic

if ~exist('imuInputData','var')
    % 独立运行
    clc
    clear all 
    close all
    %% 再此更改所需添加的数据名称：对应相关的参数设置方案    
    % load('gyro_norm.mat')            % 静止30min，仅陀螺随机噪声
    % load('gyro_const.mat')           % 静止30min，仅陀螺常值噪声
    % load('gyro_const_norm.mat')      % 静止30min，仅陀螺随机+常值噪声
    % load('acc_norm.mat')            % 静止30min，仅加计随机噪声
    % load('acc_const.mat')           % 静止30min，仅加计常值噪声
    %  load('静止30min-IMU噪声-随机RT.mat')     
    % load('静止30min-IMU噪声-真实RT.mat')
   % load('静止3min-IMU噪声-RT随机')
   % load('静止3min-IMU噪声-RT真实')
   
   load('i-直线5min-IMU噪声-随机RT')
    % load('i-直线5min-IMU噪声-真实RT')
    % load('i-静止5min-IMU噪声-随机RT')
    % load('i-静止5min-IMU噪声-真实RT')
    % load('i-圆弧5min-IMU噪声-随机RT')
    % load('i-圆弧5min-IMU噪声-真实RT')
    
    isAlone = 1;
else
    isAlone = 0;
end

%% 导入数据
% （1）IMU数据
wib_INSm = imuInputData.wib;
f_INSm = imuInputData.f;
imu_fre = imuInputData.frequency; % Hz
%% 真实轨迹的参数
if ~exist('trueTrace','var')
    trueTrace = [];
end
[planet,isKnowTrue,initialPosition_e,initialVelocity_r,initialAttitude_r,trueTraeFre,true_position,true_attitude,true_velocity,true_acc_r] = GetFromTrueTrace( trueTrace );
true_attitude = trueTrace.attitude_i ;
imu_T=1/imu_fre;     % sec
runTimeNum=size(f_INSm,2);
[pa,na,pg,ng,~] = GetIMUdrift( imuInputData,planet ) ; % pa(加计常值偏置),na（加计随机漂移）,pg(陀螺常值偏置),ng（陀螺随机漂移）
% (1) 导入纯视觉导航仿真解算的的中间结果，包括两个数据:Rbb[例3*3*127]、Tbb[例3*127]
VisualOut_RT=visualInputData.VisualRT;
RbbVision = VisualOut_RT.Rbb;
%TbbVision = VisualOut_RT.Tbb;
frequency_VO = visualInputData.frequency;
integFre = frequency_VO ;
cycleT_VNS = 1/frequency_VO;  % 视觉数据周期/滤波周期
k_integ = 0 ;
integnum = fix(runTimeNum*integFre/imu_fre) ;

RinsAngle = zeros(3,integnum-1);
RvnsAngle = zeros(3,integnum-1);
trueRbbAngle = zeros(3,integnum-1);
RinsAngleError = zeros(3,integnum-1);
RvnsAngleError = zeros(3,integnum-1);
%% 2014.4.15 宁老师的想法：利用Rbb直接估计陀螺常值漂移
%%%%%%%%%%%  A：相减法
gyroDriftEsmA = zeros(3,integnum);          % 陀螺漂移估计值
gyroDriftEsmAError = zeros(3,integnum);          % 陀螺漂移估计值误差
P_gyroDRbbKFA = zeros(3,3,integnum);
Q_gyroDRbbKFA = diag([1 1 1]*0);
R_gyroDRbbKFA = diag([1 1 1]*1e-8);

isTrueX0=0;
if isTrueX0==1
    gyroDriftEsmA(:,1) = pg ;
    P_gyroDRbbKFA(:,:,1) = diag([ (pg(1))^2+1e-12,(pg(2))^2+1e-12,(pg(3))^2+1e-12 ]);
else
    pgError0 = [0.1;0.1;0.1]*pi/180/3600 ;  % 陀螺常值漂移 状态分量初值误差
    % gyroDriftEsmA(:,1)=  pg-pgError0 ;
    gyroDriftEsmA(:,1) = zeros(3,1) ;
    P_gyroDRbbKFA(:,:,1) = diag([ (pg(1))^2+1e-5,(pg(2))^2+1e-5,(pg(3))^2+1e-5 ]);
end
%%%%%%%%%%%  B：相乘法
gyroDriftEsmB = zeros(3,integnum-1);          % 陀螺漂移估计值
vnsdRbbEsmB = zeros(3,integnum-1);        % Vns的Rbb常值误差估计
gyroDriftEsmBError = zeros(3,integnum-1);          % 陀螺漂移估计值误差
vnsdRbbEsmBError = zeros(3,integnum-1);        % Vns的Rbb常值误差估计误差
P_gyroDRbbUKFB = zeros(6,6,integnum-1);
Q_gyroDRbbUKFB = diag([1 1 1 1 1 1]*0);
R_gyroDRbbUKFB = diag([1 1 1 1]*1e-8);
if isTrueX0==1
    gyroDriftEsmB(:,1) = pg ;
    P_gyroDRbbUKFB(:,:,1) = diag([ (pg(1))^2+1e-8,(pg(2))^2+1e-8,(pg(3))^2+1e-6 ]);
else
    pgError0 = [0.1;0.1;0.1]*pi/180/3600 ;  % 陀螺常值漂移 状态分量初值误差
    % gyroDriftEsmB(:,1)=  pg-pgError0 ;
    gyroDriftEsmB(:,1)= zeros(3,1) ;
    P_gyroDRbbUKFB(:,:,1) = diag([ (pg(1))^2+1e-5,(pg(2))^2+1e-5,(pg(3))^2+1e-5 ,  (pg(1))^2+1e-5,(pg(2))^2+1e-5,(pg(3))^2+1e-5 ]);
end
vnsdRbbEsmB(:,1) = [0;0;0] ;
%%%%%%%%%%%  DRR：两时刻相乘法
gyroDriftEsmDRR = zeros(3,integnum);          % 陀螺漂移估计值
gyroDriftEsmDRRError = zeros(3,integnum);          % 陀螺漂移估计值误差
P_gyroDRRUKF = zeros(3,3,integnum);
Q_gyroDRRUKF = diag([1 1 1]*0);
R_gyroDRRUKF = diag([1 1 1 1]*1e-7);
if isTrueX0==1
    gyroDriftEsmDRR(:,1) = pg ;
    P_gyroDRRUKF(:,:,1) = diag([ (pg(1))^2+1e-8,(pg(2))^2+1e-8,(pg(3))^2+1e-6 ]);
else
    pgError0 = [0.1;0.1;0.1]*pi/180/3600 ;  % 陀螺常值漂移 状态分量初值误差
  %  gyroDriftEsmDRR(:,1)=  pg-pgError0 ;
    gyroDriftEsmDRR(:,1)=  zeros(3,1) ;
    P_gyroDRRUKF(:,:,1) = diag([ (pg(1))^2+1e-5,(pg(2))^2+1e-5,(pg(3))^2+1e-5 ]);
end

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
% 
% Wiee=[0;0;wip];
% 
% earthConst = getEarthCosnt;
% g0_e = earthConst.g0 ;  % 用于给定仿真的加计噪声

%% 初始条件

% 地理系导航参数
% attitude_t=zeros(3,runTimeNum);
% velocity_t=zeros(3,runTimeNum);
% position_e=zeros(3,runTimeNum); % 经度（rad）,纬度（rad）,高度

% 世界坐标系导航参数
velocity_r=zeros(3,runTimeNum);
position_r=zeros(3,runTimeNum);
attitude_r = zeros(3,runTimeNum);
acc_r = zeros(3,runTimeNum);
% % n 是当地地理坐标系， r 是世界坐标系（初始时刻地理坐标系）
% 
% position_e(:,1)=initialPosition_e;  %初始位置    position_e(1): 经度   position_e(2):纬度  position_e(3):高度
% position_ini_e = FJWtoZJ(position_e(:,1),planet);  %初始时刻地固坐标系中的位置 （x,y,z/m）
attitude_r(:,1)=initialAttitude_r + [0/3600/180*pi;0/3600/180*pi;0/3600/180*pi];    %初始姿态 sita ,gama ,fai
Cbi=FCbn(attitude_r(:,1));
Cib=Cbi';
% Cen=FCen(position_e(1,1),position_e(2,1));       %calculate Cen
% Cer = Cen; % 发射点惯性系相对于初始时刻地固系的旋转矩阵
% Cre = Cer';
% Crb = Cnb;
% Cbr = Crb';
Cib_ins = Cib ;
% % velocity_t(:,1) = Cbn * initialVelocity_r;
% velocity_r(:,1) = Cbr * initialVelocity_r;
% Wirr = Cer * Wiee;

% 根据初始姿态矩阵Crb计算初始姿态四元数
Q0 = FCnbtoQ(Cib);
R_INS = eye(3);
R_VNS = eye(3);
Wibb_last = wib_INSm(:,1) ;

wh = waitbar(0,'纯惯导解算中...');
for t = 1:runTimeNum-1
    
    Wibb = wib_INSm(:,t) ;
    Q0=Q0+0.5*imu_T*[    0    ,-Wibb(1,1),-Wibb(2,1),-Wibb(3,1);
                 Wibb(1,1),     0    , Wibb(3,1),-Wibb(2,1);
                 Wibb(2,1),-Wibb(3,1),     0    , Wibb(1,1);
                 Wibb(3,1), Wibb(2,1),-Wibb(1,1),     0    ]*Q0;
    Q0=Q0/norm(Q0);
    Cib = FQtoCnb(Q0);
%     Cbr = Crb';
%     g = gp * (1+gk1*sin(position_e(2,t))^2-gk2*sin(2*position_e(2,t))^2);
%     gn = [0;0;-g];
%     Cen = FCen(position_e(1,t),position_e(2,t));
%     Cnr = Cer * Cen';
%     Cnb = Crb * Cnr;
%     gb = Cnb * gn;
%     gr = Cbr * gb;
%   	 %%%%%%%%%%% 速度方程 %%%%%%%%%%            
%     a_rbr = Cbr * f_INSm(:,t) - getCrossMarix( 2*Wirr )*velocity_r(:,t) + gr;      
%     acc_r(:,t) = a_rbr;
%     
%     velocity_r(:,t+1) = velocity_r(:,t) + a_rbr * imu_T;
%     position_r(:,t+1) = position_r(:,t) + velocity_r(:,t+1) * imu_T;
%     positione0 = Cre * position_r(:,t+1) + position_ini_e; % 将发射点惯性系中的位置转化到初始时刻地固系
%     position_e(:,t+1) = FZJtoJW(positione0,planet);    % 再转化为经纬高度

    opintions.headingScope=180;
    attitude_r(:,t+1) = GetAttitude(Cib,'rad',opintions);
    
    
    %%%%%%%% 陀螺常值漂移标定
    %   t=21开始组合
    if mod(t-1,cycleT_VNS/imu_T)==0 && t>1
        k_integ = k_integ+1;
        Cib_ins_last = Cib_ins ;
        Cib_ins = Cib ;
        
        R_INS_last = R_INS ;
        R_VNS_last = R_VNS ;
        R_INS = Cib_ins * Cib_ins_last';
        R_VNS = RbbVision(:,:,k_integ);        
        
        % DRbb标定陀螺常值漂移法A
        [gyroDriftEsmA(:,k_integ+1),P_gyroDRbbKFA(:,:,k_integ+1)] = gyroDRbbAKF(R_INS,R_VNS,cycleT_VNS,gyroDriftEsmA(:,k_integ),P_gyroDRbbKFA(:,:,k_integ),Q_gyroDRbbKFA,R_gyroDRbbKFA) ;
        % DRbb标定陀螺常值漂移法B
     %     dbstop in gyroDRbbBUKF
        [gyroDriftEsmB(:,k_integ+1),vnsdRbbEsmB(:,k_integ+1),P_gyroDRbbUKFB(:,:,k_integ+1)] = c_gyroDRbbBUKF(R_INS,R_VNS,cycleT_VNS,gyroDriftEsmB(:,k_integ),vnsdRbbEsmB(:,k_integ),P_gyroDRbbUKFB(:,:,k_integ),Q_gyroDRbbUKFB,R_gyroDRbbUKFB,Wibb) ;
        % DRR
    %    dbstop in gyroDRRUKF
        [gyroDriftEsmDRR(:,k_integ+1),P_gyroDRRUKF(:,:,k_integ+1)] = gyroDRRUKF(R_INS_last,R_VNS_last,R_INS,R_VNS,cycleT_VNS,gyroDriftEsmDRR(:,k_integ),P_gyroDRRUKF(:,:,k_integ),Q_gyroDRRUKF,R_gyroDRRUKF,Wibb_last,Wibb) ;
        Wibb_last = Wibb ;
        
        opintions.headingScope = 180 ;
        RinsAngle(:,k_integ) = GetAttitude(R_INS,'rad',opintions) ;
        RvnsAngle(:,k_integ) = GetAttitude(R_VNS,'rad',opintions) ;
        trueFCnb = FCbn(true_attitude(:,t+1))' ;
        trueFCnb_last = FCbn(true_attitude(:,t+1-cycleT_VNS/imu_T))' ;
        trueRbb = trueFCnb*trueFCnb_last' ;
        trueRbbAngle(:,k_integ) = GetAttitude(trueRbb,'rad',opintions) ;
        
        RinsAngleError(:,k_integ) = GetAttitude(R_INS*trueRbb','rad',opintions) ;
        RvnsAngleError(:,k_integ) = GetAttitude(R_VNS*trueRbb','rad',opintions) ;        
        
        
    end
    if mod(t,fix(runTimeNum/100))==0
        waitbar(t/runTimeNum)
    end
end
close(wh)

%% 统一去掉最后一个数据
runTimeNum = runTimeNum-1;  
time=zeros(1,runTimeNum);
for i=1:runTimeNum
    time(i)=i/imu_fre/60;
end
% position_r = position_r(:,1:runTimeNum);
% attitude_r = attitude_r(:,1:runTimeNum);
% velocity_r = velocity_r(:,1:runTimeNum);
% acc_r = acc_r(:,1:runTimeNum);

%% 已知真实：计算误差
if  isKnowTrue==1
    % 计算组合数据有效长度
    lengthArrayOld = [length(position_r),length(true_position)];
    frequencyArray = [imu_fre,trueTraeFre];
    [validLenthArray,combineK,combineLength,combineFre] = GetValidLength(lengthArrayOld,frequencyArray);
    SINSPositionError = zeros(3,combineLength); % SINS的位置误差
    SINSAttitudeError = zeros(3,combineLength); % SINS的姿态误差
    SINSVelocityError = zeros(3,combineLength); % SINS的速度误差
    SINSAccError = zeros(3,combineLength);      % SINS的加速度误差
    for k=1:combineLength
        k_true = fix((k-1)*trueTraeFre/combineFre)+1 ;
        k_imu = fix((k-1)*imu_fre/combineFre)+1;
        SINSPositionError(:,k) = position_r(:,k_imu)-true_position(:,k_true) ;
        SINSAttitudeError(:,k) = attitude_r(:,k_imu)-true_attitude(:,k_true);
        SINSAttitudeError(3,k) = YawErrorAdjust(SINSAttitudeError(3,k),'rad') ;
        SINSVelocityError(:,k) = velocity_r(:,k_imu)-true_velocity(:,k_true);
        SINSAccError(:,k) = acc_r(:,k_imu)-true_acc_r(:,k_true);
    end    
    gyroDriftEsmAError = gyroDriftEsmA-repmat(pg,1,integnum) ;
    gyroDriftEsmBError = gyroDriftEsmB-repmat(pg,1,integnum) ;
    gyroDriftEsmDRRError = gyroDriftEsmDRR - repmat(pg,1,integnum) ;
    
    errorStr = CalPosErrorIndex( true_position,SINSPositionError,SINSAttitudeError*180/pi*3600 );
    
    m=mean(RvnsAngleError,2);
    vnsdRbbEsmBError = vnsdRbbEsmB-repmat(m,1,size(vnsdRbbEsmB,2)) ;
else
    errorStr = '\n真实未知';
end

diagP_gyroDRbbKFA = zeros(3,length(P_gyroDRbbKFA));
diagP_gyroDRbbUKFB = zeros(6,length(P_gyroDRbbUKFB));
for k=1:length(P_gyroDRbbKFA)
    diagP_gyroDRbbKFA(:,k) = diag(P_gyroDRbbKFA(:,:,k)) ;
    diagP_gyroDRbbUKFB(:,k) = diag(P_gyroDRbbUKFB(:,:,k)) ;
end
%% 记录
gyroDriftEsmAStartErrorStr = sprintf('%0.3g  ',(gyroDriftEsmA(:,1)-pg)*180/pi*3600);
errorStr = sprintf('%s\n陀螺常值漂移初值误差：%s °/h\n',errorStr,gyroDriftEsmAStartErrorStr) ;
meanRinsAngleErrorStr = sprintf('%0.3g  ',mean(RinsAngleError,2)*180/pi*3600);
stdRinsAngleErrorStr = sprintf('%0.3g  ',std(RinsAngleError,0,2)*180/pi*3600);
errorStr = sprintf('%sR_INS欧拉角误差平均值：%s 〞\nR_INS欧拉角误差方差：%s 〞\n',errorStr,meanRinsAngleErrorStr,stdRinsAngleErrorStr) ;
meanRvnsAngleErrorStr = sprintf('%0.3g  ',mean(RvnsAngleError,2)*180/pi*3600);
stdRvnsAngleErrorStr = sprintf('%0.3g  ',std(RvnsAngleError,0,2)*180/pi*3600);
errorStr = sprintf('%sR_VNS欧拉角误差平均值：%s 〞\nR_VNS欧拉角误差方差：%s 〞\n',errorStr,meanRvnsAngleErrorStr,stdRvnsAngleErrorStr) ;


gyroDRbbAKFDriftEndErrorStr = sprintf('%0.3g  ',gyroDriftEsmAError(:,length(gyroDriftEsmAError))*180/pi*3600);
errorStr = sprintf('%s\nAKF法陀螺常值漂移标定\n\tAKF陀螺常值漂移标定误差：(%s) °/h\n',errorStr,gyroDRbbAKFDriftEndErrorStr) ;
Q_gyroDRbbKFAstr = sprintf('%0.3g  ',diag(Q_gyroDRbbKFA)');
R_gyroDRbbKFAstr = sprintf('%0.3g  ',diag(R_gyroDRbbKFA)');
P_gyroDRbbKFA0str = sprintf('%0.3g  ',diag(P_gyroDRbbKFA(:,:,1))');
gyroDriftEsmA0str = sprintf('%0.3g  ',gyroDriftEsmA(:,1));
errorStr = sprintf('%s\tAKF滤波参数：\n\tgyroDriftEsmA(0)=( %s )\n\tP_gyroDRbbUKFB_0=( %s )\n\tQ_gyroDRbbKFA=( %s )\n\tR_gyroDRbbKFA=( %s )\n',...
    errorStr,gyroDriftEsmA0str,P_gyroDRbbKFA0str,Q_gyroDRbbKFAstr,R_gyroDRbbKFAstr);

gyroDRbbBUKFDriftEndErrorStr = sprintf('%0.3g  ',gyroDriftEsmBError(:,length(gyroDriftEsmBError))*180/pi*3600);
vnsdRbbEsmBErrorEndStr = sprintf('%0.3g  ',vnsdRbbEsmBError(:,length(vnsdRbbEsmBError))*180/pi*3600);
errorStr = sprintf('%s\nBUKF法陀螺常值漂移标定\n\tBUKF陀螺常值漂移标定误差：(%s) °/h\n\tVNS_Rbb误差角最终估计误差:(%s)〞\n',errorStr,gyroDRbbBUKFDriftEndErrorStr,vnsdRbbEsmBErrorEndStr) ;
Q_gyroDRbbUKFBstr = sprintf('%0.3g  ',diag(Q_gyroDRbbUKFB)');
R_gyroDRbbUKFBstr = sprintf('%0.3g  ',diag(R_gyroDRbbUKFB)');
P_gyroDRbbUKFB0str = sprintf('%0.3g  ',diag(P_gyroDRbbUKFB(:,:,1))');
gyroDriftEsmB0str = sprintf('%0.3g  ',gyroDriftEsmB(:,1));
errorStr = sprintf('%s\tBUKF滤波参数：\n\tgyroDriftEsmB(0)=( %s )\n\tP_gyroDRbbUKFB_0=( %s )\n\tQ_gyroDRbbUKFB=( %s )\n\tR_gyroDRbbUKFB=( %s )\n',...
    errorStr,gyroDriftEsmB0str,P_gyroDRbbUKFB0str,Q_gyroDRbbUKFBstr,R_gyroDRbbUKFBstr);

gyroDRbbBUKFDriftEndErrorStr = sprintf('%0.3g  ',gyroDriftEsmDRRError(:,length(gyroDriftEsmDRRError))*180/pi*3600);
errorStr = sprintf('%s\nDRR法陀螺常值漂移标定\n\tDRR陀螺常值漂移标定误差：(%s) °/h\n',errorStr,gyroDRbbBUKFDriftEndErrorStr) ;

%% 输出 
% 存储为特定格式：每个变量一个细胞，包含成员：data，name,comment 和 dataFlag,frequency,project,subName
resultNum = 23 ;
SINS_Result = cell(1,resultNum);

% 有4个共同的成员
for j=1:resultNum
    SINS_Result{j}.dataFlag = 'xyz result display format';
    SINS_Result{j}.frequency = imu_fre ;
    SINS_Result{j}.project = 'SINS';
    SINS_Result{j}.subName = {'x','y','z'};
end

SINS_Result{1}.data = position_r ;
SINS_Result{1}.name = 'position(m)';
SINS_Result{1}.comment = '位置';

SINS_Result{2}.data = velocity_r ;
SINS_Result{2}.name = 'velocity_r(m/s)';
SINS_Result{2}.comment = '速度';

SINS_Result{3}.data = attitude_r*180/pi ;
SINS_Result{3}.name = 'attitude_r(°)';
SINS_Result{3}.comment = '姿态';
SINS_Result{3}.subName = {'俯仰','横滚','航向'};

SINS_Result{4}.data = acc_r ;
SINS_Result{4}.name ='acc_r(m/s^2)';
SINS_Result{4}.comment = '加速度';

SINS_Result{5}.data = gyroDriftEsmA*180/pi*3600 ;     % rad/s 转化为 °/h
SINS_Result{5}.name = 'gyroDriftEsmA(°/h)';
SINS_Result{5}.comment = 'DRbb_A陀螺常值漂移估计';
SINS_Result{5}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
meanGyroDrift = mean(gyroDriftEsmA*180/pi*3600,2);  % 陀螺常值漂移估计均值
meanGyroDriftText{1} = '均值';
meanGyroDriftText{2} = sprintf('x：%0.3f°/h',meanGyroDrift(1));
meanGyroDriftText{3} = sprintf('y：%0.3f°/h',meanGyroDrift(2));
meanGyroDriftText{4} = sprintf('z：%0.3f°/h',meanGyroDrift(3));
SINS_Result{5}.text = meanGyroDriftText ;
SINS_Result{5}.project = 'GyroCalib_A';

SINS_Result{6}.data = gyroDriftEsmB*180/pi*3600 ;     % rad/s 转化为 °/h
SINS_Result{6}.name = 'gyroDriftEsmB(°/h)';
SINS_Result{6}.comment = 'DRbb_B陀螺常值漂移估计';
SINS_Result{6}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
meanGyroDrift = mean(gyroDriftEsmB*180/pi*3600,2);  % 陀螺常值漂移估计均值
meanGyroDriftText{1} = '均值';
meanGyroDriftText{2} = sprintf('x：%0.3f°/h',meanGyroDrift(1));
meanGyroDriftText{3} = sprintf('y：%0.3f°/h',meanGyroDrift(2));
meanGyroDriftText{4} = sprintf('z：%0.3f°/h',meanGyroDrift(3));
SINS_Result{6}.text = meanGyroDriftText ;
SINS_Result{6}.project = 'GyroCalib_B';

SINS_Result{7}.data = diagP_gyroDRbbKFA ;    
SINS_Result{7}.name = 'diagP_gyroDRbbKFA';
SINS_Result{7}.comment = 'DRbb_A陀螺常值漂移估计方差';
SINS_Result{7}.frequency = integFre ;
SINS_Result{7}.project = 'GyroCalib_A';

SINS_Result{8}.data = diagP_gyroDRbbUKFB ;    
SINS_Result{8}.name = 'diagP_gyroDRbbUKFB';
SINS_Result{8}.comment = 'DRbb_B陀螺常值漂移估计方差';
SINS_Result{8}.frequency = integFre ;
SINS_Result{8}.project = 'GyroCalib_B';

SINS_Result{9}.data = RinsAngle ;    
SINS_Result{9}.name = 'RinsAngle';
SINS_Result{9}.comment = 'R_INS欧拉角(rad)';
SINS_Result{9}.frequency = integFre ;

SINS_Result{10}.data = RvnsAngle ;    
SINS_Result{10}.name = 'RvnsAngle';
SINS_Result{10}.comment = 'R_VNS欧拉角(rad)';
SINS_Result{10}.frequency = integFre ;

SINS_Result{11}.data = trueRbbAngle ;    
SINS_Result{11}.name = 'trueRbbAngle';
SINS_Result{11}.comment = 'Rbb真实欧拉角(rad)';
SINS_Result{11}.frequency = integFre ;

SINS_Result{12}.data = RvnsAngleError*180/pi*3600 ;    
SINS_Result{12}.name = 'RvnsAngleError';
SINS_Result{12}.comment = 'R_VNS欧拉角误差(〞)';
SINS_Result{12}.frequency = integFre ;

SINS_Result{13}.data = RinsAngleError*180/pi*3600 ;    
SINS_Result{13}.name = 'RinsAngleError';
SINS_Result{13}.comment = 'R_INS欧拉角误差(〞)';
SINS_Result{13}.frequency = integFre ;

SINS_Result{14}.data = vnsdRbbEsmB*180/pi*3600 ;    
SINS_Result{14}.name = 'vnsdRbbEsmB';
SINS_Result{14}.comment = 'VNS_Rbb误差角估计(〞)';
SINS_Result{14}.frequency = integFre ;

SINS_Result{15}.data = gyroDriftEsmDRR*180/pi*3600 ;     % rad/s 转化为 °/h
SINS_Result{15}.name = 'gyroDriftEsmDRR(°/h)';
SINS_Result{15}.comment = 'DRR陀螺常值漂移估计';
SINS_Result{15}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
meanGyroDrift = mean(gyroDriftEsmDRR*180/pi*3600,2);  % 陀螺常值漂移估计均值
meanGyroDriftText{1} = '均值';
meanGyroDriftText{2} = sprintf('x：%0.3f°/h',meanGyroDrift(1));
meanGyroDriftText{3} = sprintf('y：%0.3f°/h',meanGyroDrift(2));
meanGyroDriftText{4} = sprintf('z：%0.3f°/h',meanGyroDrift(3));
SINS_Result{15}.text = meanGyroDriftText ;
SINS_Result{15}.project = 'GyroCalib_DRR';

frontNum = 15 ;
if isKnowTrue
    SINS_Result{frontNum+1}.data = SINSPositionError ;
    SINS_Result{frontNum+1}.name = 'positionError(m)';
    SINS_Result{frontNum+1}.comment = '位置误差';
    SINS_Result{frontNum+1}.frequency = combineFre ;
     % 计算最大相对误差    
    validLength = fix(length(position_r)*trueTraeFre/combineFre);
    true_position_valid = true_position(:,1:validLength) ;
    text_error_xyz = GetErrorText( true_position_valid,SINSPositionError ) ;
    SINS_Result{frontNum+1}.text = text_error_xyz ;
    
    SINS_Result{frontNum+2}.data = SINSAttitudeError*180/pi*3600;
    SINS_Result{frontNum+2}.name = 'attitudeError(‘’)';
    SINS_Result{frontNum+2}.comment = '姿态误差';
    SINS_Result{frontNum+2}.subName = {'俯仰','横滚','航向'};
    SINS_Result{frontNum+2}.frequency = combineFre ;
    
    SINS_Result{frontNum+3}.data = SINSVelocityError;
    SINS_Result{frontNum+3}.name = 'velocityError(m/ｓ)';
    SINS_Result{frontNum+3}.comment = '速度误差';
    SINS_Result{frontNum+3}.frequency = combineFre ;
    
    SINS_Result{frontNum+4}.data = SINSAccError/(gp*1e-6);
    SINS_Result{frontNum+4}.name = 'SINS_accError(ug)';
    SINS_Result{frontNum+4}.comment = 'SINS解算加速度误差';
    SINS_Result{frontNum+4}.frequency = combineFre ;
    
    SINS_Result{frontNum+5}.data = gyroDriftEsmAError*180/pi*3600 ;     % 转换为 °/h 
    SINS_Result{frontNum+5}.name = 'gyroDriftEsmAError(°/h)';
    SINS_Result{frontNum+5}.comment = 'DRbb_A陀螺常值漂移估计误差';
    SINS_Result{frontNum+5}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
    SINS_Result{frontNum+5}.project = 'GyroCalib_A';
    
    SINS_Result{frontNum+6}.data = gyroDriftEsmBError*180/pi*3600 ;     % 转换为 °/h 
    SINS_Result{frontNum+6}.name = 'gyroDriftEsmBError(°/h)';
    SINS_Result{frontNum+6}.comment = 'DRbb_B陀螺常值漂移估计误差';
    SINS_Result{frontNum+6}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
    SINS_Result{frontNum+6}.project = 'GyroCalib_B';
    
    SINS_Result{frontNum+7}.data = vnsdRbbEsmBError*180/pi*3600;
    SINS_Result{frontNum+7}.name = 'vnsdRbbEsmBError(〞)';
    SINS_Result{frontNum+7}.comment = 'VNS_Rbb误差角估计误差';
    SINS_Result{frontNum+7}.subName = {'俯仰','横滚','航向'};
    SINS_Result{frontNum+7}.frequency = combineFre ;
    
    SINS_Result{frontNum+8}.data = gyroDriftEsmDRRError*180/pi*3600 ;     % 转换为 °/h 
    SINS_Result{frontNum+8}.name = 'gyroDriftEsmDRRError(°/h)';
    SINS_Result{frontNum+8}.comment = 'DRR陀螺常值漂移估计误差';
    SINS_Result{frontNum+8}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
    SINS_Result{frontNum+8}.project = 'GyroCalib_DRR';
    
else
    SINS_Result = SINS_Result(1:4);
    
end
% 保存
resultPath = [pwd,'\result'];
if isdir(resultPath)
    delete([resultPath,'\*'])
else
    mkdir(resultPath)
end
save([resultPath,'\SINS_Result.mat'],'SINS_Result')

if isAlone == 1
    fid = fopen([resultPath,'\实验笔记（SINS独立运行）.txt'], 'w+');
    visualInputData=[];
    RecodeInput (fid,visualInputData,imuInputData,trueTrace);
    fprintf(fid,'\nSINS解算误差：\n');
    fprintf(fid,'%s',errorStr);
    fprintf(fid,'SINS解算耗时：%0.5g sec\n',toc);
    fclose(fid);
    open([resultPath,'\实验笔记（SINS独立运行）.txt'])
    %% 显示结果
    global projectDataPath
    projectDataPath = resultPath ;

    ResultDisplay()
end