% buaa xyz 2014.1.9

% 月球表面 世界坐标系下纯惯导解算：以初始时刻的xyz系作为导航系
% 源至 白鑫贝（2013.1.21-SINSr_addnoise）

function [SINS_Result,imuInputData] = main_SINSNav( imuInputData,trueTrace )
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
   
    load('圆弧1min')
    % load('直线5min-IMU噪声-真实RT')
    % load('静止5min-IMU噪声-随机RT')
    % load('静止5min-IMU噪声-真实RT')
    % load('圆弧5min-IMU噪声-随机RT')
    % load('圆弧5min-IMU噪声-真实RT')
    
    isAlone = 1;
else
    isAlone = 0;
end
if ~exist('imuInputData','var')
   load('imuInputData.mat') 
end
if ~exist('trueTrace','var')
    if exist('trueTrace.mat','file')
        load('trueTrace.mat') 
    else
        trueTrace=[];
    end
end
save imuInputData imuInputData
save trueTrace trueTrace
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

imu_T=1/imu_fre;     % sec
runTimeNum=size(f_INSm,2);
[pa,na,pg,ng,~] = GetIMUdrift( imuInputData,planet ) ; % pa(加计常值偏置),na（加计随机漂移）,pg(陀螺常值偏置),ng（陀螺随机漂移）

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

Wiee=[0;0;wip];
% 
% earthConst = getEarthCosnt;
% g0_e = earthConst.g0 ;  % 用于给定仿真的加计噪声

%% 初始条件

% 地理系导航参数
% attitude_t=zeros(3,runTimeNum);
% velocity_t=zeros(3,runTimeNum);
position_e=zeros(3,runTimeNum); % 经度（rad）,纬度（rad）,高度

% 世界坐标系导航参数
velocity_r=zeros(3,runTimeNum+1);
position_r=zeros(3,runTimeNum+1);
attitude_r = zeros(3,runTimeNum+1);
acc_r = zeros(3,runTimeNum);
% n 是当地地理坐标系， r 是世界坐标系（初始时刻地理坐标系）

position_e(:,1)=initialPosition_e;  %初始位置    position_e(1): 经度   position_e(2):纬度  position_e(3):高度
position_ini_e = FJWtoZJ(position_e(:,1),planet);  %初始时刻地固坐标系中的位置 （x,y,z/m）
attitude_r(:,1)=initialAttitude_r + [0/3600/180*pi;0/3600/180*pi;0/3600/180*pi];    %初始姿态 sita ,gama ,fai
Cbn=FCbn(attitude_r(:,1));
Cnb=Cbn';
Cen=FCen(position_e(1,1),position_e(2,1));       %calculate Cen
Cer = Cen; % 发射点惯性系相对于初始时刻地固系的旋转矩阵
Cre = Cer';
Crb = Cnb;
Cbr = Crb';
Crb_ins = Crb ;
velocity_r(:,1) = Cbr * initialVelocity_r;
Wirr = Cer * Wiee;

% 根据初始姿态矩阵Crb计算初始姿态四元数
Q0 = FCnbtoQ(Crb);

wh = waitbar(0,'纯惯导解算中...');
for t = 1:runTimeNum
    
    Wrbb = wib_INSm(:,t) - Crb * Wirr;
%     Q0=Q0+0.5*imu_T*[    0    ,-Wrbb(1,1),-Wrbb(2,1),-Wrbb(3,1);
%                  Wrbb(1,1),     0    , Wrbb(3,1),-Wrbb(2,1);
%                  Wrbb(2,1),-Wrbb(3,1),     0    , Wrbb(1,1);
%                  Wrbb(3,1), Wrbb(2,1),-Wrbb(1,1),     0    ]*Q0;
%     Q0=Q0/norm(Q0);
    Q0  = QuaternionDifferential( Q0,Wrbb,imu_T ) ;
        
    g = gp * (1+gk1*sin(position_e(2,t))^2-gk2*sin(2*position_e(2,t))^2);
    gn = [0;0;-g];
    Cen = FCen(position_e(1,t),position_e(2,t));
    Cnr = Cer * Cen';
    gr = Cnr * gn ;
  	%%%%%%%%%%% 速度方程 %%%%%%%%%%     
    a_rbr = Cbr * f_INSm(:,t) - getCrossMarix( 2*Wirr )*velocity_r(:,t) + gr;      
    acc_r(:,t) = a_rbr;

    Crb = FQtoCnb(Q0);
    Cbr = Crb';

    velocity_r(:,t+1) = velocity_r(:,t) + a_rbr * imu_T;
    position_r(:,t+1) = position_r(:,t) + velocity_r(:,t) * imu_T;
    positione0 = Cre * position_r(:,t+1) + position_ini_e; % 将发射点惯性系中的位置转化到初始时刻地固系
    position_e(:,t+1) = FZJtoJW(positione0,planet);    % 再转化为经纬高度

    opintions.headingScope=180;
    attitude_r(:,t+1) = GetAttitude(Crb,'rad',opintions);
   
    if mod(t,fix(runTimeNum/100))==0
        waitbar(t/runTimeNum)
    end
end
close(wh)
%%  内存不够时： 先 释放一些空间

%% 已知真实：计算误差
if  isKnowTrue==1
    % 计算组合数据有效长度
    lengthArrayOld = [length(position_r),length(true_position)];
    frequencyArray = [imu_fre,trueTraeFre];
    [~,~,combineLength,combineFre] = GetValidLength(lengthArrayOld,frequencyArray);
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
    errorStr = CalPosErrorIndex( true_position,SINSPositionError,SINSAttitudeError*180/pi*3600 );

else
    errorStr = '\n真实未知';
end

%% 输出 
imuInputData.errorStr=  errorStr ;
% 存储为特定格式：每个变量一个细胞，包含成员：data，name,comment 和 dataFlag,frequency,project,subName
resultNum = 30;
SINS_Result = cell(1,resultNum);

% 有4个共同的成员
for j=1:resultNum
    SINS_Result{j}.dataFlag = 'xyz result display format';
    SINS_Result{j}.frequency = imu_fre ;
    SINS_Result{j}.project = 'SINS';
    SINS_Result{j}.subName = {'x','y','z'};
end

res_k=0 ;

res_k = res_k+1 ;  
SINS_Result{res_k}.data = position_r ;
SINS_Result{res_k}.name = 'position(m)';
SINS_Result{res_k}.comment = '位置';

res_k = res_k+1 ;  
SINS_Result{res_k}.data = velocity_r ;
SINS_Result{res_k}.name = 'velocity_r(m/s)';
SINS_Result{res_k}.comment = '速度';

res_k = res_k+1 ;  
SINS_Result{res_k}.data = attitude_r*180/pi ;
SINS_Result{res_k}.name = 'attitude_r(°)';
SINS_Result{res_k}.comment = '姿态';
SINS_Result{res_k}.subName = {'俯仰','横滚','航向'};

res_k = res_k+1 ;  
SINS_Result{res_k}.data = acc_r ;
SINS_Result{res_k}.name ='acc_r(m/s^2)';
SINS_Result{res_k}.comment = '加速度';


if isKnowTrue
    
    res_k = res_k+1 ;  
    SINS_Result{res_k}.data = SINSPositionError ;
    SINS_Result{res_k}.name = 'positionError(m)';
    SINS_Result{res_k}.comment = '位置误差';
    SINS_Result{res_k}.frequency = combineFre ;
     % 计算最大相对误差    
    validLength = fix(length(position_r)*trueTraeFre/combineFre);
    true_position_valid = true_position(:,1:validLength) ;
    text_error_xyz = GetErrorText( true_position_valid,SINSPositionError ) ;
    SINS_Result{res_k}.text = text_error_xyz ;
    
    res_k = res_k+1 ; 
    SINS_Result{res_k}.data = SINSAttitudeError*180/pi*3600;
    SINS_Result{res_k}.name = 'attitudeError(‘’)';
    SINS_Result{res_k}.comment = '姿态误差';
    SINS_Result{res_k}.subName = {'俯仰','横滚','航向'};
    SINS_Result{res_k}.frequency = combineFre ;
    
    res_k = res_k+1 ; 
    SINS_Result{res_k}.data = SINSVelocityError;
    SINS_Result{res_k}.name = 'velocityError(m/ｓ)';
    SINS_Result{res_k}.comment = '速度误差';
    SINS_Result{res_k}.frequency = combineFre ;
    
    res_k = res_k+1 ; 
    SINS_Result{res_k}.data = SINSAccError/(gp*1e-6);
    SINS_Result{res_k}.name = 'SINS_accError(ug)';
    SINS_Result{res_k}.comment = 'SINS解算加速度误差';
    SINS_Result{res_k}.frequency = combineFre ;
    
end
SINS_Result = SINS_Result(1:res_k);

% 保存
resultPath = [pwd,'\result'];
if isdir(resultPath)
    delete([resultPath,'\*'])
else
    mkdir(resultPath)
end
save([resultPath,'\SINS_Result.mat'],'SINS_Result')
global projectDataPath
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
    
    projectDataPath = resultPath ;
    ResultDisplay()
end