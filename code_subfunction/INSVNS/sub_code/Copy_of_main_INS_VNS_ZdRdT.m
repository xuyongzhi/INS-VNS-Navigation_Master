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
function INS_VNS_NavResult = main_INS_VNS_ZdRdT(integMethod,visualInputData,imuInputData,trueTrace,isFigure)
% clc
% clear all 
% close all
% load([pwd,'\INS_VNS_allData'])
% isFigure=1;

format long
disp('函数 INS_VNS_ZdRdT 开始运行')
if ~exist('isFigure','var')
    isFigure = 0;
end
%% 导入数据

% (1) 导入纯视觉导航仿真解算的的中间结果，包括两个数据:Rbb[例3*3*127]、Tbb[例3*127]
VisualOut_RT=visualInputData.VisualRT;
RccVision = VisualOut_RT.Rbb;
TccVision = VisualOut_RT.Tbb;
frequency_VO = visualInputData.frequency;
% （2）IMU数据
wib_INSm = imuInputData.wib;
f_INSm = imuInputData.f;
imu_fre = imuInputData.frequency; % Hz

% 真实轨迹的参数
if ~exist('trueTrace','var')
    trueTrace = [];
end
resultPath = [pwd,'\result'];
if isdir(resultPath)
    delete([resultPath,'\*'])
else
   mkdir(resultPath) 
end
[planet,isKnowTrue,initialPosition_e,initialVelocity_r,initialAttitude_r,trueTraeFre,true_position,true_attitude,true_velocity,true_acc_r] = GetFromTrueTrace( trueTrace );

%% 星体常数
if strcmp(planet,'m')
    moonConst = getMoonConst;   % 得到月球常数
    gp = moonConst.g ;     % 用于导航解算
    wip = moonConst.wim ;
    Rp = moonConst.Rm ;
    e = moonConst.e;
    gk1 = moonConst.gk1;
    gk2 = moonConst.gk2;
    disp('轨迹发生器：月球')
else
    earthConst = getEarthConst;   % 得到地球常数
    gp = earthConst.g ;     % 用于导航解算
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
validLenth_INS_VNS = GetValidLength([size(f_INSm,2),size(TccVision,2)],[imu_fre,frequency_VO]); % 进行组合处理时，INS和VNS数据有效个数
imuNum = validLenth_INS_VNS(1); % 有效的IMU数据长度
%integnum = floor(imuNum/(imu_fre/frequency_VO))+1; % 组合导航数据个数 = 有效的VNS数据个数+1
integnum = validLenth_INS_VNS(2); % 组合导航数据个数 = 有效的VNS数据个数+1
integFre = frequency_VO;
cycleT_INS = 1/imu_fre;  % 捷联解算周期
cycleT_VNS = 1/frequency_VO;  % 视觉数据周期/滤波周期

%% SINS导航参数
% 由IMU噪声确定滤波PQ初值的选取
    % 仿真时噪声已知，存储在imuInputData中，实验室噪声未知，手动输入 常值偏置 和 随机标准差
[pa,na,pg,ng,~] = GetIMUdrift( imuInputData,planet ) ; % pa(加计常值偏置),na（加计随机漂移）,pg(陀螺常值偏置),ng（陀螺随机漂移）
%初始位置误差
dinit_pos = [0/(Rp*cos(pi/4));0/Rp;0];
%初始姿态误差
dinit_att = [0/3600/180*pi;0/3600/180*pi;0/3600/180*pi];

% 组合导航参数
INTGatt = zeros(3,integnum);  % 欧拉角姿态
INTGvel = zeros(3,integnum);  % 速度
INTGpos = zeros(3,integnum);  % 位置

% 捷联惯导解算导航参数
SINSatt = zeros(3,imuNum);  % 欧拉角姿态
SINSvel = zeros(3,imuNum);  % 速度
SINSpos = zeros(3,imuNum);  % 位置 米
SINSacc_r = zeros(3,imuNum);  % 加速度
SINSposition_d = zeros(3,imuNum);% 大地坐标系 经纬度

%% SINS初始条件
SINSposition_d(:,1) = initialPosition_e+dinit_pos;  % 经度 纬度 高度
SINSatt(:,1) = initialAttitude_r+dinit_att;         % 初始姿态 sita ,gama ,fai （rad）
Cen=FCen(SINSposition_d(1,1),SINSposition_d(2,1));       %calculate Cen
Cne=Cen';
positionr = Fdtoe(SINSposition_d(:,1),planet);  %地固坐标系中的初始位置
Cbn = FCbn(SINSatt(:,1));
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
Q0 = FCnbtoQ(Crb);
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
        
        szj1 = 0;
        szj2 = 0;
        szj3 = 0;
        P(:,:,1) = diag([(szj1)^2,(szj2)^2,(szj3)^2,(0.001)^2,(0.001)^2,(0.001)^2,1e-9,1e-9,1e-9,...
                        (pg(1))^2,(pg(2))^2,(pg(3))^2,(pa(1))^2,(pa(2))^2,(pa(3))^2]);
        Q_ini = diag([(ng(1))^2,(ng(2))^2,(ng(3))^2,(na(1))^2,(na(2))^2,(na(3))^2]);
         R = diag([1e-3,1e-3,1e-3,3e-5,1e-6,1e-6]);
        % display(P)
        % display(Q_ini)
        % R = diag(1e0*[1.6e-5,3.4e-7,9.9e-6,2.1e-5,3.4e-5,5.6e-5]); % with noise: 0.5 pixel
        % R = diag([1e-12*ones(1,3),2.1e-5,3.4e-5,5.6e-5]);
        % R = diag([5.9e-6,6.2e-8,3.1e-6,5.9e-5,1.5e-5,1.0e-4]); % line60 0.5pixel
        % R = diag([4.3e-6,1.5e-7,6.5e-6,1.3e-4,1.1e-5,7.7e-5]); % arc 0.5pixel
        % R = diag([2.5e-6,8.5e-8,3.9e-6,7.4e-5,1.1e-5,4.3e-5]); % zhx 0.5pixel
        H = [eye(3),zeros(3,12);
             zeros(3,6),-eye(3),zeros(3,6)];        % 量测矩阵为常量
    case 'augment_dRdT'
        %% 增广状态方程，dRdT为量测
        XNum = 21;
        ZNum = 6; % 量测信息维数
        X = zeros(XNum,integnum);       % 状态向量
        P = zeros(XNum,XNum,integnum); % 滤波P阵s
        szj1 = 0;
        szj2 = 0;
        szj3 = 0;
        P(:,:,1) = diag([(szj1)^2,(szj2)^2,(szj3)^2,(0.001)^2,(0.001)^2,(0.001)^2,1e-9,1e-9,1e-9,(pg(1)*pi/180/3600)^2,(pg(2)*pi/180/3600)^2,...
          (pg(3)*pi/180/3600)^2,(pa(1)*1e-6*g0)^2,(pa(2)*1e-6*g0)^2,(pa(3)*1e-6*g0)^2]);
        Q_ini = diag([(ng(1)*pi/180/3600)^2,(pg(2)*pi/180/3600)^2,(pg(3)*pi/180/3600)^2,(pa(1)*1e-6*g0)^2,(pa(2)*1e-6*g0)^2,(pa(3)*1e-6*g0)^2]);
        R = diag([1e-3,1e-3,1e-3,1e-4,1e-4,1e-4]);
end

%% 开始导航解算
% 记录上一滤波时刻的姿态和位置
Crb_last = Crb;
SINSpos_last = SINSpos(:,1);

waitbar_h=waitbar(0,'惯性/视觉-简化模型-dRdT ');
for t_imu = 1:imuNum-1
    if mod(t_imu,ceil((imuNum-1)/200))==0
        waitbar(t_imu/(imuNum-1))
    end
    %% 世界坐标系SINS导航解算
    Wrbb = wib_INSm(:,t_imu) - Crb * Wirr;
    % 角增量法解四元数微分方程（简化的）
    Q0=Q0+0.5*cycleT_INS*[      0    ,-Wrbb(1,1),-Wrbb(2,1),-Wrbb(3,1);
                            Wrbb(1,1),     0    , Wrbb(3,1),-Wrbb(2,1);
                            Wrbb(2,1),-Wrbb(3,1),     0    , Wrbb(1,1);
                            Wrbb(3,1), Wrbb(2,1),-Wrbb(1,1),     0    ]*Q0;
    Q0=Q0/norm(Q0);      % 单位化四元数
    % 四元数->方向余弦矩阵
    Crb = FQtoCnb(Q0);
    Cbr = Crb';
    % 更新当地加速度
    g = gp * (1+gk1*sin(SINSposition_d(2,t_imu))^2-gk2*sin(2*SINSposition_d(2,t_imu))^2);
    gn = [0;0;-g];
    % 更新姿态旋转矩阵
    Cen = FCen(SINSposition_d(1,t_imu),SINSposition_d(2,t_imu));
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
    SINSpos(:,t_imu+1) = SINSpos(:,t_imu) + SINSvel(:,t_imu+1) * cycleT_INS;
    positione0 = Cre * SINSpos(:,t_imu+1) + positionr; % 将发射点惯性系中的位置转化到初始时刻地固系
    SINSposition_d(:,t_imu+1) = Fetod(positione0,planet);
    
    %% KF滤波
    % 判断当前IMU数据是否为与某个图像最近的IMU数据，是则进行一次组合
    % t_imu=1001 开始第一次信息融合
    t_vision = (t_imu-1)/imu_fre*frequency_VO ;   % t_imu对应的t_vision（带小数点的个数）
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
                %% 简化的状态模型（不扩维），dRdT作为量测
                % Fai
                [F_k,G_k,Fai_k] = calFaiG_simpledRdT(Cbr,Wirr,f_INSm(:,t_imu),cycleT_VNS);
                % Q：系统噪声方差阵
                Q_k = calQ_simpledRdT( Q_ini,F_k,cycleT_VNS,G_k );
                 % 量测信息
                Z = calZ_simpledRdT( SINSpos(:,t_imu),SINSpos_last,Crb,Crb_last,RccVision(:,:,k_integ),TccVision(:,k_integ) );           
                % R设为固定值

                % KF滤波
                x_est = Fai_k * X(:,k_integ);   % 状态一步预测
                P_est = Fai_k * P(:,:,k_integ) * Fai_k' + Q_k;   % 均方误差一步预测

                K_t = P_est * H' / (H * P_est * H' + R);   % 滤波增益
                X(:,k_integ+1) = x_est + K_t * (Z - H * x_est);   % 状态估计

                P_k_integ = (eye(XNum) - K_t * H) * P_est * (eye(XNum) - K_t * H)' + K_t * R * K_t';   % 估计均方误差
                P(:,:,k_integ+1) = P_k_integ;
                % 保存误差估计值
                dangleEsm(:,k_integ+1) = X(1:3,k_integ+1); 
                dVelocityEsm(:,k_integ+1) = X(4:6,k_integ+1);
                dPositionEsm(:,k_integ+1) = X(7:9,k_integ+1);       
                gyroDrift(:,k_integ+1) = X(10:12,k_integ+1) ;
                accDrift(:,k_integ+1) = X(13:15,k_integ+1) ;
                % 保存估计均方误差
                P_k_integ_diag = diag(P_k_integ) ;  % P阵对角元素
                dangleEsmP(:,k_integ+1) = P_k_integ_diag(1:3);
                dVelocityEsmP(:,k_integ+1) = P_k_integ_diag(4:6);
                dPositionEsmP(:,k_integ+1) = P_k_integ_diag(7:9);
                gyroDriftP(:,k_integ+1) = P_k_integ_diag(10:12);
                accDriftP(:,k_integ+1) = P_k_integ_diag(13:15);
            case 'augment_dRdT'
                %% 增广状态方程，dRdT为量测
                
        end
        %% 修正位置和速度
        SINSpos(:,t_imu+1) = SINSpos(:,t_imu+1) - dPositionEsm(:,k_integ+1);        
        SINSvel(:,t_imu+1) = SINSvel(:,t_imu+1) - dVelocityEsm(:,k_integ+1);
        positione0 = Cre * SINSpos(:,t_imu+1) + positionr; % 将发射点惯性系中的位置转化到初始时刻地固系
        SINSposition_d(:,t_imu+1) = Fetod(positione0,planet);
        Cen = FCen(SINSposition_d(1,t_imu+1),SINSposition_d(2,t_imu+1));
        Cnr = Cer * Cen';
        % 由计算世界坐标系到真实世界坐标系的旋转矩阵
        Ccr = FCbn(dangleEsm(:,k_integ+1));           % 认为X(1:3,k_integ+1)是从计算机系c（SINS计算用r），转动到真实r坐标系的角度
        % 修正方向余弦矩阵和姿态四元数(修正姿态) 
%         Cbr= Ccr * Cbr;  % Cbr=Ccr*Cbc  ---> Cbc=Cbr，即未更新前的r是c
%         Crb = Cbr';
        Crb = Ccr * Crb;
        Cnb = Crb * Cnr;
        Q0 = FCnbtoQ(Crb);
        % 修正状态
        X(1:9,k_integ+1) = 0;
        Crb_last = Crb;
        SINSpos_last = SINSpos(:,t_imu+1);
        
        % 组合导航参数
        INTGpos(:,k_integ+1) = SINSpos(:,t_imu+1);
        INTGvel(:,k_integ+1) = SINSvel(:,t_imu+1);
        % 由方向余弦矩阵求姿态角
        opintions.headingScope=180;
        INTGatt(:,k_integ+1) = GetAttitude(Crb,'rad',opintions);
    end
end
close(waitbar_h)

%% 已知真实：计算误差
if  isKnowTrue==1
    % 计算组合数据有效长度
    lengthArrayOld = [length(INTGpos),length(true_position)];
    frequencyArray = [integFre,trueTraeFre];
    [~,~,combineLength,combineFre] = GetValidLength(lengthArrayOld,frequencyArray);
    INTGPositionError = zeros(3,combineLength); % 组合导航的位置误差
    INTGAttitudeError = zeros(3,combineLength); % 组合导航的姿态误差
    INTGVelocityError = zeros(3,combineLength); % 组合导航的速度误差
    for k=1:combineLength
        k_true = fix((k-1)*trueTraeFre/combineFre)+1 ;
        k_integ = fix((k-1)*integFre/combineFre)+1;
        INTGPositionError(:,k) = INTGpos(:,k_integ)-true_position(:,k_true) ;
        INTGAttitudeError(:,k) = INTGatt(:,k_integ)-true_attitude(:,k_true);
        INTGVelocityError(:,k) = INTGvel(:,k_integ)-true_velocity(:,k_true);  
    end    
    SINS_accError  =SINSacc_r-true_acc_r(:,1:length(SINSacc_r)) ; % SINS的加速度误差
    accDriftError = (accDrift-repmat(pa,1,integnum)) ;        % 组合导航的加计估计误差
    gyroDriftError = gyroDrift-repmat(pg,1,integnum) ;      % 组合导航的陀螺估计误差
end

time=zeros(1,integnum);
for i=1:integnum
    time(i)=(i-1)/frequency_VO/60;
end

%% 保存结果为特定格式
INS_VNS_NavResult = saveINS_VNS_NavResult(integFre,combineFre,imu_fre,projectName,gp,isKnowTrue,trueTraeFre,...
    INTGpos,INTGvel,INTGatt,dPositionEsm,dVelocityEsm,dangleEsm,accDrift,gyroDrift,INTGPositionError,true_position,...
    INTGAttitudeError,true_attitude,INTGVelocityError,accDriftError,gyroDriftError,dangleEsmP,dVelocityEsmP,dPositionEsmP,...
    gyroDriftP,accDriftP,SINS_accError);
save([resultPath,'\INS_VNS_NavResult.mat'],'INS_VNS_NavResult')
disp('INS_VNS_ZdRdT 函数运行结束')

% % 绘图显示

if isFigure
    figure;
    plot(time,INTGpos);
    %plot(time,SINSpos(1,1:imu_fre/frequency_VO:imuNum));
    title('月球车轨迹','fontsize',16);
    xlabel('时间(s)','fontsize',12);
    ylabel('航线(m)','fontsize',12);
    legend('x','y','z');
    saveas(gcf,[resultPath,'\月球车轨迹.emf'])
    saveas(gcf,[resultPath,'\月球车轨迹.fig'])
    
    figure;
    plot(INTGvel');
    figure;
    plot(INTGatt');

end

function [F,G,Fai] = calFaiG_simpledRdT(Cbr,Wirr,fb,cycleT)
% 计算量测矩阵 F G 
% 状态转移矩阵 Fi
XNum = 15;  % 状态维数
F11 = -getCrossMarix(Wirr);
F14 = Cbr;
fr = Cbr * fb;
F21 = -getCrossMarix(fr);
F22 = -2 * getCrossMarix(Wirr);
F25 = Cbr;
F32 = eye(3);
% 得状态矩阵
F = [F11,zeros(3),zeros(3),     F14, zeros(3);
       F21,     F22,zeros(3),zeros(3),      F25;
       zeros(3),F32,zeros(3),zeros(3), zeros(3);
       zeros(3,15);zeros(3,15)  ];
G = [Cbr,zeros(3);zeros(3),Cbr;zeros(3,6);zeros(3,6);zeros(3,6)];
% 得状态转移矩阵
step = 1;
Fai = eye(XNum,XNum);
for i = 1:10
    step = step*i;
    Fai = Fai + (F * cycleT)^i/step;
end

function Q = calQ_simpledRdT( Q_ini,F,cycleT,G )
% Q：系统噪声方差阵
Fi = F * cycleT;
Q = G*Q_ini*G';
tmp1 = Q * cycleT;
Q = tmp1;
for i = 2:11
    tmp2 = Fi * tmp1;
    tmp1 = (tmp2 + tmp2')/i;
    Q = Q + tmp1;
end

% 计算量测量
function Z = calZ_simpledRdT( SINSpos_t_imu,SINSpos_last,Crb,Crb_last,RccVision_k_integ,TccVision_k_integ )
% 量测信息
% RccVision(:,:,k_integ)是当前帧(k_integ)的本体系到下一帧(k_integ+1)的本体系的旋转矩阵C_bcurrent_to_bnext
% TccVision(:,:,k_integ)是VNS解算的k_integ+1本体坐标系下，当前帧(k_integ)的本体系到下一帧(k_integ+1)的本体系的平移矩阵 
%%%%%%%%%%% 注意这个地方原师姐程序弄错符号  **************
T_INS =  -(SINSpos_t_imu-SINSpos_last) ;  % 世界坐标系下，真实的k_integ帧位置到k_integ+1帧位置的平移
R_INS = (Crb * Crb_last')';
opintions.headingScope=180;
Z_INS = [GetAttitude(R_INS,'rad',opintions);T_INS];
R_VNS = RccVision_k_integ';   % (VNS)的VNS解算b(k_integ)到VNS解算b(k_integ+1)的旋转矩阵，以cycle_TVNS为周期
T_VNS = -(R_VNS * Crb_last)' * TccVision_k_integ;    % 将TccVision从视觉(k_integ+1)本体系转换到世界系
% 得到量测量
Z = [GetAttitude(R_INS*R_VNS','rad',opintions);T_INS-T_VNS];  % 误差=INS-VNS，=> INS_true=INS-error_estimate
