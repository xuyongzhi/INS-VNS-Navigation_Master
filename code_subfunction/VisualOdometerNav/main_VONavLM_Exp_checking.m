%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.5
%       2014.5.18
% 作者：xyz
% 功能：纯视觉实验程序：基于最小平方中值定理的视觉里程计，针对实验数据
% 源于白师姐的程序“VO_LMedS_LM”
% 5.19改： Tcc = M1 - Rcc * M0; 为 Tcc = -M1 + Rcc * M0;
%   改 X = LMalgorithm1(P2new,P1new,Q0,Tcc);Tcc = X(5:7); 为 X = LMalgorithm1(P2new,P1new,Q0,-Tcc);Tcc = -X(5:7);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function  [VOResult,visualInputData] = main_VONavLM_Exp(visualInputData,trueTrace,timeShorted) %trueTrace为可选输入
% 视觉导航需要输入 （1）特征匹配点信息visualInputData（2）初始位置姿态
% 输入trueTrace真实数据时生成导航结果误差，并以真实轨迹的初值作为 初始位置姿态
% 不输入trueTrace时不生成误差参数，并在运行时手动输入初始位置和姿态
format long
isAlone=0;
if ~exist('visualInputData','var')
    isAlone=1;
    load('visualInputData.mat')
    if exist('trueTrace.mat','file')
       load('trueTrace.mat') 
    else
        trueTrace = [];
    end
    timeShorted = 1 ;
end
save visualInputData visualInputData
save trueTrace trueTrace

[planet,isKnowTrue,initialPosition_e,initialVelocity_r,initialAttitude_r,trueTraeFre,true_position,...
    true_attitude,true_velocity,true_acc_r,runTime_IMU,runTime_image] = GetFromTrueTrace( trueTrace );
if isempty(trueTrace)
   % 需要再次给定 initialVelocity_r initialAttitude_r，初始位置总为[0;0;0]
   answer = inputdlg({'初始姿态(俯仰,横滚,航向)°                                          . ','初始速度(m/s)'},'独立视觉导航 - 给定初始条件',1,{'0 0 0','0 0 0'});
   initialVelocity_r = sscanf(answer{2},'%f');
   initialAttitude_r = sscanf(answer{1},'%f')*pi/180;   
   isKnowTrue=0;
end

% 检查图片参数

if ~isfield(visualInputData,'calibData')
    visualInputData.calibData = loadCalibData();  
else
    button=questdlg('是否更新 真实 相机标定参数?'); 
    if strcmp(button,'Yes')
        visualInputData.calibData = loadCalibData();  
    end    
end
calibData=visualInputData.calibData;
if ~isfield(calibData,'cameraSettingAngle')
    answer = inputdlg({'相机安装角°','相机安装角 误差°'},'俯仰 横滚 偏航 ',1,{'0 0 0','0 0 0'});
    cameraSettingAngle_true = sscanf(answer{1},'%f')'*pi/180;  
    cameraSettingAngle_error = sscanf(answer{2},'%f')'*pi/180;
    cameraSettingAngle = cameraSettingAngle_true+cameraSettingAngle_error ;
    calibData.cameraSettingAngle = cameraSettingAngle ;
    calibData.cameraSettingAngle_true = cameraSettingAngle_true ;
    calibData.cameraSettingAngle_error=cameraSettingAngle_error;
    visualInputData.calibData=calibData ;
end

%% 计算RT 或 导入现成的RT
% 根据 visualInputData 的成员判断是否需要进行三维重建和运动估计
%　存在特征点信息则进行三维重建，不存在特征点信息则直接提取RT
if isfield(visualInputData,'leftLocCurrent')
    fen = length(visualInputData.leftLocCurrent);
    if fen>5 && isfield(visualInputData,'VisualRT')
       button=questdlg('有特征点，也有RT，时间较长，是否进行特征点解算并更新RT?'); 
       if strcmp(button,'Yes')
           % 从特征点计算RT
            visualInputData = calculateRT_VO_new(visualInputData,timeShorted);
       end
    else
        visualInputData = calculateRT_VO_new(visualInputData,timeShorted);
    end    
end

VisualRT = visualInputData.VisualRT;
Rbb = VisualRT.Rbb;
if isfield(VisualRT,'Tbb_last')
    isTbb_last=1;
    Tbb_sel = VisualRT.Tbb_last ;
else
    isTbb_last=0;
    Tbb_sel = VisualRT.Tbb;
end

timeNum = length(visualInputData.leftLocCurrent);
timeNum = fix(timeNum*timeShorted) ;      %  截取一部分数据

VOfre = visualInputData.frequency ;
if isempty(runTime_image)
    runTime_image = (1:timeNum+1)/VOfre ;
end

%%
[ trueTbb,trueRbb  ] = GetTrueTbbRbb(trueTrace,visualInputData.frequency,isTbb_last) ;
[RTerrorStr,AngleError,TbbError] = analyseRT(Rbb,Tbb_sel,trueRbb,trueTbb);
angle_bb = RbbtoAngle_bb(Rbb);
true_angle_bb = RbbtoAngle_bb(trueRbb);
% % 以初始时刻地理系为导航系（世界坐标系），因此初始位置始终为0
% initialPosition=zeros(3,1);

% navigation parameter in world frame
VOsta = zeros(3,timeNum+1);
VOsta_trueRbb = zeros(3,timeNum+1);    % 无姿态误差时的视觉导航位置
VOsta_trueTbb = zeros(3,timeNum+1);    % 无Tbb误差时的视觉导航位置
VOsta_trueRbb_trueTbb = zeros(3,timeNum+1);
VOpos = zeros(3,timeNum+1);
VOvel = zeros(3,timeNum+1);
VOsta(:,1) = [0;0;0] ;  % 初始位置:以初始时刻左摄像机坐标系为原点
VOpos(:,1) = initialAttitude_r;   %初始姿态
VOvel(:,1) = initialVelocity_r;    % 初始速度
CrbSave = zeros(3,3,timeNum+1);
%% 初始误差
if isfield(trueTrace,'InitialPositionError')
    VOsta(:,1) = VOsta(:,1)+trueTrace.InitialPositionError ;
    VOpos(:,1) = VOpos(:,1)+trueTrace.InitialAttitudeError ;
end

Cbr=FCbn(VOpos(:,1));
Crb=Cbr';

true_position_valid = zeros(3,timeNum);
true_position_valid(:,1)=true_position(:,1);
%% 运动积分
% compute the path -- in local level coordinate 
% pos = zeros(3,timeNum+1);
for i = 1:timeNum
    % Rbb(:,:,i)是 b(i)到b(i+1)的旋转矩阵
    % Tbb(:,i)  是 b(i)到b(i+1)的平移矩阵
    
    Crb_last  = Crb ;
    Crb = Rbb(:,:,i) * Crb;
    if isTbb_last==1
        VOsta(:,i+1) = VOsta(:,i) + Crb_last' * Tbb_sel(:,i);  
    else
        VOsta(:,i+1) = VOsta(:,i) + Crb' * Tbb_sel(:,i);   
    end
    VOvel(:,i+1) = (VOsta(:,i+1) - VOsta(:,i)) / ( runTime_image(i+1)-runTime_image(i) ) ;
    
    opintions.headingScope=180;  
    VOpos(:,i+1) = GetAttitude(Crb,'rad',opintions);    
    CrbSave(:,:,i+1) = Crb ;
    % 计算无Rbb误差时的视觉导航位置
    if  isKnowTrue==1
        k_true = fix((i-1)*(trueTraeFre/VOfre))+1 ;
        k_true_next = fix((i)*(trueTraeFre/VOfre))+1 ;
        if k_true_next<=length(true_attitude)            
            
            if isTbb_last==1
                Cbr_true = FCbn(true_attitude(:,k_true)) ;
                VOsta_trueRbb(:,i+1) = VOsta_trueRbb(:,i) + Cbr_true * Tbb_sel(:,i);
                VOsta_trueTbb(:,i+1) = VOsta_trueTbb(:,i) + Crb_last' * trueTbb(:,i);   
                VOsta_trueRbb_trueTbb(:,i+1) = VOsta_trueRbb_trueTbb(:,i) + Cbr_true * trueTbb(:,i);  
            else
                Cbr_true_next = FCbn(true_attitude(:,k_true_next)) ;
                VOsta_trueRbb(:,i+1) = VOsta_trueRbb(:,i) + Cbr_true_next * Tbb_sel(:,i);  
                VOsta_trueTbb(:,i+1) = VOsta_trueTbb(:,i) + Crb' * trueTbb(:,i);   
                VOsta_trueRbb_trueTbb(:,i+1) = VOsta_trueRbb_trueTbb(:,i) + Cbr_true_next * trueTbb(:,i);  
            end
            true_position_valid(:,i+1) = true_position(:,k_true_next);                       
        end
    end
end

%% 初始速度
if  isKnowTrue==1
    VOvel(:,1) = true_velocity(:,1);    % 假定初始速度已知
end
%% 已知真实：计算误差
if  isKnowTrue==1
    % 计算组合数据有效长度
    lengthArrayOld = [length(VOsta),length(true_position)];
    frequencyArray = [VOfre,trueTraeFre];
    [validLenthArray,combineK,combineLength,combineFre] = GetValidLength(lengthArrayOld,frequencyArray);
    VOstaError = zeros(3,combineLength);
    VOsta_trueRbb_error = zeros(3,combineLength);
    VOsta_trueTbb_error = zeros(3,combineLength);
    VOsta_trueRbb_trueTbb_error = zeros(3,combineLength);
    VOposError = zeros(3,combineLength);
    VOvelError = zeros(3,combineLength);
    VOStaStepError = zeros(3,combineLength-1);      % 视觉位置单步误差
    VOStaStepError_A = zeros(3,combineLength-1);      % 视觉位置单步误差（按总的减去B部分计算）
    VOStaStepError_Adefine = zeros(3,combineLength-1);      % 视觉位置单步误差(按定义计算)
    VOStaStepError_B = zeros(3,combineLength-1);      % 视觉位置单步误差
    VOCrbError = zeros(3,3,combineLength-1) ;         % 姿态矩阵误差
    VOCrcError = zeros(3,3,combineLength-1) ;         % 平台失准矩阵与单位阵的差
    VOrcAngle = zeros(3,combineLength-1) ;              % 平台失准角
    for k=1:combineLength
        k_true = fix((k-1)*(trueTraeFre/combineFre))+1 ;
        k_VO = fix((k-1)*(VOfre/combineFre))+1;
        VOstaError(:,k) = VOsta(:,k_VO)-true_position(:,k_true) ;
        VOsta_trueRbb_error(:,k) = VOsta_trueRbb(:,k_VO)-true_position(:,k_true) ;
        VOsta_trueTbb_error(:,k) = VOsta_trueTbb(:,k_VO)-true_position(:,k_true) ;
        VOsta_trueRbb_trueTbb_error(:,k) = VOsta_trueRbb_trueTbb(:,k_VO)-true_position(:,k_true) ;
        VOposError(:,k) = VOpos(:,k_VO)-true_attitude(:,k_true) ;
        VOposError(3,k) = YawErrorAdjust(VOposError(3,k),'rad') ;
        VOvelError(:,k) = VOvel(:,k_VO)-true_velocity(:,k_true) ;
        if k>1 && k<combineLength
            VOStaStepError(:,k) = VOstaError(:,k)-VOstaError(:,k-1) ;            
            
            CrbError_k = FCbn(true_attitude(:,k_true))-CrbSave(:,:,k_VO)' ;
            VOCrbError(:,:,k) = CrbError_k ;
            
            Crc = CrbSave(:,:,k_VO)' * FCbn(true_attitude(:,k_true))' ;
            VOCrcError(:,:,k) = eye(3)-Crc ;
            opintions.headingScope = 180 ;
            VOrcAngle(:,k) = GetAttitude(Crc','rad',opintions);
            VOStaStepError_B(:,k) = CrbError_k*Tbb_sel(:,k_VO) ;
            
            VOStaStepError_A(:,k) = VOStaStepError(:,k)-VOStaStepError_B(:,k) ;
            % 按定义计算A部分的误差
            VOStaStepError_Adefine(:,k) = CrbSave(:,:,k_VO)'*(trueTbb(:,k_VO) - Tbb_sel(:,k_VO) ) ;
        end
    end    
    k_true_shorted = fix((timeNum-1)*(trueTraeFre/VOfre))+1 ;
    true_position_shorted = true_position(:,1:k_true_shorted);
    % 计算空间二维/三维位置误差及相对值
    errorStr = CalPosErrorIndex_route( true_position_shorted,VOstaError,VOposError*180/pi*3600,VOsta );
    % 计算A B 部分的误差
    [VOStaStepError_AStr,~,~,~] = AnalysSingleStepErorr(VOStaStepError_A);
    [VOStaStepError_BStr,~,~,~] = AnalysSingleStepErorr(VOStaStepError_B);
    errorStr = sprintf('%sA部分（dTbb导致）位置误差：%s\nB部分（Tbb在误差姿态上的分解导致）：%s\n',errorStr,VOStaStepError_AStr,VOStaStepError_BStr);
    % 真实Rbb视觉解算误差
    errorStr_trueRbb = CalPosErrorIndex_route( true_position_shorted,VOsta_trueRbb_error,[],VOsta_trueRbb );
    % 真实Tbb视觉解算误差
    errorStr_trueTbb = CalPosErrorIndex_route( true_position_shorted,VOsta_trueTbb_error,[],VOsta_trueTbb );
    errorStr_trueTbb_trueRbb = CalPosErrorIndex_route( true_position_shorted,VOsta_trueRbb_trueTbb_error,[],VOsta_trueRbb_trueTbb );
    
    errorStr = sprintf('%s真实Rbb时视觉导航位置误差：\n%s真实Tbb时视觉导航位置误差：\n%s真实Tbb+真实Rbb时视觉导航位置误差：\n%s',errorStr,errorStr_trueRbb,errorStr_trueTbb,errorStr_trueTbb_trueRbb);
    
    errorStr = sprintf('%s\n%s\n',errorStr,RTerrorStr);
    
else
    errorStr = '误差未知';
    VOposError = [];VOvelError=[];VOstaError=[];combineFre=[];VOStaStepError=[];
end

%% 输出
VisualRT.Rbb = Rbb ;
if isTbb_last==1
    VisualRT.Tbb_last = Tbb_sel ;
else
    VisualRT.Tbb = Tbb_sel ;
end
VisualRT.trueTbb = trueTbb;
VisualRT.trueRbb = trueRbb;
VisualRT.AngleError = AngleError;
VisualRT.TbbError = TbbError;
VisualRT.RTerrorStr = RTerrorStr;
visualInputData.VisualRT = VisualRT;
visualInputData.errorStr = errorStr;
% 存储为特定格式：每个变量一个细胞，包含成员：data，name,comment 和 dataFlag,frequency,project,subName

if isfield(visualInputData,'matchedNum')
    matchedNum = visualInputData.matchedNum ;
else
    matchedNum=[];
end
if isfield(visualInputData,'aveFeatureNum')
    aveFeatureNum = visualInputData.aveFeatureNum ;
else
    aveFeatureNum=[];
end

if ~exist('AngleError','var')
   AngleError=[]; 
   TbbError=[];
end
VOResult = saveVOResult_subplot( isKnowTrue,VOfre,VOsta,VOsta_trueRbb,VOpos,VOvel,matchedNum,aveFeatureNum,...
    VOposError,VOvelError, VOstaError,VOsta_trueRbb_error,VOsta_trueTbb_error,combineFre,trueTraeFre,true_position,VOStaStepError,VOStaStepError_A,...
    VOStaStepError_B,VOCrbError,VOCrcError,VOrcAngle,VOStaStepError_Adefine,angle_bb,Tbb_sel,true_angle_bb,trueTbb,AngleError,TbbError,runTime_image );

save([pwd,'\VONavResult\VOResult.mat'],'VOResult')
assignin('base','VOResult',VOResult)
save([pwd,'\VONavResult\VisualRT.mat'],'VisualRT')
assignin('base','VisualRT',VisualRT)

save visualInputData visualInputData
save errorStr errorStr
disp('纯视觉实验导航解算结束')
if isAlone==1
   display(errorStr)    
end
figure('name','视觉导航轨迹')
VOsta_length = length(VOsta);
trueTraceValidLength = fix((VOsta_length-1)*trueTraeFre/VOfre) +1 ;
trueTraceValidLength = min(trueTraceValidLength,length(true_position));
true_position_valid = true_position(:,1:trueTraceValidLength);
hold on
plot(true_position_valid(1,:),true_position_valid(2,:),'k','linewidth',1.3);
plot(VOsta(1,:),VOsta(2,:),'r','linewidth',1.3);
plot(VOsta_trueRbb(1,:),VOsta_trueRbb(2,:),'--b','linewidth',1.3);
plot(VOsta_trueTbb(1,:),VOsta_trueTbb(2,:),'-.m','linewidth',1.3);
plot(VOsta_trueRbb_trueTbb(1,:),VOsta_trueRbb_trueTbb(2,:),'--g');
legend('trueTrace','VO','trueRbb','trueTbb','trueRbb_trueTbb','fontsize',5);
%legend('trueTrace','VO','trueRbb','trueTbb','trueRT');
saveas(gcf,'视觉导航轨迹.fig')



function angle_bb = RbbtoAngle_bb(Rbb)
N=length(Rbb);
angle_bb=zeros(3,N);
for k=1:N
    opintions.headingScope = 180;
    angle_bb(:,k) = GetAttitude(Rbb(:,:,k),'rad',opintions);
end



function calibData = loadCalibData()
% 选择是否加入相机标定参数。添加：如果是真实实验采集，则加载，如果是视景仿真采集，则计算。
global projectDataPath
button =  questdlg('根据图片获取的方法选择','添加标定数据','视景仿真：计算标定参数','真实实验：载入标定参数文件','不添加','视景仿真：计算标定参数') ;
if strcmp(button,'视景仿真：计算标定参数')
    calibData = GetCalibData() ;
    calibData = SetCalibDataError(calibData) ;
end
if strcmp(button,'真实实验：载入标定参数文件')
    if isempty(projectDataPath) % 独立运行此函数时
        calibDataPath = pwd; 
    else
        calibDataPath = [GetUpperPath(projectDataPath),'\相机标定数据'];   % 默认相机标定数据路径
    end
    [cameraCalibName,cameraCalibPath] = uigetfile('.mat','选择相机标定数据',[calibDataPath,'\*.mat']);
    calibData = importdata([cameraCalibPath,cameraCalibName]); 
end


%% 特征点-> Rcc Tcc Rbb Tbb 
% new: Tcc Tbb 改为在前一时刻
function [visualInputData] = calculateRT_VO_new(visualInputData,timeShorted)


%% 视觉数据
button=questdlg('是否进行最小视差检查?'); 
if strcmp(button,'Yes')
    minDx = 9 ;     % 特征点视差检查（1024x1024,45°角，景深=247/dX，dX=5时景深为49m，dX=6时41.2m，dX=10时24.7m，dX=12时21.6mdX=7时35m，dX=8时31m，dX=15时16.5m）
    disp(sprintf('最小视差检查：%d',minDx)) ; %#ok<DSPS>
    visualInputData = RejectUselessFeaturePoint(visualInputData,minDx);    
    if minDx<10 && min(visualInputData.matchedNum)>300 
        disp('进行第二次最小视差检查');
        visualInputData_temp = RejectUselessFeaturePoint(visualInputData,11);
        if min(visualInputData_temp.matchedNum)>200
            visualInputData = visualInputData_temp ;
        end
    end
end
leftLocCurrent = visualInputData.leftLocCurrent ;
rightLocCurrent = visualInputData.rightLocCurrent ;
leftLocNext = visualInputData.leftLocNext ;
rightLocNext = visualInputData.rightLocNext ;
matchedNum = visualInputData.matchedNum ;

%aveFeatureNum = visualInputData.aveFeatureNum ;
timeNum = length(leftLocCurrent);   % 图像采样时刻数
timeNum = fix(timeNum*timeShorted) ;      %  截取一部分数据
% 存储计算的摄像机三维坐标结果
featureCPosCurrent = cell(1,timeNum);
featureCPosNext = cell(1,timeNum);
%% 相机标定的10+2个参数
calibData = visualInputData.calibData ;
button=questdlg('是否更新相机标定参数 误差?'); 
if strcmp(button,'Yes')
    calibData = SetCalibDataError(calibData) ;
    visualInputData.calibData = calibData ;
end
[ Cbc,Tcb_c,T,alpha_c_left,alpha_c_right,cc_left,cc_right,fc_left,fc_right,kc_left,kc_right,om,calibData ] = ExportCalibData( calibData ) ;
visualInputData.calibData = calibData ;
Ccb = Cbc';
%% 视觉图像的特征点信息

Rcc_save = zeros(3,3,timeNum);  % whole rotation 
Tcc_last_save = zeros(3,timeNum);  % whole translation
Rbb = zeros(3,3,timeNum);
Tbb_last = zeros(3,timeNum);
sm = 100;   % the number of Monte Carlo sample
q = 3;   % the number of matching point for each sample
Rcc_sm = zeros(3,3,sm);
Tcc_sm = zeros(3,1,sm);
Median = zeros(1,sm);
S = diag([1,1,-1]);
spixel = cell(1,timeNum);

Rk = zeros(6,6,timeNum); % 误差协方差
RELMOV = zeros(7,timeNum);
qRk = zeros(7,7,timeNum); % 误差协方差


% 显示进度条
h = waitbar(0,'从匹配特征点计算RbbTbb中...');
steps = timeNum;

for i = 1:timeNum
   %% 三维重建
   % Three-dimension restruction to get dots' position in world coordinate 
   P1 = zeros(matchedNum(i),3);    % store position information in previous time
   P2 = zeros(matchedNum(i),3);    % store position information in present time
   N = matchedNum(i);    % the number of features

    for j = 1:N

          xL = [leftLocCurrent{i}(j,2);leftLocCurrent{i}(j,1)]; % 第i个时刻的第j个当前帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
          xR = [rightLocCurrent{i}(j,2);rightLocCurrent{i}(j,1)];
          [P1(j,:),~] = stereo_triangulation(xL,xR,om,T'/1000,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right);
          % 得到当前特征点的左摄像机三维坐标

          xL = [leftLocNext{i}(j,2);leftLocNext{i}(j,1)]; % 第i个时刻的第j个下一帧帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
          xR = [rightLocNext{i}(j,2);rightLocNext{i}(j,1)];
          [P2(j,:),~] = stereo_triangulation(xL,xR,om,T'/1000,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right);

          % 得到（与以上当前时刻特征点匹配的）下一时刻特征点的左摄像机三维坐标
    end
    j_isnan=1;
   while j_isnan<=length(P1)
        for k_isnan=1:3
            if isnan(P1(j_isnan,k_isnan)) || isnan(P2(j_isnan,k_isnan))
                P1(j_isnan,:)=[];
                P2(j_isnan,:)=[];
                matchedNum(i)=matchedNum(i)-1;
                N=N-1;
                sprintf('第%d个时刻的第%d个特征点三维坐标无效',i,j_isnan)
                break;
            end
        end
       j_isnan=j_isnan+1;
    end
    featureCPosCurrent{i} = P1;
    featureCPosNext{i} = P2;
    %% 运动估计
   % Motion estimation to get coordinate translate matrix: LMedS
   for j = 1:sm
       ind = randi(N,1,q);
       % SVD method
       M0 = zeros(3,1);
       M1 = zeros(3,1);
       for k = 1:q
           M0 = M0 + P1(ind(k),:)';
           M1 = M1 + P2(ind(k),:)';
       end
       M0 = M0 / q;
       M1 = M1 / q;
       Pset0 = zeros(3,q);
       Pset1 = zeros(3,q);
       for k = 1:q
           Pset0(:,k) = P1(ind(k),:)' - M0;
           Pset1(:,k) = P2(ind(k),:)' - M1;
       end
       Q = Pset1*Pset0'/q;
       [U,~,V] = svd(Q);
       if abs(det(U)*det(V)-1) < 1e-10
           Rcc = U*V';
       elseif abs(det(U)*det(V)+1) < 1e-10
           Rcc = U*S*V';
       end
       
%     %    Tcc = M1 - Rcc * M0;
%       Tcc =- M1 + Rcc * M0; % 在后一时刻
      % 在前一时刻的表达
      Tcc_last = M0-Rcc'*M1 ;
       
       Rcc_sm(:,:,j) = Rcc;
       Tcc_sm(:,:,j) = Tcc_last;
       % compute regression variance and find Median
       r = zeros(1,N);
       for k = 1:N
           r(k) = norm(P2(k,:)' - (Rcc * P1(k,:)' + Tcc_last));
       end
%        rr = isnan(r);
%        indexr =  rr == 1;
%        r(indexr) = Inf;
       Median(j) = median(r);
   end
   
   % find the minimum Median
   mMed = min(Median);
   ord = find( Median == min(Median));
   Rcc = Rcc_sm(:,:,ord(1));
   Tcc_last = Tcc_sm(:,:,ord(1));
   
   % compute robust standrad deviation
   sigma = 1.4826 * (1 + 5 / (N - q)) * sqrt(mMed);
   % exstract matching point
   P1new = zeros(3,matchedNum(i));
   P2new = zeros(3,matchedNum(i));
   leftLocCurrentNew = zeros(matchedNum(i),2);
   rightLocCurrentNew = zeros(matchedNum(i),2);
   leftLocNextNew = zeros(matchedNum(i),2);
   rightLocNextNew = zeros(matchedNum(i),2);
   enum = 0;
   for j = 1:N
       res = norm(P2(j,:)' - (Rcc * P1(j,:)' + Tcc_last));
       if res ^ 2 <= (2.5 * sigma) ^ 2
           enum = enum + 1;
           P1new(:,enum) = P1(j,:)';
           P2new(:,enum) = P2(j,:)';
           leftLocCurrentNew(enum,:) = leftLocCurrent{i}(j,:);
           rightLocCurrentNew(enum,:) = rightLocCurrent{i}(j,:);
           leftLocNextNew(enum,:) = leftLocNext{i}(j,:);
           rightLocNextNew(enum,:) = rightLocNext{i}(j,:);
       end
   end
   % 选取残差最小的20个点
%    res = zeros(1,N);
%    for j = 1:N
%        res(j) = norm(P2(j,:)' - (Rcc * P1(j,:)' + Tcc_last));
%    end
%    [vals,indx] = sort(res);
%    for enum = 1:20
%        P1new(:,enum) = P1(indx(enum),:)';
%        P2new(:,enum) = P2(indx(enum),:)';
%        leftLocCurrent(enum,:) = visualInputData{i}.leftLocCurrent(indx(enum),:);
%        rightLocCurrent(enum,:) = visualInputData{i}.rightLocCurrent(indx(enum),:);
%        leftLocNext(enum,:) = visualInputData{i}.leftLocNext(indx(enum),:);
%        rightLocNext(enum,:) = visualInputData{i}.rightLocNext(indx(enum),:);
%    end
   P1new(:,enum+1:N) = [];
   P2new(:,enum+1:N) = [];
   leftLocCurrentNew(enum+1:N,:) = [];
   rightLocCurrentNew(enum+1:N,:) = [];
   leftLocNextNew(enum+1:N,:) = [];
   rightLocNextNew(enum+1:N,:) = [];
   spixel{i}.leftLocCurrent = leftLocCurrentNew;
   spixel{i}.rightLocCurrent = rightLocCurrentNew;
   spixel{i}.leftLocNext = leftLocNextNew;
   spixel{i}.rightLocNext = rightLocNextNew;
   % SVD method to get the final motion estimation (R,T)
   M0 = zeros(3,1);
   M1 = zeros(3,1);
   for k = 1:enum
       M0 = M0 + P1new(:,k);
       M1 = M1 + P2new(:,k);
   end
   M0 = M0 / enum;
   M1 = M1 / enum;
   Pset0 = zeros(3,enum);
   Pset1 = zeros(3,enum);
   for k = 1:enum
       Pset0(:,k) = P1new(:,k) - M0;
       Pset1(:,k) = P2new(:,k) - M1;
   end
   Q = Pset1*Pset0'/enum;
   [U,D,V] = svd(Q);
   if abs(det(U)*det(V)-1) < 1e-10
       Rcc = U*V';
   elseif abs(det(U)*det(V)+1) < 1e-10
       Rcc = U*S*V';
   end

  Tcc_last = M0-Rcc'*M1 ;

   % 根据姿态矩阵Rcc计算姿态四元数
   q1=1/2*sqrt(abs(1+Rcc(1,1)-Rcc(2,2)-Rcc(3,3)));
   q2=1/2*sqrt(abs(1-Rcc(1,1)+Rcc(2,2)-Rcc(3,3)));
   q3=1/2*sqrt(abs(1-Rcc(1,1)-Rcc(2,2)+Rcc(3,3)));
   q0=sqrt(abs(1-q1^2-q2^2-q3^2));
   if Rcc(2,3)-Rcc(3,2)<0
       q1=-q1;
   end
   if Rcc(3,1)-Rcc(1,3)<0
       q2=-q2;
   end
   if Rcc(1,2)-Rcc(2,1)<0
       q3=-q3;
   end
   Q0=[q0;q1;q2;q3];
   Q0=Q0/norm(Q0);
   X = LMalgorithm1(P2new,P1new,Q0,-Tcc_last);
   
   %% 得到Rcc和Tcc_last的最终值
   Rcc = [X(1)^2+X(2)^2-X(3)^2-X(4)^2,    2*(X(2)*X(3)+X(1)*X(4)),        2*(X(2)*X(4)-X(1)*X(3));
         2*(X(2)*X(3)-X(1)*X(4)),    X(1)*X(1)-X(2)*X(2)+X(3)*X(3)-X(4)*X(4),    2*(X(3)*X(4)+X(1)*X(2));
         2*(X(2)*X(4)+X(1)*X(3)),        2*(X(3)*X(4)-X(1)*X(2)),    X(1)*X(1)-X(2)*X(2)-X(3)*X(3)+X(4)*X(4)];
   Tcc_last = -X(5:7);
   Rcc_save(:,:,i) = Rcc;
   Tcc_last_save(:,i) = Tcc_last;
   % 计算重投影误差目标函数的Jacobi矩阵
   % 以计算相对运动参数的误差协方差矩阵
   %% 从 Tcc Rcc 到 Tbb Rbb
   Rbb(:,:,i) = Ccb * Rcc * Cbc; % Rbb
   Tbb_last(:,i) = Ccb * Tcc_last + Ccb*(Rcc'-eye(3)) * Tcb_c ;
%    Tbb_last(:,i) = Ccb * Tcc_last ;
   % 计算量测噪声方差阵
    % 计算姿态角
    pos(1) = asin(Rbb(2,3,i));  % 俯仰角  
    if Rbb(3,3,i)>0
        pos(2)=atan(-Rbb(1,3,i)/Rbb(3,3,i)); % roll
    elseif Rbb(3,3,i)<0
        if Rbb(1,3,i)>0
            pos(2)=pos(2)-pi;
        else
            pos(2)=pos(2)+pi;
        end
    elseif Rbb(3,3,i)==0
        if Rbb(1,3,i)>0
            pos(2)=-pi/2;
        else
            pos(2)=1/2*pi;
        end
    end
    if Rbb(2,2,i)>0   % 航向角
        if Rbb(2,1,i)>=0
            pos(3) = atan(-Rbb(2,1,i)/Rbb(2,2,i)); % + 2 * pi
        elseif Rbb(2,1,i)<0
            pos(3) = atan(-Rbb(2,1,i)/Rbb(2,2,i));
        end
    elseif Rbb(2,2,i)<0
        pos(3) = pi + atan(-Rbb(2,1,i)/Rbb(2,2,i));
    elseif Rbb(2,2,i)==0
        if Rbb(2,1,i)>0
            pos(3) = 1.5 * pi;
        elseif Rbb(2,1)<0
            pos(3) = pi / 2;
        end
    end
%     Rk(:,:,i) = R_covEuler(P1new,pos);
    Rk(:,:,i) = R_covEuler1(P2new,P1new,Rbb(:,:,i),pos,Tbb_last(:,i));

   % 计算量测噪声方差阵
   % 根据姿态矩阵Rbb计算姿态四元数
   q1=1/2*sqrt(abs(1+Rbb(1,1,i)-Rbb(2,2,i)-Rbb(3,3,i)));
   q2=1/2*sqrt(abs(1-Rbb(1,1,i)+Rbb(2,2,i)-Rbb(3,3,i)));
   q3=1/2*sqrt(abs(1-Rbb(1,1,i)-Rbb(2,2,i)+Rbb(3,3,i)));
   q0=sqrt(abs(1-q1^2-q2^2-q3^2));
   if Rbb(2,3,i)-Rbb(3,2,i)<0
       q1=-q1;
   end
   if Rbb(3,1,i)-Rbb(1,3,i)<0
       q2=-q2;
   end
   if Rbb(1,2,i)-Rbb(2,1,i)<0
       q3=-q3;
   end
   Q0=[q0;q1;q2;q3];
   Q0=Q0/norm(Q0);
%    Rk(:,:,i) = R_cov1(P1new,Q0);
   qRk(:,:,i) = R_cov2(P2new,P1new,Q0,Rbb(:,:,i),Tbb_last(:,i));
   RELMOV(:,i) = [Q0;Tbb_last(:,i)];
   if mod(i,fix(timeNum/80))==0 || timeNum<100
        waitbar(i/steps,h);
   end
end
close(h);

VisualRT.Rbb = Rbb ;
VisualRT.Tbb_last = Tbb_last ;
VisualRT.Rcc = Rcc_save ;
VisualRT.Tcc_last = Tcc_last_save ;

visualInputData.VisualRT = VisualRT;
visualInputData.matchedNum = matchedNum;
visualInputData.featureCPosCurrent = featureCPosCurrent;
visualInputData.featureCPosNext = featureCPosNext;


save([pwd,'\VONavResult\spixel.mat'],'spixel');

%% 从真实轨迹计算真实的 Tbb Rbb
% isTbb_last=1 : Tbb 在上一时刻分解，
% isTbb_last=0 : Tbb 在后一时刻分解
function [ trueTbb,trueRbb  ] = GetTrueTbbRbb(trueTrace,visualFre,isTbb_last)
format long

position = trueTrace.position ;
attitude = trueTrace.attitude ;
trueFre = trueTrace.frequency;

if isempty(visualFre)
    answer = inputdlg('视觉信息频率');
    visualFre = str2double(answer);
end
visualNum = fix( (length(position)-1)*visualFre/trueFre);
Rbb = zeros(3,3,visualNum);
Tbb = zeros(3,visualNum);
% 根据真实轨迹生成真实 Tbb Rbb
for k=1:visualNum
    k_true_last = 1+fix((k-1)*trueFre/visualFre) ;
    k_true = 1+fix((k)*trueFre/visualFre) ;
    if isTbb_last==1    % 得到 Tbb_last 在上一时刻分解
        Tbb(:,k) = FCbn(attitude(:,k_true_last))' * ( position(:,k_true)-position(:,k_true_last) ) ;
        
    else                % 得到 Tbb 在后一时刻分解
        Tbb(:,k) = FCbn(attitude(:,k_true))' * ( position(:,k_true)-position(:,k_true_last) ) ;
    end
    Rbb(:,:,k) =  FCbn(attitude(:,k_true))' * FCbn(attitude(:,k_true_last)) ;     % R:b(k)->b(k+1)

end
trueTbb = Tbb ;
trueRbb = Rbb;

%% 分析 Rbb Tbb 的噪声特性
% 设 Rbb Tbb 为高斯白噪声
function [RTerrorStr,AngleError,TbbError] = analyseRT(Rbb,Tbb,trueRbb,trueTbb)

RbbNum = length(Rbb);
trueRbbNum = length(trueRbb);
num = min(RbbNum,trueRbbNum);
TbbError = Tbb(:,1:num)-trueTbb(:,1:num);
RbbError = zeros(3,3,num);
AngleError = zeros(3,num);
opintions.headingScope = 180 ;
for k=1:num
    RbbError(:,:,k) = Rbb(:,:,k)*trueRbb(:,:,k)';
    AngleError(:,k) = GetAttitude(RbbError(:,:,k),'degree',opintions);
end

TbbErrorMean = mean(TbbError,2);
TbbErrorStd = std(TbbError,0,2);
AngleErrorMean = mean(AngleError,2);
AngleErrorStd = std(AngleError,0,2);

TbbErrorMeanstr = sprintf('%0.3e ',TbbErrorMean);
TbbErrorStdstr = sprintf('%0.3e ',TbbErrorStd);
AngleErrorMeanstr = sprintf('%0.3e ',AngleErrorMean*180/pi);
AngleErrorStdstr = sprintf('%0.3e ',AngleErrorStd*180/pi);

RTerrorStr = sprintf('Tbb误差特性：\n\t常值：%s m\n\t方差：%s m',TbbErrorMeanstr,TbbErrorStdstr);
RTerrorStr = sprintf('%s\nRbb误差角特性：\n\t常值：%s °\n\t方差：%s °',RTerrorStr,AngleErrorMeanstr,AngleErrorStdstr);


%% 特征点->Rbb+Tbb
function [visualInputData] = calculateRT_VO(visualInputData)


%% 视觉数据
button=questdlg('是否进行最小视差检查?'); 
if strcmp(button,'Yes')
    minDx = 12 ;     % 特征点视差检查（1024x1024,45°角，景深=247/dX，dX=5时景深为49m，dX=6时41.2m，dX=10时24.7m，dX=12时21.6mdX=7时35m，dX=8时31m，dX=15时16.5m）
    disp(sprintf('最小视差检查：%d',minDx)) ; %#ok<DSPS>
    visualInputData = RejectUselessFeaturePoint(visualInputData,minDx);    
    if minDx<8 && min(visualInputData.matchedNum)>300
        disp('进行第二次最小视差检查');
        visualInputData_temp = RejectUselessFeaturePoint(visualInputData,11);
        if min(visualInputData_temp.matchedNum)>200
            visualInputData = visualInputData_temp ;
        end
    end
end
leftLocCurrent = visualInputData.leftLocCurrent ;
rightLocCurrent = visualInputData.rightLocCurrent ;
leftLocNext = visualInputData.leftLocNext ;
rightLocNext = visualInputData.rightLocNext ;
matchedNum = visualInputData.matchedNum ;

%aveFeatureNum = visualInputData.aveFeatureNum ;
timeNum = length(leftLocCurrent);   % 图像采样时刻数
% 存储计算的摄像机三维坐标结果
featureCPosCurrent = cell(1,timeNum);
featureCPosNext = cell(1,timeNum);
%% 相机标定的10个参数
calibData = visualInputData.calibData ;
button=questdlg('是否更新相机标定参数 误差?'); 
if strcmp(button,'Yes')
    calibData = SetCalibDataError(calibData) ;
    visualInputData.calibData = calibData ;
end

T = calibData.T;  % mm为单位，列存储
alpha_c_left = calibData.alpha_c_left;
alpha_c_right = calibData.alpha_c_right;
cc_left = calibData.cc_left;
cc_right = calibData.cc_right;
fc_left = calibData.fc_left;
fc_right = calibData.fc_right;
kc_left = calibData.kc_left;
kc_right = calibData.kc_right;
om = calibData.om;
cameraSettingAngle = calibData.cameraSettingAngle ;
if isfield(calibData,'isEnableCalibError')
   if  calibData.isEnableCalibError==1
     	disp('双目视觉误差激活');
        T =  T+calibData.T_error ;
        om = om+calibData.om_error ;
        cameraSettingAngle = cameraSettingAngle+calibData.cameraSettingAngle_error ;
   end
end
%% 视觉图像的特征点信息

Rcc_save = zeros(3,3,timeNum);  % whole rotation 
Tcc_last_save = zeros(3,timeNum);  % whole translation
Rbb = zeros(3,3,timeNum);
Tbb = zeros(3,timeNum);
sm = 100;   % the number of Monte Carlo sample
q = 3;   % the number of matching point for each sample
Rcc_sm = zeros(3,3,sm);
Tcc_sm = zeros(3,1,sm);
Median = zeros(1,sm);
S = diag([1,1,-1]);
spixel = cell(1,timeNum);

Rk = zeros(6,6,timeNum); % 误差协方差
RELMOV = zeros(7,timeNum);
qRk = zeros(7,7,timeNum); % 误差协方差

Cbb1 = FCbn(cameraSettingAngle)';
Cb1c = [1, 0, 0;     % 本体系到摄像机坐标系:绕x轴转动-90度
       0, 0,-1;     % 摄像机坐标系c： x和y在相机平面，y向下，x向右，z向前
       0, 1, 0];    % 本体系b：x向右，y向前，z向上
Cbc = Cb1c*Cbb1 ;
Ccb = Cbc';

% 显示进度条
h = waitbar(0,'从匹配特征点计算RbbTbb中...');
steps = timeNum;

for i = 1:timeNum
   %% 三维重建
   % Three-dimension restruction to get dots' position in world coordinate 
   P1 = zeros(matchedNum(i),3);    % store position information in previous time
   P2 = zeros(matchedNum(i),3);    % store position information in present time
   N = matchedNum(i);    % the number of features

    for j = 1:N

          xL = [leftLocCurrent{i}(j,2);leftLocCurrent{i}(j,1)]; % 第i个时刻的第j个当前帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
          xR = [rightLocCurrent{i}(j,2);rightLocCurrent{i}(j,1)];
          [P1(j,:),~] = stereo_triangulation(xL,xR,om,T'/1000,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right);
          % 得到当前特征点的左摄像机三维坐标

          xL = [leftLocNext{i}(j,2);leftLocNext{i}(j,1)]; % 第i个时刻的第j个下一帧帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
          xR = [rightLocNext{i}(j,2);rightLocNext{i}(j,1)];
          [P2(j,:),~] = stereo_triangulation(xL,xR,om,T'/1000,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right);

          % 得到（与以上当前时刻特征点匹配的）下一时刻特征点的左摄像机三维坐标
    end
    j_isnan=1;
   while j_isnan<=length(P1)
        for k_isnan=1:3
            if isnan(P1(j_isnan,k_isnan)) || isnan(P2(j_isnan,k_isnan))
                P1(j_isnan,:)=[];
                P2(j_isnan,:)=[];
                matchedNum(i)=matchedNum(i)-1;
                N=N-1;
                sprintf('第%d个时刻的第%d个特征点三维坐标无效',i,j_isnan)
                break;
            end
        end
       j_isnan=j_isnan+1;
    end
    featureCPosCurrent{i} = P1;
    featureCPosNext{i} = P2;
    %% 运动估计
   % Motion estimation to get coordinate translate matrix: LMedS
   for j = 1:sm
       ind = randi(N,1,q);
       % SVD method
       M0 = zeros(3,1);
       M1 = zeros(3,1);
       for k = 1:q
           M0 = M0 + P1(ind(k),:)';
           M1 = M1 + P2(ind(k),:)';
       end
       M0 = M0 / q;
       M1 = M1 / q;
       Pset0 = zeros(3,q);
       Pset1 = zeros(3,q);
       for k = 1:q
           Pset0(:,k) = P1(ind(k),:)' - M0;
           Pset1(:,k) = P2(ind(k),:)' - M1;
       end
       Q = Pset1*Pset0'/q;
       [U,~,V] = svd(Q);
       if abs(det(U)*det(V)-1) < 1e-10
           Rcc = U*V';
       elseif abs(det(U)*det(V)+1) < 1e-10
           Rcc = U*S*V';
       end
       
    %    Tcc = M1 - Rcc * M0;
      Tcc =- M1 + Rcc * M0; % 在后一时刻
      % 在前一时刻的表达
      Tcc_last = M0-Rcc'*M1 ;
       
       Rcc_sm(:,:,j) = Rcc;
       Tcc_sm(:,:,j) = Tcc;
       % compute regression variance and find Median
       r = zeros(1,N);
       for k = 1:N
           r(k) = norm(P2(k,:)' - (Rcc * P1(k,:)' + Tcc));
       end
%        rr = isnan(r);
%        indexr =  rr == 1;
%        r(indexr) = Inf;
       Median(j) = median(r);
   end
   
   % find the minimum Median
   mMed = min(Median);
   ord = find( Median == min(Median));
   Rcc = Rcc_sm(:,:,ord(1));
   Tcc = Tcc_sm(:,:,ord(1));
   
   % compute robust standrad deviation
   sigma = 1.4826 * (1 + 5 / (N - q)) * sqrt(mMed);
   % exstract matching point
   P1new = zeros(3,matchedNum(i));
   P2new = zeros(3,matchedNum(i));
   leftLocCurrentNew = zeros(matchedNum(i),2);
   rightLocCurrentNew = zeros(matchedNum(i),2);
   leftLocNextNew = zeros(matchedNum(i),2);
   rightLocNextNew = zeros(matchedNum(i),2);
   enum = 0;
   for j = 1:N
       res = norm(P2(j,:)' - (Rcc * P1(j,:)' + Tcc));
       if res ^ 2 <= (2.5 * sigma) ^ 2
           enum = enum + 1;
           P1new(:,enum) = P1(j,:)';
           P2new(:,enum) = P2(j,:)';
           leftLocCurrentNew(enum,:) = leftLocCurrent{i}(j,:);
           rightLocCurrentNew(enum,:) = rightLocCurrent{i}(j,:);
           leftLocNextNew(enum,:) = leftLocNext{i}(j,:);
           rightLocNextNew(enum,:) = rightLocNext{i}(j,:);
       end
   end
   % 选取残差最小的20个点
%    res = zeros(1,N);
%    for j = 1:N
%        res(j) = norm(P2(j,:)' - (Rcc * P1(j,:)' + Tcc));
%    end
%    [vals,indx] = sort(res);
%    for enum = 1:20
%        P1new(:,enum) = P1(indx(enum),:)';
%        P2new(:,enum) = P2(indx(enum),:)';
%        leftLocCurrent(enum,:) = visualInputData{i}.leftLocCurrent(indx(enum),:);
%        rightLocCurrent(enum,:) = visualInputData{i}.rightLocCurrent(indx(enum),:);
%        leftLocNext(enum,:) = visualInputData{i}.leftLocNext(indx(enum),:);
%        rightLocNext(enum,:) = visualInputData{i}.rightLocNext(indx(enum),:);
%    end
   P1new(:,enum+1:N) = [];
   P2new(:,enum+1:N) = [];
   leftLocCurrentNew(enum+1:N,:) = [];
   rightLocCurrentNew(enum+1:N,:) = [];
   leftLocNextNew(enum+1:N,:) = [];
   rightLocNextNew(enum+1:N,:) = [];
   spixel{i}.leftLocCurrent = leftLocCurrentNew;
   spixel{i}.rightLocCurrent = rightLocCurrentNew;
   spixel{i}.leftLocNext = leftLocNextNew;
   spixel{i}.rightLocNext = rightLocNextNew;
   % SVD method to get the final motion estimation (R,T)
   M0 = zeros(3,1);
   M1 = zeros(3,1);
   for k = 1:enum
       M0 = M0 + P1new(:,k);
       M1 = M1 + P2new(:,k);
   end
   M0 = M0 / enum;
   M1 = M1 / enum;
   Pset0 = zeros(3,enum);
   Pset1 = zeros(3,enum);
   for k = 1:enum
       Pset0(:,k) = P1new(:,k) - M0;
       Pset1(:,k) = P2new(:,k) - M1;
   end
   Q = Pset1*Pset0'/enum;
   [U,D,V] = svd(Q);
   if abs(det(U)*det(V)-1) < 1e-10
       Rcc = U*V';
   elseif abs(det(U)*det(V)+1) < 1e-10
       Rcc = U*S*V';
   end
  Tcc = - M1 + Rcc * M0;
 %   Tcc = M1 - Rcc * M0;
   % 根据姿态矩阵Rcc计算姿态四元数
   q1=1/2*sqrt(abs(1+Rcc(1,1)-Rcc(2,2)-Rcc(3,3)));
   q2=1/2*sqrt(abs(1-Rcc(1,1)+Rcc(2,2)-Rcc(3,3)));
   q3=1/2*sqrt(abs(1-Rcc(1,1)-Rcc(2,2)+Rcc(3,3)));
   q0=sqrt(abs(1-q1^2-q2^2-q3^2));
   if Rcc(2,3)-Rcc(3,2)<0
       q1=-q1;
   end
   if Rcc(3,1)-Rcc(1,3)<0
       q2=-q2;
   end
   if Rcc(1,2)-Rcc(2,1)<0
       q3=-q3;
   end
   Q0=[q0;q1;q2;q3];
   Q0=Q0/norm(Q0);
   X = LMalgorithm1(P2new,P1new,Q0,-Tcc);
   
   %% 得到Rcc和Tcc的最终值
   Rcc = [X(1)^2+X(2)^2-X(3)^2-X(4)^2,    2*(X(2)*X(3)+X(1)*X(4)),        2*(X(2)*X(4)-X(1)*X(3));
         2*(X(2)*X(3)-X(1)*X(4)),    X(1)*X(1)-X(2)*X(2)+X(3)*X(3)-X(4)*X(4),    2*(X(3)*X(4)+X(1)*X(2));
         2*(X(2)*X(4)+X(1)*X(3)),        2*(X(3)*X(4)-X(1)*X(2)),    X(1)*X(1)-X(2)*X(2)-X(3)*X(3)+X(4)*X(4)];
   Tcc = -X(5:7);
   Rcc_save(:,:,i) = Rcc;
   Tcc_last_save(:,i) = Tcc;
   % 计算重投影误差目标函数的Jacobi矩阵
   % 以计算相对运动参数的误差协方差矩阵
   Rbb(:,:,i) = Ccb * Rcc * Cbc; % Rbb
   Tbb(:,i) = Ccb * Tcc;
   % 计算量测噪声方差阵
    % 计算姿态角
    pos(1) = asin(Rbb(2,3,i));  % 俯仰角  
    if Rbb(3,3,i)>0
        pos(2)=atan(-Rbb(1,3,i)/Rbb(3,3,i)); % roll
    elseif Rbb(3,3,i)<0
        if Rbb(1,3,i)>0
            pos(2)=pos(2)-pi;
        else
            pos(2)=pos(2)+pi;
        end
    elseif Rbb(3,3,i)==0
        if Rbb(1,3,i)>0
            pos(2)=-pi/2;
        else
            pos(2)=1/2*pi;
        end
    end
    if Rbb(2,2,i)>0   % 航向角
        if Rbb(2,1,i)>=0
            pos(3) = atan(-Rbb(2,1,i)/Rbb(2,2,i)); % + 2 * pi
        elseif Rbb(2,1,i)<0
            pos(3) = atan(-Rbb(2,1,i)/Rbb(2,2,i));
        end
    elseif Rbb(2,2,i)<0
        pos(3) = pi + atan(-Rbb(2,1,i)/Rbb(2,2,i));
    elseif Rbb(2,2,i)==0
        if Rbb(2,1,i)>0
            pos(3) = 1.5 * pi;
        elseif Rbb(2,1)<0
            pos(3) = pi / 2;
        end
    end
%     Rk(:,:,i) = R_covEuler(P1new,pos);
    Rk(:,:,i) = R_covEuler1(P2new,P1new,Rbb(:,:,i),pos,Tbb(:,i));

   % 计算量测噪声方差阵
   % 根据姿态矩阵Rbb计算姿态四元数
   q1=1/2*sqrt(abs(1+Rbb(1,1,i)-Rbb(2,2,i)-Rbb(3,3,i)));
   q2=1/2*sqrt(abs(1-Rbb(1,1,i)+Rbb(2,2,i)-Rbb(3,3,i)));
   q3=1/2*sqrt(abs(1-Rbb(1,1,i)-Rbb(2,2,i)+Rbb(3,3,i)));
   q0=sqrt(abs(1-q1^2-q2^2-q3^2));
   if Rbb(2,3,i)-Rbb(3,2,i)<0
       q1=-q1;
   end
   if Rbb(3,1,i)-Rbb(1,3,i)<0
       q2=-q2;
   end
   if Rbb(1,2,i)-Rbb(2,1,i)<0
       q3=-q3;
   end
   Q0=[q0;q1;q2;q3];
   Q0=Q0/norm(Q0);
%    Rk(:,:,i) = R_cov1(P1new,Q0);
   qRk(:,:,i) = R_cov2(P2new,P1new,Q0,Rbb(:,:,i),Tbb(:,i));
   RELMOV(:,i) = [Q0;Tbb(:,i)];
   if mod(i,fix(timeNum/20))==0
        waitbar(i/steps,h);
   end
end
close(h);

VisualRT.Rbb = Rbb ;
VisualRT.Tbb = Tbb ;

visualInputData.VisualRT = VisualRT;
visualInputData.matchedNum = matchedNum;
visualInputData.featureCPosCurrent = featureCPosCurrent;
visualInputData.featureCPosNext = featureCPosNext;


save([pwd,'\VONavResult\spixel.mat'],'spixel');
