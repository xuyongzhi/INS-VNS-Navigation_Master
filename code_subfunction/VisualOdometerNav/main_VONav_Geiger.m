%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   调用 Geiger, Andreas 的 libviso2 library
%                       buaaxyz 2014.8.15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [VOResult,visualInputData] = main_VONav_Geiger(visualInputData,trueTrace,timeShorted)

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
    timeShorted = 1;
end
save visualInputData visualInputData
save trueTrace trueTrace

isUpdate = 1 ;
if isfield(visualInputData,'Tr_total')
       button=questdlg('是否重新载入图片'); 
       if strcmp(button,'No')
           isUpdate = 0 ;            
       end
end


if isUpdate==1
    if ~isfield(visualInputData,'calibData')
        visualInputData.calibData = loadCalibData();  
    else
        button=questdlg('是否更新相机标定参数?'); 
        if strcmp(button,'Yes')
            visualInputData.calibData = loadCalibData();  
        end
    end
    if ~isempty(trueTrace)
        true_position = trueTrace.position ;
        trueTraeFre = trueTrace.frequency;
        vsFre = visualInputData.frequency;
        true_position_vsFre = decreaseDataFre( true_position,trueTraeFre,vsFre ) ;
    end
   [ Tr_total,Tr_pc,featureImagePos,visualInputData,Cbc,Tcb_c ] = libviso2( visualInputData,true_position_vsFre ) ; 
else
    % 直接读取已经解算好的 Tr_total,Tr_pc,featureImagePos
    Tr_total = visualInputData.Tr_total  ;
    Tr_pc = visualInputData.Tr_pc ;
    featureImagePos = visualInputData.featureImagePos ;
    [ Cbc,Tcb_c,~,~,~,~,~,~,~,~,~,~,~ ] = ExportCalibData( visualInputData.calibData ) ;
end

%% kitti 的视觉解算结果 -> 我的格式
[ visualInputData,VOsta,VOpos,isKnowTrue,VOstaError,VOposError,combineFre,trueTraeFre,true_position,runTime_image ] = Tr_total_to_VOResult( Tr_total,trueTrace,Cbc,Tcb_c,visualInputData ) ;
visualInputData = GetFeatureLoc( featureImagePos,visualInputData );
visualInputData = Tr_pc_to_RT( Tr_pc,Cbc,Tcb_c,visualInputData );
if isKnowTrue==1
    [ trueTbb,trueRbb  ] = GetTrueTbbRbb(trueTrace,visualInputData.frequency,1) ;
    visualInputData.VisualRT.trueTbb = trueTbb ;
    visualInputData.VisualRT.trueRbb = trueRbb ;
    
end
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
VOfre = visualInputData.frequency ;
VOsta_trueRbb=[];
VOvel=[];
VOvelError=[];
VOsta_trueRbb_error=[];
VOsta_trueTbb_error=[];
VOStaStepError=[];
VOStaStepError_A=[];
VOStaStepError_B=[];
VOCrbError=[];
VOCrcError=[];
VOrcAngle=[];
VOStaStepError_Adefine=[];
angle_bb=[];
Tbb_sel=[];
true_angle_bb=[];
trueTbb=[];

[ visualInputData,VOsta_trueRbb,VOsta_trueTbb,VOsta_trueRbb_trueTbb,VOsta_trueRbb_error,VOsta_trueTbb_error,VOsta_trueRbb_trueTbbError ]  =GetDebugVOResult( visualInputData,trueTrace );

VOResult = saveVOResult_subplot( isKnowTrue,VOfre,VOsta,VOsta_trueRbb,VOpos,VOvel,matchedNum,aveFeatureNum,...
    VOposError,VOvelError, VOstaError,VOsta_trueRbb_error,VOsta_trueTbb_error,combineFre,trueTraeFre,true_position,VOStaStepError,VOStaStepError_A,...
    VOStaStepError_B,VOCrbError,VOCrcError,VOrcAngle,VOStaStepError_Adefine,angle_bb,Tbb_sel,true_angle_bb,trueTbb,AngleError,TbbError,runTime_image );

save([pwd,'\VONavResult\VOResult.mat'],'VOResult')
assignin('base','VOResult',VOResult)
VisualRT = visualInputData.VisualRT;
save([pwd,'\VONavResult\VisualRT.mat'],'VisualRT')
assignin('base','VisualRT',VisualRT)

save visualInputData visualInputData
errorStr = visualInputData.errorStr ;
save errorStr errorStr
disp('kitti 纯视觉实验导航解算结束')
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
plot(VOsta_trueRbb_trueTbb(1,:),VOsta_trueRbb_trueTbb(2,:),'-.m');
legend('trueTrace','VO','trueRbb','trueTbb','trueTbb_trueRbb','fontsize',5);
%legend('trueTrace','VO','trueRbb','trueTbb','trueRT');
saveas(gcf,'视觉导航轨迹.fig')

%% 减小数据频率
% 存储：3*N
% fre_new < fre_old
function data_new = decreaseDataFre( data_old,fre_old,fre_new )
N = size(data_old,2) ;
N_new = fix( (N-1)*fre_new/fre_old) +1 ;
data_new = zeros(3,N_new) ;
for k_new=1:N_new
    k_old = fix( (k_new-1)*fre_old/fre_new) +1  ;
    data_new(:,k_new) = data_old(:,k_old) ;
end

function calibData = loadCalibData()
% 选择是否加入相机标定参数。添加：如果是真实实验采集，则加载，如果是视景仿真采集，则计算。
global projectDataPath
button =  questdlg('根据图片获取的方法选择','添加标定数据','视景仿真：计算标定参数','真实实验：载入标定参数文件','不添加','视景仿真：计算标定参数') ;
if strcmp(button,'视景仿真：计算标定参数')
    calibData = GetCalibData() ;
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

function [ Tr_total,Tr_pc,featureImagePos,visualInputData,Cbc,Tcb_c ] = libviso2( visualInputData,true_position )
%% Andreas 的 libviso2
% 载入图片进行视觉导航解算
% 设置图片格式和地址 
global projectDataPath
if ~exist('projectDataPath','var')
    projectDataPath = 'E:\惯性视觉导航\NAVIGATION\data_old\kitti\raw data\2011_09_26_drive_0048\2011_09_26_drive_0048_sync';
end
[~,~,first_frame,last_frame]=ReadImage('SetImagePath') ;
[ Cbc,Tcb_c,T,alpha_c_left,alpha_c_right,cc_left,cc_right,fc_left,fc_right,kc_left,kc_right,om,calibData ] = ExportCalibData( visualInputData.calibData ) ;
visualInputData.calibData = calibData ;
Ccb = Cbc';

param.f     = (fc_left(1)+fc_left(2)+fc_right(1)+fc_right(2))/4 ;     %??????? 不能左右区分吗？
param.cu    = ( cc_left(1)+cc_right(1) )/2 ;
param.cv    = ( cc_left(2)+cc_right(2) )/2 ;
param.base  = -T(1)/1000 ;  % 基线 m  , T: [ -B 0 0 ] mm
    %%%     不考虑左右的角度？？
% first_frame = 0;
% last_frame  = lastImageN ;

% init visual odometry
visualOdometryStereoMex('init',param);

% init transformation matrix array
% 初始化变量：这些变量的第一个都不要
imageN = last_frame-first_frame+1 ;
Tr_total = cell(imageN,1);    % 世界系下 摄像机的位置 姿态
Tr_total{1} = eye(4);
Tr_pc = cell(imageN,1);       % p->c的旋转和平移
featureImagePos = cell(imageN,1); % 特征点像素坐标
num_matches = zeros(1,imageN) ;

% create figure
figure('Color',[1 1 1]);
ha1 = axes('Position',[0.05,0.7,0.9,0.25]);
axis off;
ha2 = axes('Position',[0.05,0.05,0.9,0.6]);
set(gca,'XTick',-500:10:500);
set(gca,'YTick',-500:10:500);
axis equal, grid on, hold on;

% for all frames do
for frame=first_frame:last_frame
  
  % 1-index
  k = frame-first_frame+1;  % 永远从1开始
  
  % read current images
%   I1 = imread([img_dir '/I1_' num2str(frame,'%06d') '.png']);
%   I2 = imread([img_dir '/I2_' num2str(frame,'%06d') '.png']);

  [I1,I2] = ReadImage('GetImage',frame) ;     
  
  % compute and accumulate egomotion
  Tr = visualOdometryStereoMex('process',I1,I2);
  Tr_pc{k} = Tr ;     % Tr_pc{1} 无效
  if k>1
    Tr_total{k} = Tr_total{k-1}*inv(Tr);
  end
  featureImagePos{k} = visualOdometryStereoMex('get_matches');

  % update image
  axes(ha1); cla;
  imagesc(I1); colormap(gray);
  axis off;
  
  % update trajectory
  axes(ha2);
  if k>1
    plot([Tr_total{k-1}(1,4) Tr_total{k}(1,4)], ...
         [Tr_total{k-1}(3,4) Tr_total{k}(3,4)],'-b','LineWidth',1);
%      hold on 
%      plot([true_position(1,k-1),true_position(1,k)], ...
%          [true_position(2,k-1),true_position(2,k)],'-r','LineWidth',1);
  end
  
  
  pause(0.01); refresh;

  % output statistics
  num_matches(k) = visualOdometryStereoMex('num_matches');
  num_inliers = visualOdometryStereoMex('num_inliers');
  disp(['Frame: ' num2str(frame) ...
        ', Matches: ' num2str(num_matches(k)) ...
        ', Inliers: ' num2str(100*num_inliers/num_matches(k),'%.1f') ,' %']);
    
    if mod(frame,50)==0
        save Tr_total Tr_total
        save featureImagePos featureImagePos
        save num_matches num_matches
    end
end

% release visual odometry
visualOdometryStereoMex('close');

visualInputData.Tr_total = Tr_total ;
visualInputData.Tr_pc = Tr_pc ;
visualInputData.featureImagePos = featureImagePos ;

save Tr_total Tr_total
save featureImagePos featureImagePos
save Tr_pc Tr_pc


%% 由 Tr_total (左摄像机在初始摄像机坐标系下的位置姿态) 到 本体系中心 在世界系下的位置姿态
% Tr_total中包含的是初始时刻摄像机系下的位置和姿态
% 计算出 我定义的世界系 下位置和姿态
function [ visualInputData,VOsta,VOpos,isKnowTrue,VOstaError,VOposError,combineFre,trueTraeFre,true_position,runTime_image ] = Tr_total_to_VOResult( Tr_total,trueTrace,Cbc,Tcb_c,visualInputData )

%% 初始摄像机系下的位置和姿态
N = length(Tr_total) ;  % 采样数
sta_c = zeros(3,N) ;    % 初始摄像机系 下的位置
posR_c = zeros(3,3,N) ;    % 初始摄像机系 下的姿态矩阵
opintions.headingScope = 180  ;
for k=1:N
    sta_c(:,k) = Tr_total{k}(1:3,4) ;
    Rc2w = Tr_total{k}(1:3,1:3) ;
    posR_c(:,:,k) = Rc2w' ;
end

%% 位置 姿态 转换到 世界坐标系
[planet,isKnowTrue,initialPosition_e,initialVelocity_r,initialAttitude_r,trueTraeFre,true_position,...
    true_attitude,true_velocity,true_acc_r,runTime_IMU,runTime_image] = GetFromTrueTrace( trueTrace );
if isempty(trueTrace)
   % 需要再次给定 initialVelocity_r initialAttitude_r，初始位置总为[0;0;0]
   answer = inputdlg({'初始姿态(俯仰,横滚,航向)°                                          . ','初始速度(m/s)'},'独立视觉导航 - 给定初始条件',1,{'0 0 0','0 0 0'});
   initialVelocity_r = sscanf(answer{2},'%f');
   initialAttitude_r = sscanf(answer{1},'%f')*pi/180;   
   isKnowTrue=0;
end

VOsta = zeros( 3,N );
VOpos = zeros( 3,N );
CrbSave = zeros( 3,3,N );
VOsta(:,1) = [0;0;0] ;  % 初始位置:以初始时刻左摄像机坐标系为原点
VOpos(:,1) = initialAttitude_r;   %初始姿态
Cbr=FCbn(VOpos(:,1));
Crb=Cbr';
% 世界系为 w ，初始相机系为 f
Rwf = Cbc * Crb ;
for k=1:N
   Rwb =  Cbc' * posR_c(:,:,k) * Rwf ;
   VOsta(:,k) = ( Rwb'* Cbc'- Rwf' )*Tcb_c + Rwf'*sta_c(:,k) ;
   VOpos(:,k) = GetAttitude(Rwb,'rad',opintions) ;
   CrbSave(:,:,k) = Rwb ;
end

VOfre = visualInputData.frequency ;
%% 已知真实：计算误差
if  isKnowTrue==1
    % 计算组合数据有效长度
    lengthArrayOld = [length(VOsta),length(true_position)];
    frequencyArray = [VOfre,trueTraeFre];
    [validLenthArray,combineK,combineLength,combineFre] = GetValidLength(lengthArrayOld,frequencyArray);
    VOstaError = zeros(3,combineLength);   
    VOposError = zeros(3,combineLength);

    for k=1:combineLength
        k_true = fix((k-1)*(trueTraeFre/combineFre))+1 ;
        k_VO = fix((k-1)*(VOfre/combineFre))+1;
        
        VOstaError(:,k) = VOsta(:,k_VO)-true_position(:,k_true) ;
        VOposError(:,k) = VOpos(:,k_VO)-true_attitude(:,k_true) ;
        VOposError(3,k) = YawErrorAdjust(VOposError(3,k),'rad') ;
    end    
    % 计算空间二维/三维位置误差及相对值
    errorStr = CalPosErrorIndex_route( true_position,VOstaError,VOposError*180/pi*3600,VOsta );
else
    errorStr = '误差未知';
    VOposError = [];VOvelError=[];VOstaError=[];combineFre=[];VOStaStepError=[];
end
visualInputData.errorStr = errorStr ;

% figure('name','视觉导航轨迹')
% 
% hold on
% plot(VOsta(1,:),VOsta(2,:),'r','linewidth',1.3);
% plot( true_position(1,:),true_position(2,:),'g','linewidth',1.3 )
% 
% % plot(VOsta_trueRbb_trueTbb(1,:),VOsta_trueRbb_trueTbb(2,:),'-.m');
% legend('VO','trueTrace');
% saveas(gcf,'视觉导航轨迹.fig')

%% 计算调试视觉导航结果 ： 分别将 Rbb Tbb 置换为 trueTbb trueRbb
% isTbb_last=1
function [ visualInputData,VOsta_trueRbb,VOsta_trueTbb,VOsta_trueRbb_trueTbb,VOsta_trueRbb_error,VOsta_trueTbb_error,VOsta_trueRbb_trueTbbError ]  =GetDebugVOResult( visualInputData,trueTrace )
VisualRT = visualInputData.VisualRT  ;
trueTbb = VisualRT.trueTbb  ;
trueRbb = VisualRT.trueRbb ;
Rbb = VisualRT.Rbb ;
Tbb_last = VisualRT.Tbb_last ;
VOfre = visualInputData.frequency ;

[RTerrorStr,AngleError,TbbError] = analyseRT(Rbb,Tbb_last,trueRbb,trueTbb);

[~,isKnowTrue,~,~,initialAttitude_r,trueTraeFre,true_position,...
    true_attitude,~,~,~,~] = GetFromTrueTrace( trueTrace );

imageN = size(trueTbb,2)+1 ;

VOsta_trueRbb = zeros(3,imageN);    % 无姿态误差时的视觉导航位置
VOsta_trueTbb = zeros(3,imageN);    % 无Tbb误差时的视觉导航位置
VOsta_trueRbb_trueTbb = zeros(3,imageN);

Cbr=FCbn( initialAttitude_r );
Crb=Cbr';
Cbr_true = Cbr ;
%% 运动积分
% compute the path -- in local level coordinate
% pos = zeros(3,imageN+1);
for i = 1:imageN-1
    % Rbb(:,:,i)是 b(i)到b(i+1)的旋转矩阵
    % Tbb(:,i)  是 b(i)到b(i+1)的平移矩阵
    Crb_last  = Crb ;
    Crb = Rbb(:,:,i) * Crb;
    Cbr_true_last = Cbr_true ;
    Cbr_true = Cbr_true * trueRbb(:,:,i)' ;
    
    k_true = fix((i-1)*(trueTraeFre/VOfre))+1 ;
   	k_true_next = fix((i)*(trueTraeFre/VOfre))+1 ;
    Cbr_true_last = FCbn(true_attitude(:,k_true)) ;
    
    VOsta_trueRbb(:,i+1) = VOsta_trueRbb(:,i) + Cbr_true_last * Tbb_last(:,i);
    VOsta_trueTbb(:,i+1) = VOsta_trueTbb(:,i) + Crb_last' * trueTbb(:,i);
    VOsta_trueRbb_trueTbb(:,i+1) = VOsta_trueRbb_trueTbb(:,i) + Cbr_true_last * trueTbb(:,i);  
end
VOfre = visualInputData.frequency ;
%%  计算误差
% 计算组合数据有效长度
lengthArrayOld = [imageN,length(true_position)];
frequencyArray = [VOfre,trueTraeFre];
[~,~,combineLength,combineFre] = GetValidLength(lengthArrayOld,frequencyArray);
VOsta_trueRbb_error = zeros(3,combineLength);   
VOsta_trueTbb_error = zeros(3,combineLength);   
VOsta_trueRbb_trueTbbError = zeros(3,combineLength);   
for k=1:combineLength
    k_true = fix((k-1)*(trueTraeFre/combineFre))+1 ;
    k_VO = fix((k-1)*(VOfre/combineFre))+1;

    VOsta_trueRbb_error(:,k) = VOsta_trueRbb(:,k_VO)-true_position(:,k_true) ;
    VOsta_trueTbb_error(:,k) = VOsta_trueTbb(:,k_VO)-true_position(:,k_true) ;
    VOsta_trueRbb_trueTbbError(:,k) = VOsta_trueRbb_trueTbb(:,k_VO)-true_position(:,k_true) ;
end    
% 计算空间二维/三维位置误差及相对值
errorStr = visualInputData.errorStr ;
% 真实Rbb视觉解算误差
errorStr_trueRbb = CalPosErrorIndex_route( true_position,VOsta_trueRbb_error,[],VOsta_trueRbb );
% 真实Tbb视觉解算误差
errorStr_trueTbb = CalPosErrorIndex_route( true_position,VOsta_trueTbb_error,[],VOsta_trueTbb );
errorStr_trueTbb_trueRbb = CalPosErrorIndex_route( true_position,VOsta_trueRbb_trueTbbError,[],VOsta_trueRbb_trueTbb );
errorStr = sprintf('%s真实Rbb时视觉导航位置误差：\n%s真实Tbb时视觉导航位置误差：\n%s真实Tbb+真实Rbb时视觉导航位置误差：\n%s',errorStr,errorStr_trueRbb,errorStr_trueTbb,errorStr_trueTbb_trueRbb);
errorStr = sprintf('%s\n%s\n',errorStr,RTerrorStr);

visualInputData.errorStr = errorStr ;


%% 运动积分检验 Rbb Tbb -> 位置 姿态

function RT_to_Sta()

%% 运动积分 检验
timeNum = N-1 ;
VOsta = zeros(3,timeNum+1);
VOpos = zeros(3,timeNum+1);
VOvel = zeros(3,timeNum+1);
VOsta(:,1) = [0;0;0] ;  % 初始位置:以初始时刻左摄像机坐标系为原点
VOpos(:,1) = initialAttitude_r;   %初始姿态
VOvel(:,1) = initialVelocity_r;    % 初始速度

VOsta_trueRbb = zeros(3,timeNum+1);    % 无姿态误差时的视觉导航位置
VOsta_trueTbb = zeros(3,timeNum+1);    % 无Tbb误差时的视觉导航位置
VOsta_trueRbb_trueTbb = zeros(3,timeNum+1);

CrbSave = zeros(3,3,timeNum+1);

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

%% 由 featureImagePos 得到 特征点像素坐标
% 注意输入变量的第一个无效
function visualInputData = GetFeatureLoc( featureImagePos,visualInputData )

N = length( featureImagePos )-1 ; 
matchedNum = zeros(1,N) ;
leftLocCurrent = cell(1,N) ;    % 前一时刻 左图 图像 像素 坐标  [n*2]
rightLocCurrent = cell(1,N) ;   % 前一时刻 左图 图像 像素 坐标
leftLocNext = cell(1,N) ;       % 后一时刻 左图 图像 像素 坐标
rightLocNext = cell(1,N) ;      % 后一时刻 右图 图像 像素 坐标

for k=2:N+1   
    featureImagePos_k = featureImagePos{k} ;
    matchedNum(k-1) = size(featureImagePos_k,2) ;
    leftLocCurrent{k-1} = featureImagePos_k(1:2,:) ;
    rightLocCurrent{k-1} = featureImagePos_k(3:4,:) ;
    leftLocNext{k-1} = featureImagePos_k(5:6,:) ;
    rightLocNext{k-1} = featureImagePos_k(7:8,:) ;
end

visualInputData.leftLocCurrent = leftLocCurrent ;
visualInputData.rightLocCurrent = rightLocCurrent ;
visualInputData.leftLocNext = leftLocNext ;
visualInputData.rightLocNext = rightLocNext ;
visualInputData.matchedNum = matchedNum;


%% 由Tr_pc 得到 Rcc Tcc
% 注意输入变量的第一个无效

function visualInputData = Tr_pc_to_RT( Tr_pc,Cbc,Tcb_c,visualInputData )
N = size( Tr_pc,1 )-1 ; 
Rcc = zeros(3,3,N) ;
Tcc = zeros(3,N) ;
Tcc_last = zeros(3,N) ;
Rbb = zeros(3,3,N) ;
Tbb = zeros(3,N) ;
Tbb_last = zeros(3,N) ;

% 以下参考学习笔记
% 注意Rbb从第一个开始有效，Rbb(1)表示1到2图的旋转
% Tr_pc输入变量的第一个无效,Tr_pc{2}表示1到2图的旋转
for k=1:N     
    Tr_pc_k = Tr_pc{k+1} ;
    
    Tcc(:,k) = -Tr_pc_k(1:3,4) ;
    Rpc = Tr_pc_k(1:3,1:3) ;
    Rcc(:,:,k) = Rpc ;
    
    Tcc_last(:,k) = -Rpc' * Tr_pc_k(1:3,4) ;
    
    Tbb_last(:,k) = Cbc' * Tcc_last(:,k) + Cbc' * (Rpc'-eye(3)) * Tcb_c ;    
    Rbb(:,:,k) = Cbc'* Rpc * Cbc ;
    Tbb(:,k) = Rbb(:,:,k) * Tbb_last(:,k) ;
end

VisualRT.Rbb = Rbb ;
VisualRT.Tbb_last = Tbb_last ;
VisualRT.Tcc_last = Tcc_last ;
VisualRT.Rcc = Rcc ;

visualInputData.VisualRT = VisualRT;
