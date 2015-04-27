% buaa xyz 2104.4.26
% 2014.7.26 加入了 Tbc_c 
% 修改了安装角的处理方法
% 
% 从轨迹发生器生成的 trueTrace 到视景仿真软件的输入采样点

clc
clear all
close all
%%　导入真实轨迹（在世界坐标系下－即初始时刻的左摄像机坐标系）
path = uigetdir(pwd,'真实轨迹路径');
trueTrace = importdata([path,'\trueTrace.mat']);
position_true = trueTrace.position ;
attitude_true  = trueTrace.attitude  ;
% velocity_true  = trueTrace.velocity  ;
frequency_true = trueTrace.frequency ;

frestr = inputdlg('视景仿真采样频率');
scenceFre = str2double(frestr);
%% 给定期望初始位置
% 初始的位置可以随便定，但是初始的姿态只能按轨迹发生器中给定的，否则解算时真实轨迹的姿态对不上（因为没处理，位置的处理是使初始时刻为原点）
prompt={'期望初始位置（3Dmax模型坐标系）(m)：(右，上，前)                   . '};
defaultanswer={'0 0 0'};
%defaultanswer={'-40 22 -160'};
name='期望在视景仿真模型中的初始位置';
numlines=1;
answer=inputdlg(prompt,name,numlines,defaultanswer);
initialPosition = sscanf(answer{1},'%f');  
%% 双目系统标定参数
calibData = GetCalibData();
visualInputData.calibData = calibData ; 
% 相机安装角 
cameraSettingAngle = calibData.cameraSettingAngle ;
%% 导航系统中使用的 世界坐标系（初始时刻的左相机坐标系）到 视景仿真软件的模型坐标系 坐标和姿态 转换矩阵
% r（导航系-世界坐标系） 为第一副图时左相机所在的摄像机坐标系
% s（视景仿真模型的坐标和姿态所采用参考系）
% r 的原点在第一幅图处，角度与s相同（注意不是与第一幅图相同），顺序不同：r是
Cr2s_position = [1 0 0;0 0 1;0 1 0];    % xs=xr ys=zr zs=yr  
Cr2s_attitude = [0 0 1;1 0 0;0 1 0];     % 导航系统中顺序是 ： 俯仰 倾斜 航向。视景软件中是 ： 航向 俯仰 倾斜

Cb2sc = FCbn(cameraSettingAngle)'; % 本体 --> 视景仿真的等效摄像机坐标系
Cbb1 = FCbn(cameraSettingAngle)';
Cb1c = [1, 0, 0;     % 本体系到摄像机坐标系:绕x轴转动-90度
       0, 0,-1;     % 摄像机坐标系c： x和y在相机平面，y向下，x向右，z向前
       0, 1, 0];    % 本体系b：x向右，y向前，z向上
Cbc = Cb1c*Cbb1 ;
Tcb_c = calibData.Tcb_c ;
% 初始位置和姿态差
rwc_1 = position_true(:,1)-FCbn(attitude_true(:,1))*Cbc'*Tcb_c ;
dinitialPosition = initialPosition-Cr2s_position * rwc_1 ;  % initialPosition采用3Dmax模型坐标系

%% 
num_trueTrace = length(position_true);
num_scence = fix((num_trueTrace-1)*scenceFre/frequency_true)+1;   % 视景仿真的采样点个数
num_scenceInput = num_scence*2-2 ;

% 输入到视景仿真软件中的位置和姿态,即所需求
scenceInput = zeros(6,num_scenceInput);
% 第一列为单 % 后面都是成双 % 最后为单
rwc_1 = position_true(:,1)-FCbn(attitude_true(:,1))*Cbc'*Tcb_c ;
P = Cr2s_position * rwc_1+dinitialPosition;
C = Cb2sc*FCbn(attitude_true(:,1))' ;
opintions.headingScope = 180 ;
A = GetAttitude(C,'degree',opintions) ;
scenceInput(1:3,1) = P ;
scenceInput(4:6,1) = Cr2s_attitude * A ;  
% attitude_true(:,k_true) 是本体系相对导航系的姿态

rsc = zeros(3,num_scence);  % 视景仿真系下摄像机的位置
rwc = zeros(3,num_scence);  % 视景仿真系下摄像机的位置
rsc(:,1) = P ;
rwc(:,1) = rwc_1;

for k=2:num_scence-1
    k_true = fix((k-1)*frequency_true/scenceFre+1);
    rwc_k_true = position_true(:,k_true)-FCbn(attitude_true(:,k_true))*Cbc'*Tcb_c ;
    P = Cr2s_position * rwc_k_true+dinitialPosition;
    
    C = Cb2sc*FCbn(attitude_true(:,k_true))' ;
    rsc(:,k) = P ;
    rwc(:,k) = rwc_k_true;
    A = GetAttitude(C,'degree',opintions) ;
    
    scenceInput(1:3,2*k-2) = P ;
    scenceInput(4:6,2*k-2) = Cr2s_attitude * A ;
    scenceInput(1:3,2*k-1) = P ;
    scenceInput(4:6,2*k-1) = Cr2s_attitude * A ;  
end
% 最后一个点
k = num_scence ;
k_true = fix((k-1)*frequency_true/scenceFre+1);
rwc_b_k_true = position_true(:,k_true)-FCbn(attitude_true(:,k_true))*Cbc'*Tcb_c ;
P = Cr2s_position * rwc_b_k_true+dinitialPosition;
rsc(:,k) = P ;
rwc(:,k) = rwc_k_true;

C = Cb2sc*FCbn(attitude_true(:,k_true))' ;
A = GetAttitude(C,'degree',opintions) ;
scenceInput(1:3,2*k-2) = P ;
scenceInput(4:6,2*k-2) = Cr2s_attitude * A ;    

scenceInput = scenceInput';
%  不想要视景仿真生成的0号图片时：
scenceInput = [scenceInput(1,:);scenceInput(1,:);scenceInput];
pathStr = sprintf('%s\\scenceInput_%d.txt',path,num_scence) ;
dlmwrite(pathStr,scenceInput,'\t');
save([path,'\scenceInput'], 'scenceInput')
save([path,'\calibData'], 'calibData')
save([path,'\visualInputData'], 'visualInputData')
disp('生成视景仿真 采样点 位置姿态 OK')
disp('注意手动删去最后一行')
disp('手动删除视景仿真生成的0号图')

figure;
plot(rwc(1,:),rwc(2,:),'r',position_true(1,:),position_true(2,:),'g--',rsc(1,:),rsc(3,:),'black-.')
legend('世界系下是摄像机的位置','世界系下载体的位置','视景系下摄像机的位置');
saveas(gcf,[path,'\视景仿真轨迹.fig'])