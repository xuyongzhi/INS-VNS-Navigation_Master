% buaa xyz 2104.1.2

% 从轨迹发生器生成的 trueTrace 到视景仿真软件的输入采样点

clc
clear all
close all
%%　导入真实轨迹（在世界坐标系下－即初始时刻的左摄像机坐标系）
path = uigetdir(pwd,'真实轨迹路径');
trueTrace = importdata([path,'\trueTrace.mat']);
position_true = trueTrace.position ;
attitude_true  = trueTrace.attitude  ;
velocity_true  = trueTrace.velocity  ;
frequency_true = trueTrace.frequency ;

frestr = inputdlg('视景仿真采样频率');
scenceFre = str2double(frestr);
%% 给定期望初始位置
% 初始的位置可以随便定，但是初始的姿态只能按轨迹发生器中给定的，否则解算时真实轨迹的姿态对不上（因为没处理，位置的处理是使初始时刻为原点）
prompt={'期望初始位置（3Dmax模型坐标系）(m)：'};
defaultanswer={'-40 22 -160'};
name='期望在视景仿真模型中的初始位置';
numlines=1;
answer=inputdlg(prompt,name,numlines,defaultanswer);
initialPosition = sscanf(answer{1},'%f');  
% 相机安装角
prompt={'相机安装角(俯仰/倾斜/偏航)(°)'};
defaultanswer={'-3 0 0'};
name='设置相机安装角，不影响视觉解算';
numlines=1;
answer=inputdlg(prompt,name,numlines,defaultanswer);
initialAttitude = sscanf(answer{1},'%f');  

%% 导航系统中使用的 世界坐标系（初始时刻的左相机坐标系）到 视景仿真软件的模型坐标系 坐标和姿态 转换矩阵
% r（导航系-世界坐标系） 为第一副图时左相机所在的摄像机坐标系
% s（视景仿真模型的坐标和姿态所采用参考系）
% r 的原点在第一幅图处，角度与s相同（注意不是与第一幅图相同），顺序不同：r是
Cr2s_position = [1 0 0;0 0 1;0 1 0];    % xs=xr ys=zr zs=yr  
Cr2s_attitude = [0 0 1;1 0 0;0 1 0];     % 导航系统中顺序是 ： 俯仰 倾斜 航向。视景软件中是 ： 航向 俯仰 倾斜

% 初始位置和姿态差
dinitialPosition = initialPosition-Cr2s_position * position_true(:,1) ;  % initialPosition采用3Dmax模型坐标系
dinitialAttitude = Cr2s_attitude*(initialAttitude-attitude_true(:,1)*180/pi); % initialAttitude采用 “俯仰/倾斜/偏航”
%% 
num_trueTrace = length(position_true);
num_scence = fix(num_trueTrace*scenceFre/frequency_true);   % 视景仿真的采样点个数
num_scenceInput = num_scence*2-2 ;

% 输入到视景仿真软件中的位置和姿态,即所需求
scenceInput = zeros(6,num_scenceInput);

scenceInput(1:3,1) = Cr2s_position * position_true(:,1)+dinitialPosition;
scenceInput(4:6,1) = Cr2s_attitude * attitude_true(:,1)*180/pi+dinitialAttitude;  
for k=1:num_scence-2
    k_true = fix(k*frequency_true/scenceFre+1);
    scenceInput(1:3,2*k) = Cr2s_position * position_true(:,k_true)+dinitialPosition;
    scenceInput(4:6,2*k) = Cr2s_attitude * attitude_true(:,k_true)*180/pi+dinitialAttitude;    
    scenceInput(1:3,2*k+1) = Cr2s_position * position_true(:,k_true)+dinitialPosition;
    scenceInput(4:6,2*k+1) = Cr2s_attitude * attitude_true(:,k_true)*180/pi+dinitialAttitude;   
end
k = num_scence-1 ;
k_true = fix(k*frequency_true/scenceFre+1);
scenceInput(1:3,2*k) = Cr2s_position * position_true(:,k_true)+dinitialPosition;
scenceInput(4:6,2*k) = Cr2s_attitude * attitude_true(:,k_true)*180/pi+dinitialAttitude;    

scenceInput = scenceInput';
%  不想要视景仿真生成的0号图片时：
scenceInput = [scenceInput(1,:);scenceInput(1,:);scenceInput];

dlmwrite([path,'\scenceInput.txt'],scenceInput,'\t');
disp('生成视景仿真 采样点 位置姿态 OK')
disp('注意手动删去最后一行')
