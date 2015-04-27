% buaa xyz  2013.12.28

% 捷联惯导粗初始对准，东北天

function SINS_Rough_Initial()
clc
clear
close all
disp('粗对准：')
%% 导入IMU数据
earth_const = getEarthConst();
g = earth_const.g0 ;
wie_c = earth_const.wie ;

[imuInputData_FileName,imuInputData_PathName] = uigetfile({'.mat'},'载入IMU数据',[pwd,'\imuInputData']) ;
imuInputData = importdata([imuInputData_PathName,imuInputData_FileName]);
wib_data = imuInputData.wib ;
f_data = imuInputData.f ;
fre = 100;

fid = fopen([imuInputData_PathName,'\粗对准结果.txt'],'w+');
fprintf(fid,'       粗对准结果记录\n\n');
fprintf(fid,'输入数据源：%s\n',[imuInputData_PathName,imuInputData_FileName]);
%
plot_original_f_w(f_data,wib_data,fre) ;
answer = inputdlg('哪一段属于对准时间？','对准时间',1,{['10  ',num2str(length(f_data)-5)]});
aimTime = sscanf(answer{1},'%f')*fre;

if aimTime(2)>length(f_data)
    aimTime(2) = length(f_data) ;
end
fprintf(fid,'\n采用的时间段：%s\n',answer{1});

f_data = f_data(:,aimTime(1):aimTime(2));
wib_data = wib_data(:,aimTime(1):aimTime(2));
close all
plot_original_f_w(f_data,wib_data,fre) ;
%% 剔除异常
wib_data_new = RejectUnusual_static(wib_data,[50 50 50]*pi/180/3600);% 绝对误差限单位：°/h
f_data_new = RejectUnusual_static(f_data,[1000 1000 1000]*g*1e-6); % 绝对误差限单位：ug
num_f = length(f_data)-length(f_data_new) ;
num_w = length(wib_data)-length(wib_data_new) ;
str=sprintf('陀螺剔除异常数据个数：%d(%0.2f%%)\n加计剔除异常数据个数：%d(%0.2f%%)\n',num_w,num_w/length(wib_data)*100,num_f,num_f/length(f_data)*100);
fprintf(fid,'%s',str);
% 标准差

str=sprintf('%0.4f  ',std(f_data_new,0,2)/g*1e6) ;
fprintf(fid,'\n平滑前-加计标准差:%s   ug\n',str);
display((sprintf('平滑前-加计标准差:%s   ug\n',str)));

str=sprintf('%0.4f  ',std(wib_data_new,0,2)*180/pi*3600) ;
fprintf(fid,'\n平滑前-陀螺标准差:%s   °/h\n',str);
display(sprintf('平滑前-陀螺标准差:%s   °/h\n',str));
%% 平滑
span = 300*100 ; % 平滑步长   秒
f_data_new(1,:) = smooth(f_data_new(1,:),span,'moving');
f_data_new(2,:) = smooth(f_data_new(2,:),span,'moving');
f_data_new(3,:) = smooth(f_data_new(3,:),span,'moving');
wib_data_new(1,:) = smooth(wib_data_new(1,:),span,'moving');
wib_data_new(2,:) = smooth(wib_data_new(2,:),span,'moving');
wib_data_new(3,:) = smooth(wib_data_new(3,:),span,'moving');

% 丢弃平滑后的头尾数据
plot_smooth_f_w(f_data_new,wib_data_new,fre)
answer = inputdlg('丢弃头尾多长时间 sec？','丢弃头尾多长时间 sec？',1,{'10'});
unUseTime = str2double(answer{1});

f_data_new = f_data_new(:,unUseTime*100:(length(f_data_new)-unUseTime*100));
wib_data_new = wib_data_new(:,unUseTime*100:(length(f_data_new)-unUseTime*100));

str=sprintf('%0.4f  ',std(f_data_new,0,2)/g*1e6) ;
fprintf(fid,'\n平滑后-加计标准差:%s   ug\n',str);
display(sprintf('平滑后-加计标准差:%s   ug\n',str));

str=sprintf('%0.4f  ',std(wib_data_new,0,2)*180/pi*3600) ;
fprintf(fid,'\n平滑后-陀螺标准差:%s   °/h\n',str);
display((sprintf('平滑后-陀螺标准差:%s   °/h\n',str)));
%% 多项式最小二乘拟合
g_b_v = zeros(3,1);
wie_b_v = zeros(3,1);
g_b_v(1) = polyfit(1:length(f_data_new),f_data_new(1,:),0);
g_b_v(2) = polyfit(1:length(f_data_new),f_data_new(2,:),0);
g_b_v(3) = polyfit(1:length(f_data_new),f_data_new(3,:),0);
wie_b_v(1) = polyfit(1:length(wib_data_new),wib_data_new(1,:),0);
wie_b_v(2) = polyfit(1:length(wib_data_new),wib_data_new(2,:),0);
wie_b_v(3) = polyfit(1:length(wib_data_new),wib_data_new(3,:),0);
display(g_b_v)

str = sprintf('%0.4f   ',g_b_v') ;
fprintf(fid,'\n加计数据最小二乘拟合结果：%s\n',str);
display(sprintf('加计数据最小二乘拟合结果：%s\n',str));
fprintf(fid,'陀螺数据最小二乘拟合结果：%s\n',str);
display(sprintf('陀螺数据最小二乘拟合结果：%s\n',str));
%% 输入位置
prompt={'初始位置（纬度/°）'};
defaultanswer={' 39.98057 '};
% defaultanswer={'116.35178 39.98057 53.44'};
name='设置视觉导航实验的初始位置';
numlines=1;
answer=inputdlg(prompt,name,numlines,defaultanswer);
initialPosition_d = sscanf(answer{1},'%f'); % 纬度 °
save initialPosition_d initialPosition_d
        
L = initialPosition_d(1)*pi/180;
fprintf(fid,'\n初始位置（纬度/°）:%s\n',answer{1});
%% 解析粗对准算法
% M = [ (gxwie)xg gxwie g ]
% Cnb = M_b /（M_n）
M_b = [ cross(cross(g_b_v,wie_b_v),g_b_v) ,cross(g_b_v,wie_b_v),g_b_v] ;
g_n = [ 0 0 -g ]';
Wie_n = [0  wie_c*cos(L)  wie_c*sin(L)]';
M_n = [ cross(cross(g_n,Wie_n),g_n),cross(g_n,Wie_n),g_n ] ;
Cn2b = M_b / M_n ;

opintions.headingScope=360;
attitude = GetAttitude(Cn2b,'rad',opintions);

disp('姿态角/°')
display(attitude*180/pi)
str_rad = sprintf('%0.5f   ',attitude);
str_degree = sprintf('%0.5f   ',attitude*180/pi);
fprintf(fid,'\n姿态角(°)：%s\n姿态角(rad)：%s\n',str_degree,str_rad);
fclose(fid);
% disp('Cn2b')
% display(Cn2b)
% save Cn2b Cn2b
save([imuInputData_PathName,'\attitude.mat'],'attitude')

plot_new_f_w(f_data_new,wib_data_new,fre);

disp('粗对准结束')

function plot_original_f_w(f_data,wib_data,fre)

time = (1:length(f_data))/fre ;

figure('name','原始加计曲线')
set(gcf,'position',[20,162,672,504])
subplot(3,1,1);
plot(time,f_data(1,:));
title('处理前：acc_x');
subplot(3,1,2);
plot(time,f_data(2,:));
title('处理前：acc_y');
subplot(3,1,3);
plot(time,f_data(3,:));
title('处理前：acc_z');

time = (1:length(wib_data))/fre ;

figure('name','原始陀螺曲线')
set(gcf,'position',[700,162,672,504])
subplot(3,1,1);
plot(time,wib_data(1,:));
title('处理前：gyro_x');
subplot(3,1,2);
plot(time,wib_data(2,:));
title('处理前：gyro_y');
subplot(3,1,3);
plot(time,wib_data(3,:));
title('处理前：gyro_z');

function plot_smooth_f_w(f_data_new,wib_data_new,fre)
time = (1:length(f_data_new))/fre ;

figure('name','平滑后加计曲线')
set(gcf,'position',[700,162,672,504])
subplot(3,1,1);
plot(time,f_data_new(1,:));
title('平滑后：acc_x');
subplot(3,1,2);
plot(time,f_data_new(2,:));
title('平滑后：acc_y');
subplot(3,1,3);
plot(time,f_data_new(3,:));
title('平滑后：acc_z');

time = (1:length(wib_data_new))/fre ;

figure('name','平滑后陀螺曲线')
set(gcf,'position',[20,162,672,504])
subplot(3,1,1);
plot(time,wib_data_new(1,:));
title('平滑后：gyro_x');
subplot(3,1,2);
plot(time,wib_data_new(2,:));
title('平滑后：gyro_y');
subplot(3,1,3);
plot(time,wib_data_new(3,:));
title('平滑后：gyro_z');


function plot_new_f_w(f_data_new,wib_data_new,fre)
time = (1:length(f_data_new))/fre ;

figure('name','最终加计曲线')
set(gcf,'position',[700,162,672,504])
subplot(3,1,1);
plot(time,f_data_new(1,:));
title('最终：acc_x');
subplot(3,1,2);
plot(time,f_data_new(2,:));
title('最终：acc_y');
subplot(3,1,3);
plot(time,f_data_new(3,:));
title('最终：acc_z');

time = (1:length(wib_data_new))/fre ;

figure('name','最终陀螺曲线')
set(gcf,'position',[20,162,672,504])
subplot(3,1,1);
plot(time,wib_data_new(1,:));
title('最终：gyro_x');
subplot(3,1,2);
plot(time,wib_data_new(2,:));
title('最终：gyro_y');
subplot(3,1,3);
plot(time,wib_data_new(3,:));
title('最终：gyro_z');

% -29.006009914653724

