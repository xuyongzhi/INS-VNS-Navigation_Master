% 从 kitti 的 2011_09_26_drive_0048_extract\oxts 获取真实轨迹和 IMU数据

% (1) lat:   latitude of the oxts-unit (deg)  注意这个单位是度
% (2) lon:   longitude of the oxts-unit (deg)
% (3) alt:   altitude of the oxts-unit (m)
% (4) roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
% (5) pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
% (6) yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
% (7) vn:    velocity towards north (m/s)
% (8) ve:    velocity towards east (m/s)
% (9) vf:    forward velocity, i.e. parallel to earth-surface (m/s)
% (10) vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
% (11) vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
% (12) ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
% (13) ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
% (14) ay:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
% (15) af:    forward acceleration (m/s^2)
% (16) al:    leftward acceleration (m/s^2)
% (17) au:    upward acceleration (m/s^2)
% (18) wx:    angular rate around x (rad/s)
% (19) wy:    angular rate around y (rad/s)
% (20) wz:    angular rate around z (rad/s)
% (21) wf:    angular rate around forward axis (rad/s)
% (22) wl:    angular rate around leftward axis (rad/s)
% (23) wu:    angular rate around upward axis (rad/s)
% (24) pos_accuracy:  velocity accuracy (north/east in m)
% (25) vel_accuracy:  velocity accuracy (north/east in m/s)
% (26) navstat:       navigation status (see navstat_to_string)
% (27) numsats:       number of satellites tracked by primary GPS receiver
% (28) posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
% (29) velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
% (30) orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)

% 注：
%  （1）ax~az wx~wz 的xyz指的是本体系
%  （2）vf~vu af~au wf~wu 中的 f、l、u指的是 本体系映射到当地水平面的坐标系，也就是说
%       f、l在水平面内，但f的指向沿着本体系的航向，u则与地理系的天向重合。
%  （3）kitti中，航向角为0指向东，我的系统中航向为0为指向北。kitti中，横滚角左边抬起为正，俯仰角低头为正；我的系统中，俯仰角抬头为正，横滚角左边抬起为正。
%           kitti的姿态->我(mine)的姿态：pitch_mine = roll_kitti（方向凑巧是一致的），
%           roll_mine = pitch_kitti(方向凑巧也是一致的),yaw_mine = yaw_kitti-pi/2
%  （4）kitti中，本体系：bx指向前，by指向左。我的系统中，bx指向右，by指向前。
%           kitti本体系->我的本体系：bx_mine = -by_kitti;by_mine = bx_kitti;bz_mine = bz_kitti
%   (5) kitti中图片的时间不是严格的 10HZ，有时会有0.01~0.05s的波动，且波动好像会累积
%   (6) kitti中oxt_extract的时间，一般是严格100HZ的，但是，经常会发生 突变、丢失时间、甚至时间反回 的问题
%   # 数据存储规则：不同类型数据的对齐上符合频率规则，通过runTime记录确切的时间
%%
function get_GPS_IMU_kitti()
format long
dbstop error
clc
clear all
close all

 raw_data_dir = 'E:\惯性视觉导航\NAVIGATION\data_kitVO\';
%raw_data_dir = 'O:\KITTI\';
pro_name = '2011_09_26_drive_0096';
 % pro_name='2011_10_03_drive_0034';
prodir = [raw_data_dir,pro_name];
base_sync_dir = [prodir,'\',pro_name,'_sync'];
base_extract_dir = [prodir,'\',pro_name,'_extract'];
%% 读取原始GPS-IMU数据
if exist(base_extract_dir,'file')
   isLoad_extract = 1;
else
    isLoad_extract=0;
end
if isLoad_extract==1
    oxts_extract_old = loadOxtsliteData(base_extract_dir);
    % 100HZ 未处理GPS_IMU数据的采样时间
    ts_oxts_extract = loadTimestamps([base_extract_dir,'\oxts']) ;
    runTime_IMU_extract_old = ts_to_runTime(ts_oxts_extract) ;
    stepTime_IMU_extract_old = runTime_to_setpTime(runTime_IMU_extract_old) ;
    figure
    plot(runTime_IMU_extract_old)
    title('原始未处理的 100HZ IMU数据采集时间')
    
    figure
    plot(stepTime_IMU_extract_old)
    title('原始未处理的 100HZ IMU数据采集时间 步长')
end
oxts_sync = loadOxtsliteData(base_sync_dir)';


%% 10HZ 处理后GPS_IMU数据的采样时间
ts_oxts_sync = loadTimestamps([base_sync_dir,'\oxts']) ;
% 10HZ 左图片时间
ts_image0_sync = loadTimestamps([base_sync_dir,'\image_00']) ;

%% 100HZ IMU数据提取：剔除 oxts_extract_old 的无效头尾  -> oxts_extract_new
% 计算 oxts_extract_new 在oxts_extract_old的第一个序号 k_extract_old_start
if isLoad_extract==1
    for j=1:length(ts_oxts_extract)
       if strcmp(ts_oxts_extract{j},ts_oxts_sync{1}) 
           k_extract_old_start = j;
           break;
       end
    end
    % 计算 oxts_extract_new 在oxts_extract_old最后一个序号 k_extract_old_end
    for j=1:length(ts_oxts_extract)
       if strcmp(ts_oxts_extract{j},ts_oxts_sync{length(ts_oxts_sync)}) 
           k_extract_old_end = j;
           break;
       end
    end

    oxts_extract_new_N = k_extract_old_end-k_extract_old_start+1 ;   
    % 截取得到 oxts_extract_new
    oxts_extract_new = oxts_extract_old(k_extract_old_start:k_extract_old_end) ;    % 100HZ GPS-IMU（最后代入导航程序的数据）
    ts_oxts_extract_new = ts_oxts_extract(k_extract_old_start:k_extract_old_end) ;
    isPic_IMUFlag = zeros(oxts_extract_new_N,1);            % 记录当前 GPS-IMU 时间是否对应着一幅图的时刻，0/1，为1则在这个时刻进行信息融合
    isIMU_PicFlag = zeros(length(ts_oxts_sync),1);
end

%% runTime
if isLoad_extract==1
    runTime_IMU_extract = ts_to_runTime(ts_oxts_extract_new) ;
    %%% runTime_IMU_extract
    runTime_IMU_extract_error = zeros(oxts_extract_new_N,1);
    for n=1:oxts_extract_new_N
        runTime_IMU_extract_error(n) = runTime_IMU_extract(n)-(n-1)*0.01;
    end
    figure('name','未同步 100HZ的IMU数据 与频率之间的绝对误差')
    plot(runTime_IMU_extract_error)
    title('未同步 100HZ的IMU数据 与频率之间的绝对误差')
    saveas(gcf,[prodir,'\未同步 100HZ的IMU数据 与频率之间的绝对误差.fig'])

end
runTime_IMU_sync = ts_to_runTime(ts_oxts_sync) ;
runTime_image0_sync = ts_to_runTime(ts_image0_sync) ;
stepTime_IMU_sync = runTime_to_setpTime(runTime_IMU_sync) ;
stepTime_image0_sync = runTime_to_setpTime(runTime_image0_sync) ;
%%% runTime_IMU_sync
runTime_IMU_sync_error = zeros(length(runTime_IMU_sync),1);
for n=1:length(runTime_IMU_sync)
    runTime_IMU_sync_error(n) = runTime_IMU_sync(n)-(n-1)*0.1;
end

%%% runTime_image0_sync
runTime_image0_sync_error = zeros(length(runTime_image0_sync),1);
for n=1:length(runTime_image0_sync)
    runTime_image0_sync_error(n) = runTime_image0_sync(n)-(n-1)*0.1;
end

figure('name','同步后 10HZ 图像 和 IMU时间 与频率之间的绝对误差')
hold on
title('同步后 10HZ 图像 和 IMU时间 与频率之间的绝对误差')
plot(runTime_IMU_sync_error,'color','r')
plot(runTime_image0_sync_error,'color','b')
legend('10HZ IMU','10HZ 图像')
saveas(gcf,[prodir,'\同步后 10HZ 图像 和 IMU时间 与频率之间的绝对误差.fig'])

figure('name','同步后 10HZ 图像 和 IMU时间')
hold on
title('同步后 10HZ 图像 和 IMU时间')
plot(runTime_IMU_sync,'color','r')
plot(runTime_image0_sync,'color','b')
legend('10HZ IMU','10HZ 图像')

saveas(gcf,[prodir,'\同步后 10HZ 图像 和 IMU时间.fig'])

figure
plot([stepTime_IMU_sync stepTime_image0_sync])
legend('纠正后10HZ IMU采集 步长','视觉采集 步长')
title('采集步长')
 %% 求 isPic_IMUFlag
% k_extract_old_search = 1;
% k_extract_new_last = 1;
% for k_extract_new=1:oxts_extract_new_N
% % 判断此时刻是否对应着 图片
%     dT = abs(runTime_IMU_extract(k_extract_new)-runTime_IMU_sync(k_extract_old_search)) ;
%     if dT<1/100/2
%         if k_extract_new>1
%             k_step = k_extract_new-k_extract_new_last ;
%             if k_step>13 || k_step<8
%                 errordlg(sprintf('是否对应图片的判断，第%d步出错！',k_extract_new));
%             end
%             k_extract_new_last = k_extract_new ;
%         end
%         k_extract_old_search = k_extract_old_search+1 ;
%         isPic_IMUFlag(k_extract_new) = 1;
%     end
% end
% if isPic_IMUFlag(oxts_extract_new_N)~=1
%     errordlg('isPic_IMUFlag最后一个不是1，出错！');
% end

%% 把 oxts_extract_new 转换为 trueTrace 格式 imuInputData 格式
% 数据存储规则：不同类型数据的对齐上符合频率规则，通过runTime记录确切的时间
% 取校正后的10HZ数据
[imuInputData,trueTrace,IMU_data_t,trueTrace_data_t,pos_accuracy,vel_accuracy] = oxts_to_imuInputData_trueTrace(oxts_sync) ;  

disp('取同步之后的10HZ数据')

imuInputData.runTime = runTime_IMU_sync ;
imuInputData.frequency = 10 ;
imuInputData=IMU_sub1(imuInputData);

trueTrace.runTime_IMU = runTime_IMU_sync ;
trueTrace.runTime_image = runTime_image0_sync ;
trueTrace.frequency = 10 ;

IMU_data_t.runTime_IMU = runTime_IMU_sync ;
IMU_data_t.frequency = 10 ;
trueTrace_data_t.frequency = 10 ;
trueTrace_data_t.runTime_image = runTime_image0_sync ;
trueTrace_data_t.runTime_IMU = runTime_IMU_sync ;

lon_lat_alt = trueTrace.lon_lat_alt ;
position_w = trueTrace.position ;
attitude_w = trueTrace.attitude ;
velocity_w = trueTrace.velocity ;

wholeLength = CalRouteLength( trueTrace.position );
describe = sprintf('路程长度：%0.2f m\t时间：%0.2f sec',wholeLength,runTime_IMU_sync(length(runTime_IMU_sync)));
display(describe)
trueTrace.describe = describe ;

save imuInputData imuInputData
save trueTrace trueTrace
save([prodir,'\imuInputData'],'imuInputData')
save([prodir,'\imuInputData_measure'],'imuInputData')
save([prodir,'\trueTrace'],'trueTrace')
disp('imuInputData/imuInputData_measure：直接测量的IMU数据')
disp('imuInputData有可能被替换成反推的IMU数据')
% 保存所有地理系导航解算需要用到的数据 IMU_data_t trueTrace_data_t
save IMU_data_t IMU_data_t
save trueTrace_data_t trueTrace_data_t
save([prodir,'\IMU_data_t'],'IMU_data_t')
save([prodir,'\trueTrace_data_t'],'trueTrace_data_t')
disp('IMU_data_t,trueTrace_data_t:地理系解算需要的数据')

lineWidth=2.5;
labelFontSize = 16;
axesFontsize = 13;

ph_pos_accuracy = figure('name',[pro_name,'_pos_accuracy']);
plot(pos_accuracy)
ylabel('m')
title('pos\_accuracy (m)')

ph_vel_accuracy = figure('name',[pro_name,'_vel_accuracy']);
plot(vel_accuracy)
ylabel('m/s')
title('vel\_accuracy (m/s)')

ph1 = figure('name',[pro_name,'_position_xyz_w']);
plot(runTime_IMU_sync,position_w')
title('position\_xyz\_w (m)')
legend('x','y','z');
xlabel('sec')
ylabel('m')

ph2 = figure('name',[pro_name,'_trace_xy']);
plot(position_w(1,:),position_w(2,:))
title('trace\_xy (m)')
ylabel('y/N')
xlabel('x/E')
hold on
plot(position_w(1,1),position_w(2,1),'o')


figure('name',[pro_name,'_trace_xy_lon_lat']);
set(cla,'fontsize',15)
plot(lon_lat_alt(1,:)*180/pi,lon_lat_alt(2,:)*180/pi)
% title('trace\_xy (rad)')
ylabel('latitude(°)','fontsize',20)
xlabel('longitude(°)','fontsize',20)
hold on
plot(lon_lat_alt(1,1)*180/pi,lon_lat_alt(2,1)*180/pi,'o')
saveas(gcf,[prodir,'\',[pro_name,'_trace_xy_lon_lat'],'.fig'])

ah = figure('name',[pro_name,'_attitude_w']);
plot(runTime_IMU_sync,attitude_w')
title('attitude\_w (rad)')
legend('pitch','roll','yaw');
xlabel('sec')
ylabel('rad')

saveas(ph1,[prodir,'\',[pro_name,'_position_xyz_w'],'.fig'])
saveas(ph2,[prodir,'\',[pro_name,'_trace_xy'],'.fig'])
saveas(ah,[prodir,'\',[pro_name,'_attitude_w'],'.fig'])
saveas(ph_pos_accuracy,[prodir,'\',[pro_name,'_pos_accuracy'],'.fig'])
saveas(ph_vel_accuracy,[prodir,'\',[pro_name,'_vel_accuracy'],'.fig'])
saveas(ph_pos_accuracy,[prodir,'\',[pro_name,'_pos_accuracy'],'.fig'])
saveas(ph_vel_accuracy,[prodir,'\',[pro_name,'_vel_accuracy'],'.fig'])

disp('getTrueTrace_kitti ok')

function [imuInputData,trueTrace,IMU_data_t,trueTrace_data_t,pos_accuracy,vel_accuracy] = oxts_to_imuInputData_trueTrace(oxts_extract_new)

%% 把 oxts_extract_new 转换为 trueTrace 格式 imuInputData 格式

ax = get_oxt_part(oxts_extract_new,12);
ay = get_oxt_part(oxts_extract_new,13);
az = get_oxt_part(oxts_extract_new,14);
af = get_oxt_part(oxts_extract_new,15);
al = get_oxt_part(oxts_extract_new,16);
au = get_oxt_part(oxts_extract_new,17);
wx = get_oxt_part(oxts_extract_new,18);
wy = get_oxt_part(oxts_extract_new,19);
wz = get_oxt_part(oxts_extract_new,20);
wf = get_oxt_part(oxts_extract_new,21);
wl = get_oxt_part(oxts_extract_new,22);
wu = get_oxt_part(oxts_extract_new,23);

% kitti 本体系与我的本体系定义区别
fib = [-ay;ax;az];      
wib = [-wy;wx;wz];

imuInputData.dataSource = 'kitti';
imuInputData.flag = 'exp';
imuInputData.f = fib ;
imuInputData.wib = wib ;

lat = get_oxt_part(oxts_extract_new,1) * pi/180; % 纬度
lon = get_oxt_part(oxts_extract_new,2) * pi/180;
alt = get_oxt_part(oxts_extract_new,3);
pos_accuracy = get_oxt_part(oxts_extract_new,24);
vel_accuracy = get_oxt_part(oxts_extract_new,25);
roll_kitti = get_oxt_part(oxts_extract_new,4);
pitch_kitti = get_oxt_part(oxts_extract_new,5);
yaw_kitti = get_oxt_part(oxts_extract_new,6);
%%%%%%%%%  从 kitti 的姿态定义 -> 我的姿态定义
yaw = yaw_kitti-pi/2 ;
yaw = yawHandle(yaw) ;  % 将 航向角 转换到 -180~180 
pitch = roll_kitti;
roll = pitch_kitti;

vn = get_oxt_part(oxts_extract_new,7);
ve = get_oxt_part(oxts_extract_new,8);
vf = get_oxt_part(oxts_extract_new,9);
vl = get_oxt_part(oxts_extract_new,10);
vu = get_oxt_part(oxts_extract_new,11);
velocity_t = [ve;vn;vu] ;

% attitude_t ： 本体系相对 东北天 的姿态，定义采用我的常用定义
attitude_t = [pitch;roll;yaw];
% kitti 自带的 经纬度->米 转换函数（不考虑地理系的变动）
[ position_w_kitti,attitude_w_kitti,position_b1 ] = oxts_to_posW(oxts_extract_new,attitude_t);
% 我的
lon_lat_alt = [lon;lat;alt]   ;
position_w_me = lon_lat_alt_to_Wxyz(lon_lat_alt,'e') ;
attitude_w_me = attitude_t_to_attitude_w(attitude_t,lon_lat_alt) ;

position_w = position_w_me ;
attitude_w = attitude_w_me ;

velocity_w = velocity_t_to_velocity_w(velocity_t,lon_lat_alt) ;
velocity_b = velocity_t_to_velocity_b(velocity_t,attitude_t) ;              %%%%%% ?????????? 为什么 bz的速度那么大，by的速度那么小（这个还可以理解）

initialAttitude_r = attitude_t(:,1);

trueTrace.dataSource = 'kitti';
trueTrace.planet = 'e';
trueTrace.initialPosition_e = [lon(1),lat(1),alt(1)];
trueTrace.initialPosition_r = zeros(3,1);
trueTrace.initialVelocity_r = velocity_w(:,1);
trueTrace.initialAttitude_r = initialAttitude_r ;

trueTrace.lon_lat_alt = lon_lat_alt;    % 经纬高度
trueTrace.position = position_w;        % 世界系（初始时刻地理系）位置
trueTrace.attitude_t = attitude_t ;
trueTrace.attitude = attitude_w ;
trueTrace.velocity = velocity_w ;
trueTrace.velocity_b = velocity_b ;

IMU_data_t.fib = fib ;
IMU_data_t.wib = wib ;
trueTrace_data_t.lon_lat_alt = lon_lat_alt ;
trueTrace_data_t.attitude_t = attitude_t ;
trueTrace_data_t.velocity_t = velocity_t ;

%% IMU 数据减少一个
function imuInputData=IMU_sub1(imuInputData)
N = length(imuInputData.f);
imuInputData.f(:,N) = [];
imuInputData.wib(:,N) = [];
imuInputData.runTime(N) = [];

function [ position_w,attitude_w,position_b1 ] = oxts_to_posW(oxts_extract_new,attitude_t)
pose_extract_new = convertOxtsToPose(oxts_extract_new);
N = length(pose_extract_new);
% position_b1：相对于第一个时刻
%   x指向前，y指向左
position_b1 = zeros(3,N);   
attitude_w = zeros(3,N);
Cb1w = FCbn(attitude_t(:,1));
for k=1:N
    position_b1(:,k) = pose_extract_new{k}(1:3,4);
    Cb1_bk = pose_extract_new{k}(1:3,1:3);
    Cw_bk = Cb1_bk * Cb1w' ;
    opintions.headingScope = 180 ;
    attitude_w(:,k) = GetAttitude(Cw_bk,'rad',opintions);
end

position_w = Cb1w * position_b1 ;



function data_k = get_oxt_part(oxts,k)
oxts_length = length(oxts);
data_k = zeros(1,oxts_length);
for i=1:oxts_length
    data_k(i) = oxts{i}(k);
end


%% 把年月日时分秒的时间转换为 从第一个时刻开始的 时间，单位：秒
% 只考虑 时分秒
function runTime_sec = ts_to_runTime(ts)
ts_length = length(ts);
runTime_sec = zeros(ts_length,1);

[ts_sec_1,ts_min_1,ts_h_1] = tsFormat_to_secFormat(ts{1}) ;

for k=1:ts_length
    
    [ts_sec_k,ts_min_k,ts_h_k] = tsFormat_to_secFormat(ts{k}) ;
    runTime_sec(k) = ts_sec_k-ts_sec_1 + (ts_min_k-ts_min_1)*60 + (ts_h_k-ts_h_1)*60*60 ;
end
disp('')

function [ts_sec_k,ts_min_k,ts_h_k] = tsFormat_to_secFormat(ts_k)
% 秒
ts_sec_k = ts_k(18:length(ts_k));
ts_sec_k = str2double(ts_sec_k) ;
% 分
ts_min_k = ts_k(15:16);
ts_min_k = str2double(ts_min_k) ;
% 时
ts_h_k = ts_k(12:13);
ts_h_k = str2double(ts_h_k) ;

function velocity = postion_to_velocity(postion,runTime)
N = length(postion);
velocity = zeros(size(postion));
for k=1:N
    if k==1
        k1 = 1 ;
        k2 = 2 ;
    elseif k==N
        k1 = N-1 ;
        k2 = N ;
    else
        k1 = k-1 ;
        k2 = k+1 ;
    end
    velocity(:,k) = (postion(:,k2)-postion(:,k1))/(runTime(k2)-runTime(k1));
end
