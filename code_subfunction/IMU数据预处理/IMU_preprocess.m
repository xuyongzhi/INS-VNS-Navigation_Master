%************************************************************************/
%*                                                                      */
%*                    < IMU数据补偿 处理模块 >                           */
%*                                                                      */
%*                        原版本作者：杨胜                               */
%*                        改版作者：康泰钟                               */
%*						  编制日期: 2009-11           					 */
%************************************************************************/

%*************************  程序说明  ***********************************/
%   程序功能：
%       光纤IMU数据补偿。输入原始IMU数据，经过该程序处理后，输出三轴角速度和三轴加速度
%
%   程序输入输出及单位规定：
%       输入三轴加速度计的脉冲信号和三轴陀螺的脉冲信号（单位：^——即脉冲数）
%       输出相对于IMU基准面的三个正交轴x,y,z轴的角速度和加速度（角速度：o/h； 加速度：g）
%
%
%
%   程序说明：
%             1）该程序中包含光纤98系统标定结果
%             2）标定结果中包含以下信息：
%                   % 1.加计
%                      % 加计偏置
%                      % 加计标度因数
%                      % 加计安装误差
%                   % 2.陀螺
%                      % 陀螺常值漂移
%                      % 陀螺标度因数
%                      % 陀螺安装误差
% 
%   使用说明： 
%
%   程序中的符号说明：
%
%
%   程序中的问题：
%
%
%
%*************************  程序说明  ***********************************/

function compensition()
clc
clear all
close all
disp('未进行温度补偿');
format long;

% 0. 程序功能设定
    % 0.1 零偏校正（利用双位置对准结果）
CorrectBias = 0;
if CorrectBias
    bias_gyro = [0.011691, 0.056850, -0.206671]'; % 单位：度/小时
    bias_acc = [0.000285, -0.000135, 0.000266]'; % 单位：g
else
    bias_gyro = [0, 0, 0]'; % 单位：度/小时
    bias_acc = [0, 0, 0]'; % 单位：g
end

% 1. 待补偿数据载入
[fname,dirpath] = uigetfile('*.txt');
sourcename=[dirpath fname]
imu_data = load(sourcename);

n = length(imu_data(:,1))
% 2. 参数设定
    % 数据采集频率
FRE=100;

% 3. 加计参数(按标定结果填入)

    % 标定参数数据载入
    
%     IMU_Param = load('F:\A1系统\2011-01-20第一套激光POS全温标定\2011年1月19日（20°）\CaliResult\CaliRlt_and_SelfChk.dat');
    
%         IMU_Param = load('F:\A2系统\2011-02-23_26-全温标定\20度\CaliResult\CaliRlt_and_SelfChk.dat');
% IMU_Param = load('D:\标定参数\2011-02-23_26，A2全温标定结果\20\CaliResult\CaliRlt_and_SelfChk.dat');
%     IMU_Param = load('F:\A1系统\2011-04-23_26全温标定（A1系统重新更换陀螺后）\20度\CaliResult\CaliRlt_and_SelfChk.dat');
%     IMU_Param = load('F:\A3系统\2011-02-28-自研激光A3-全温标定\20度\CaliResult\CaliRlt_and_SelfChk.dat');
    
IMU_Param = load('E:\惯性视觉导航\综合程序\综合程序\code_subfunction\IMU数据预处理\CaliResult20\CaliRlt_and_SelfChk.dat');

    
    % 三个加计的零偏，单位：g(按标定结果填入)
    K0x =  IMU_Param(1,1);
	K0y =  IMU_Param(1,2);
	K0z =  IMU_Param(1,3);
    
    % 三个加计的标度因数的倒数，单位：1/(^/s/g) (按标定结果填入)
    K1x_ = IMU_Param(2,1);
	K1y_ = IMU_Param(2,2);
	K1z_ = IMU_Param(2,3);
    
    % 三个加计的安装误差阵(按标定结果填入)
    K = IMU_Param(3:5,:);
	K_ = inv(K);
    
% 4. 陀螺参数(按标定结果填入)

    % 三个陀螺的零偏,单位: o/h
	E0x = IMU_Param(6,1);
	E0y = IMU_Param(6,2);
	E0z = IMU_Param(6,3);

    % 陀螺标度因数的倒数,单位: "/^
	E1x_ = IMU_Param(7,1);
	E1y_ = IMU_Param(7,2);
	E1z_ = IMU_Param(7,3);
    
    % 陀螺安装误差(按标定结果填入)
    E  = IMU_Param(8:10,:);
	E_ = inv(E);
    
    % 与g有关项阵,单位: o/h/g
	D = IMU_Param(11:13,:);

% 5. 为补偿结果分配存储空间
fbx = zeros(1,n);
fby = zeros(1,n);
fbz = zeros(1,n);
gx = zeros(1,n);
gy = zeros(1,n);
gz = zeros(1,n);
index = zeros(1,n);
PCSTime = zeros(1,n);       % IPS发送的时间
ReadTime = zeros(1,n);      % 计算机读取的时间
InferReadTime = zeros(1,n); % 推测的计算机读取时间：不丢包的前提，
                                %认为第1次读取时间精确等于第一次采集时间，然后按照100HZ频率往后推测读取时间

% 6. 补偿加速度和角速度
for i=1 : n
    % 补偿得到载体系三个方向的加速度
        % 补偿得到三个加速度计方向的加速度
    tmpa=[  imu_data(i,6)*FRE*K1x_ - K0x;
            imu_data(i,7)*FRE*K1y_ - K0y;
            imu_data(i,8)*FRE*K1z_ - K0z;
         ];
        % 补偿得到相对IMU基准面的三轴正交的加速度
    tempa = K_*tmpa;
    fbx(i)=tempa(1) - bias_acc(1);
    fby(i)=tempa(2) - bias_acc(2);
    fbz(i)=tempa(3) - bias_acc(3);
    
    % 补偿得到载体系三个方向的角速度
        % 计算得到与g有关项误差
    tmpa_w = D*tempa;
        % 补偿得到三个陀螺方向的角速度，单位：o/h（备注: "/s = o/h）
    tmpw=[  imu_data(i,3)*FRE*E1x_-E0x; 
            imu_data(i,4)*FRE*E1y_-E0y;
            imu_data(i,5)*FRE*E1z_-E0z;
         ];
        % 补偿得到相对IMU基准面的三轴正交的角速度
    tempw = E_*tmpw;
    gx(i) = tempw(1) - bias_gyro(1);
    gy(i) = tempw(2) - bias_gyro(2);
    gz(i) = tempw(3) - bias_gyro(3);
    
    % 记录每组数据的标号和时间点信息
    index(i) = imu_data(i,1);
%     if i>1
%         if imu_data(i,2)-imu_data(i-1,2)>1009 &imu_data(i,2)-imu_data(i-1,2)<1011%
%             imu_data(i,2)=imu_data(i,2)-1000;
%         end
%     end
    PCSTime(i) = imu_data(i,2)-imu_data(1,2);
    ReadTime(i) = imu_data(i,9)-imu_data(1,9);
end 

% 7. 将补偿结果存入文件

% clear imu_data;
% 
% fid = fopen(strcat(sourcename(1:(length(sourcename)-4)), 'T35.dat'),'w');
% 
% for i=1:n
%     fprintf(fid,'%8d  %15.5f  %20.10f  %20.10f  %20.10f  %20.10f  %20.10f  %20.10f\n',index(i),PCSTime(i)-12.5,gx(i),gy(i),gz(i),fbx(i),fby(i),fbz(i));
% end;
% fclose(fid);

%% xyz 加
f = [fbx;fby;fbz];
wib = [gx;gy;gz];

fnameOnly = fname(1:(length(fname)-4));
newFilePath = [dirpath,fnameOnly];
if isdir(newFilePath)
    delete([newFilePath,'\*']);
else
   mkdir(newFilePath); 
end
% 计算 InferReadTime
InferReadTime(1) = ReadTime(1) ;
for i=2:length(ReadTime);
    InferReadTime(i) = InferReadTime(i-1)+10 ;  % 100HZ ms 单位
end
%% 

%% 丢包的处理
[PCSTime_new,PSC_ReadTime] = IPSTime_Correct(PCSTime) ;
% 显示原始数据
PlotOriginalFig(newFilePath,fbx,fby,fbz,gx,gy,gz,PCSTime,PCSTime_new,ReadTime,InferReadTime,PSC_ReadTime);
% 从 PSC_ReadTime 可得到丢包的序号
lossedOrder = GetLossedNum(PSC_ReadTime);
% 补充数据
fbx = CorrectLossedNum(fbx,lossedOrder);
fby = CorrectLossedNum(fby,lossedOrder);
fbz = CorrectLossedNum(fbz,lossedOrder);
gx = CorrectLossedNum(gx,lossedOrder);
gy = CorrectLossedNum(gy,lossedOrder);
gz = CorrectLossedNum(gz,lossedOrder);
% 显示丢包处理之后的数据
PlotBugCorrectedFig(newFilePath,fbx,fby,fbz,gx,gy,gz);

%% 去除异常噪声
save([newFilePath,'\imu_original.mat'],'f','wib','PCSTime');

fid = fopen([newFilePath,'\IMU数据的标准差.txt'],'w+');
str=sprintf('%0.4f  ',std(f,0,2)*1e6) ;
fprintf(fid,'\n原始-加计标准差:\t\t%s   ug\n',str);
str=sprintf('%0.4f  ',std(wib,0,2)) ;
fprintf(fid,'\n原始-加计标准差:\t\t%s   °/h\n',str);

isRejectUnusual = questdlg('是否剔除野值并平滑？','t','Yes','NO','Yes') ;
if strcmp(isRejectUnusual,'Yes')
 
    disp('正在去除野值并平滑')  
    %%　根据未平滑前的原始数据进行剔除野值，原始加计的标准差大约为：1000ug，原始陀螺的标准差大约为：10°/h
    % （1）3s 5min数据 30*6 s
    span  = 0.1;
    [smooth_f,real_f] = RejectUnusual_Smooth_dynamic( f,span,[3000;3000;3000]*1e-6 ) ; % 3s步长，50ug 误差限  
    [smooth_wib,real_wib] = RejectUnusual_Smooth_dynamic( wib,span,[30;30;30] ) ; % 30 °/h 误差限

    str=sprintf('%0.4f  ',std(real_f,0,2)*1e6) ;
    fprintf(fid,'\n剔除野值后-加计标准差:%s   ug\n',str);
    str=sprintf('%0.4f  ',std(real_wib,0,2)) ;
    fprintf(fid,'\n剔除野值后-加计标准差:%s   °/h\n',str);

    str=sprintf('%0.4f  ',std(smooth_f,0,2)*1e6) ;
    fprintf(fid,'\n剔除野值并平滑后-加计标准差:%s   ug\n',str);
    str=sprintf('%0.4f  ',std(smooth_wib,0,2)) ;
    fprintf(fid,'\n剔除野值并平滑后-加计标准差:%s   °/h\n',str);
    %%
    figure('name','剔除野值后-加计-x');
    plot(time,real_f(1,:));
    title('剔除野值后-加计-x');
    saveas(gcf,[newFilePath,'\剔除野值后-加计-x.fig']);

    figure('name','剔除野值后-加计-y');
    plot(time,real_f(2,:));
    title('剔除野值后-加计-y');
    saveas(gcf,[newFilePath,'\剔除野值后-加计-y.fig']);

    figure('name','剔除野值后-加计-z');
    plot(time,real_f(3,:));
    title('剔除野值后-加计-z');
    saveas(gcf,[newFilePath,'\剔除野值后-加计-z.fig']);

    figure('name','剔除野值后-陀螺-x');
    plot(time,real_wib(1,:));
    title('剔除野值后-陀螺-x');
    saveas(gcf,[newFilePath,'\剔除野值后-陀螺-x.fig']);

    figure('name','剔除野值后-陀螺-y');
    plot(time,real_wib(2,:));
    title('剔除野值后-陀螺-y');
    saveas(gcf,[newFilePath,'\剔除野值后-陀螺-y.fig']);

    figure('name','剔除野值后-陀螺-z');
    plot(time,real_wib(3,:));
    title('剔除野值后-陀螺-z');
    saveas(gcf,[newFilePath,'\剔除野值后-陀螺-z.fig']);
    %%%%%%%%%%%%%
    figure('name','剔除野值并平滑后-加计-x');
    plot(time,smooth_f(1,:));
    title('剔除野值并平滑后-加计-x');
    saveas(gcf,[newFilePath,'\剔除野值并平滑后-加计-x.fig']);

    figure('name','剔除野值并平滑后-加计-y');
    plot(time,smooth_f(2,:));
    title('剔除野值并平滑后-加计-y');
    saveas(gcf,[newFilePath,'\剔除野值并平滑后-加计-y.fig']);

    figure('name','剔除野值并平滑后-加计-z');
    plot(time,smooth_f(3,:));
    title('剔除野值并平滑后-加计-z');
    saveas(gcf,[newFilePath,'\剔除野值并平滑后-加计-z.fig']);

    figure('name','剔除野值并平滑后-陀螺-x');
    plot(time,smooth_wib(1,:));
    title('剔除野值并平滑后-陀螺-x');
    saveas(gcf,[newFilePath,'\剔除野值并平滑后-陀螺-x.fig']);

    figure('name','剔除野值并平滑后-陀螺-y');
    plot(time,smooth_wib(2,:));
    title('剔除野值并平滑后-陀螺-y');
    saveas(gcf,[newFilePath,'\剔除野值并平滑后-陀螺-y.fig']);

    figure('name','剔除野值并平滑后-陀螺-z');
    plot(time,smooth_wib(3,:));
    title('剔除野值并平滑后-陀螺-z');
    saveas(gcf,[newFilePath,'\剔除野值并平滑后-陀螺-z.fig']);
    
    f = smooth_f;
    wib = smooth_wib ;
end
%%
earth_const = getEarthConst();

imuInputData.f = f * (-earth_const.g0);  % 三维输出的数据都是以 -9.8 为单位的，乘以 -9.8 后得到的数据才是按 IMU 坐标系的加速度（东北天）
imuInputData.wib = wib * pi/180/3600;
imuInputData.computerTime = PCSTime;

assignin('base','imuInputData',imuInputData);

save([newFilePath,'\imuInputData.mat'],'imuInputData');

disp('数据保存OK');
fclose(fid);

function [PCSTime_new,PSC_ReadTime] = IPSTime_Correct(PCSTime)
%% 修正IPSTime 
% 直接接收的 PCSTime 的bug：经常只接收到低位，没有高位
% 检查出这种情况并修正：小于前一时刻的时间（认为第一个没有出错）
% 另外，检查这段时间是否有丢包

disp('正在寻找 PCSTime 的bug区间，并修正')

total = length(PCSTime);
bugSectionNum = 0;     % 记录出错的区段数
bugTotalNum = 0 ;       %记录出错的个数
frontK = 0;  % 当前bug段的前一个正常IPS时间 序号
backK = 0;   % 当前bug段的后一个正常IPS时间 序号
flag = 'seekFront'; % 标记正在寻找bug区的头还是尾‘seekFront'/'seekBack’

for k=2:total
    if strcmp(flag,'seekFront')
        % 正在寻找头：变小
        if(PCSTime(k)<PCSTime(k-1))
            % 还需排除正常的翻转
            if (PCSTime(k-1)-PCSTime(k))>3.3e5 && (PCSTime(k-1)-PCSTime(k))<3.5e5
               % 判断为正常的翻转 
               display(sprintf('翻转一次,k=%d',k))
            else
                frontK = k-1 ; %找到头
                flag = 'seekBack';
            end            
        end
    else
        if(PCSTime(k)>PCSTime(frontK))
            backK = k ;  % 找到尾
            flag = 'seekFront';
            bugSectionNum=bugSectionNum+1;display(sprintf('第%d段bug区间:%d - %d',bugSectionNum,frontK+1,backK-1))
            %%%%%%%%% 检查这段有无丢包
            bugNum = backK-frontK-1 ;   % 此段bug个数
            bugTotalNum = bugTotalNum+bugNum ;
            bugTime = PCSTime(backK)-PCSTime(frontK)-10;    % 此段bug时间
            if (bugTime-5)>bugNum*10
                % 此段有丢包
                 display(sprintf('\t此段有丢包，bugNum=%d,bugTime=%dms',bugNum,bugTime));
            else
                display(sprintf('\t此段无丢包，bugNum=%d,bugTime=%dms',bugNum,bugTime));
            end
            %%%%%%%%% 修正这段区间
            for j=(frontK+1):(backK-1)
                PCSTime(j) = PCSTime(j-1)+10 ;
            end
        end
    end
    
end
display(sprintf('IPSTime修正结束,共%d段，共%d个',bugSectionNum,bugTotalNum))
PCSTime_new = PCSTime ;

PSC_ReadTime = zeros(size(PCSTime));
PSC_ReadTime(1) = PCSTime(1) ;
base = 0;
for k=2:length(PCSTime)
    if PCSTime(k)<PCSTime(k-1)
       % 翻转
       base = PCSTime(k-1)-PCSTime(k)+10 ;
    end
    PSC_ReadTime(k) = PCSTime(k)+base ;    
end

function lossedOrder = GetLossedNum(PSC_ReadTime)
%% 查找丢包的情况
readNum = length(PSC_ReadTime);
realNum = fix((PSC_ReadTime(length(PSC_ReadTime))-PSC_ReadTime(1)+5)/10) ;
lossedTotalNum1 = realNum-readNum ;
display(sprintf('丢包个数：%d',lossedTotalNum1));
% 细致查找丢包区间/个数
lossedTotalNum2 = 0;
lossN = 0;  % 第 次丢包
lossedOrder = zeros(10,2);    % 存储丢包信息，lossOrder(:,1)存储丢包前一时刻的序号，lossOrder(:,2)存储
for k=2:length(PSC_ReadTime)
    dtime = PSC_ReadTime(k)-PSC_ReadTime(k-1) ;
    if dtime>10+5
       % 丢包
       lossN = lossN+1 ;
       lossNum = fix((dtime+5)/10)-1 ;    % 丢的个数
       lossedTotalNum2 = lossedTotalNum2+lossNum ;
       lossedOrder(lossN,1) = k-1 ;
       lossedOrder(lossN,2) = lossedTotalNum2 ;
    end
end
lossedOrder = lossedOrder(1:lossN,:) ;
if lossedTotalNum2~=lossedTotalNum1
    str = sprintf('丢包个数未全部找到！ %d/%d',lossedTotalNum2,lossedTotalNum1);
   warndlg(str); 
else
    disp('丢包检查OK')
end

function newdata = CorrectLossedNum(data,lossedOrder)
%%　修补丢包数据
% lossedOrder：丢包信息，lossedOrder(:,1)之后开始丢包，lossedOrder(:,2)为被丢个数
% data 为一维列向量
% 方法：取前后若干时刻数据进行线性拟合，然后插值，补上被丢数据

%% 先计算好所有的拟合函数
P = zeros(size(lossedOrder,1),3) ; % 2次多项式拟合，3个参数
polyNum = 100*5 ;% 前后用于拟合的个数
span = 20 ; % 用于平滑的范围
for k=1:size(lossedOrder,1)
    orderStart = lossedOrder(k,1);  % 丢包前一个的序号
    % 取前后附近的数据用于拟合
    toPolyData = data(orderStart-polyNum:orderStart+polyNum+1); % 2*polyNum+2 个
    % 先做简单的平滑
    toPolyData = smooth(toPolyData,span,'rlowess');
    % 最小二乘拟合
    time = 1:length(toPolyData);
    time(polyNum+2 : length(time)) = time(polyNum+2 : length(time))+lossedOrder(k,2) ;
    P(k,:) = polyfit(time,toPolyData',2);     % 2次最小二乘拟合
end
%% 补充数据
lossedNum = sum(lossedOrder(:,2));
newdata = zeros(1,size(data,1)+lossedNum);  % 新数据的大小
newpitch = 1;
losspitch = 1;
for k=1:length(data)
    newdata(newpitch) = data(k) ;
    newpitch = newpitch+1;
    if k == lossedOrder(losspitch,1)
        % 到丢包处
        for j=1:lossedOrder(losspitch,2)
            newdata(newpitch) = P(losspitch,1)*(polyNum+1+j)^2 + P(losspitch,2)*(polyNum+1+j) + P(losspitch,3) ; % 拟合结果
            newpitch = newpitch+1;
        end
    end
end


function PlotOriginalFig(newFilePath,fbx,fby,fbz,gx,gy,gz,PCSTime,PCSTime_new,ReadTime,InferReadTime,PSC_ReadTime)
time = (1:length(fbx));

figure('name','原始-加计-x');
plot(time,fbx);
title('原始-加计-x');
saveas(gcf,[newFilePath,'\原始-加计-x.fig']);

figure('name','原始-加计-y');
plot(time,fby);
title('原始-加计-y');
saveas(gcf,[newFilePath,'\原始-加计-y.fig']);

figure('name','原始-加计-z');
plot(time,fbz);
title('原始-加计-z');
saveas(gcf,[newFilePath,'\原始-加计-z.fig']);

figure('name','原始-陀螺-x');
plot(time,gx);
title('原始-陀螺-x');
saveas(gcf,[newFilePath,'\原始-陀螺-x.fig']);

figure('name','原始-陀螺-y');
plot(time,gy);
title('原始-陀螺-y');
saveas(gcf,[newFilePath,'\原始-陀螺-y.fig']);

figure('name','原始-陀螺-z');
plot(time,gz);
title('原始-陀螺-z');
saveas(gcf,[newFilePath,'\原始-陀螺-z.fig']);

figure('name','PCS发送时间');
plot(time,[PCSTime;PCSTime_new]);
title('PCS发送时间');
legend({'原始IPSTime','修正后的IPSTime'});
saveas(gcf,[newFilePath,'\PCS发送时间.fig']);

figure('name','计算机读取时间');
plot(time,[ReadTime;InferReadTime;PSC_ReadTime]);
title('计算机读取时间');
legend({'记录的读取时间','推测的读取时间','PCS读取时间'});
saveas(gcf,[newFilePath,'\计算机读取时间.fig']);

function PlotBugCorrectedFig(newFilePath,fbx,fby,fbz,gx,gy,gz)
time = (1:length(fbx));

figure('name','补充丢包后-加计-x');
plot(time,fbx);
title('补充丢包后-加计-x');
saveas(gcf,[newFilePath,'\补充丢包后-加计-x.fig']);

figure('name','补充丢包后-加计-y');
plot(time,fby);
title('补充丢包后-加计-y');
saveas(gcf,[newFilePath,'\补充丢包后-加计-y.fig']);

figure('name','补充丢包后-加计-z');
plot(time,fbz);
title('补充丢包后-加计-z');
saveas(gcf,[newFilePath,'\补充丢包后-加计-z.fig']);

figure('name','补充丢包后-陀螺-x');
plot(time,gx);
title('补充丢包后-陀螺-x');
saveas(gcf,[newFilePath,'\补充丢包后-陀螺-x.fig']);

figure('name','补充丢包后-陀螺-y');
plot(time,gy);
title('补充丢包后-陀螺-y');
saveas(gcf,[newFilePath,'\补充丢包后-陀螺-y.fig']);

figure('name','补充丢包后-陀螺-z');
plot(time,gz);
title('补充丢包后-陀螺-z');
saveas(gcf,[newFilePath,'\补充丢包后-陀螺-z.fig']);
