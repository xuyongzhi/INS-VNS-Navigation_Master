% buaa xyz 2014.1.14

% 从 imuInputData 中得到 imu 常值和随机漂移

% 输入：imuInputData（格式见综合程序说明文档）
% 输出：
%       pa:加计常值漂移
%       na:加计随机漂移
%       pg:陀螺常值漂移
%       ng:陀螺随机漂移

function [pa,na,pg,ng,imuInputData,gp] = GetIMUdrift( imuInputData,planet )
% 仿真时噪声已知，存储在imuInputData中，实验时噪声未知，手动输入 常值偏置 和 随机标准差
format long
if strcmp(planet,'m')
    moonConst = getMoonConst;   % 得到月球常数
    gp = moonConst.g0 ;     % 用于导航解算
else
    earthConst = getEarthConst;   % 得到地球常数
    gp = earthConst.g0 ;     % 用于导航解算
end
if ~isfield(imuInputData,'flag')    
   if  isfield(imuInputData,'f_noise')  
       imuInputData.flag = 'sim';
   else
       imuInputData.flag = 'exp';
   end
end
if strcmp(imuInputData.flag,'sim')
    disp('仿真：imuInputData中包含噪声成员，P和Q阵初值由f_noise和wib_noise确定') 
    if ~isfield(imuInputData,'pg')  % 还未计算噪声参数
        % 常值噪声
        imuInputData.pa = mean(imuInputData.f_noise,2);      % imuInputData.f_noise [3*n]
        imuInputData.pg = mean(imuInputData.wib_noise,2);
        % 随机标准差
        imuInputData.na = std(imuInputData.f_noise,0,2);
        imuInputData.ng = std(imuInputData.wib_nois,0,2);
    end
    
else
    disp('实验：imuInputData中不包含噪声成员，P和Q阵初值由手动输入的噪声经验值确定') 
    if strcmp(planet,'m')
        dlg_title = '月面-IMU经验常值漂移大小';
    else
        dlg_title = '地面-IMU经验常值漂移大小';
    end

    prompt = {'加计常值噪声:  (ug)   ','加计随机噪声标准差: (ug)       .','陀螺常值噪声: (°/h)   ','陀螺随机噪声标准差: (°/h)  '};
    num_lines = 1;
   % def = {'50 50 50','30 30 30','5 5 5','3 3 3'};
    def = {'200 200 200','100 100 100','7 7 7','6 6 6'};
    % def = {'10 10 10','10 10 10','0.1 0.1 0.1','0.1 0.1 0.1'};
    %def = {'1','1','0.01','0.01'};
    answer = inputdlg(prompt,dlg_title,num_lines,def);
    constNoise_f = sscanf(answer{1},'%f')*1e-6*gp ;   % 加计常值偏置
    sigmaNoise_f = sscanf(answer{2},'%f')*1e-6*gp ;   % 加计噪声的标准差
    constNoise_wib = sscanf(answer{3},'%f')*pi/180/3600 ; % 陀螺常值偏置
    sigmaNoise_wib = sscanf(answer{4},'%f')*pi/180/3600 ; % 陀螺噪声的标准差

    imuInputData.pa = constNoise_f ;
    imuInputData.na = sigmaNoise_f ;
    imuInputData.pg = constNoise_wib ;
    imuInputData.ng = sigmaNoise_wib ;
end

% 常值噪声
pg = imuInputData.pg ;
pa = imuInputData.pa ;
% 随机标准差
ng = imuInputData.ng ;
na = imuInputData.na ;

