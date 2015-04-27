%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.3
% 作者：xyz
% 功能：IMU仿真数据生成：
%   输入轨迹发生器的理想IMU数据，叠加噪声，输出含噪声的IMU数据
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

function imuInputData = GetIMUInputData(projectConfiguration,trueTrace)
% 实验时不需输入 trueTrace
if(strcmp(projectConfiguration.imuDataSource,'e'))  % 实验时直接读取IMU数据文件
    imuInputData = GetExpIMUData();
    disp('提取地面实验IMU数据完毕');
else    % 仿真时在轨迹发生器数据的基础上叠加噪声得到IMU数据
    if( isfield(trueTrace,'f_IMU')&&isfield(trueTrace,'wib_IMU') )
        disp('trueTrace中包含理想的IMU数据，在其基础上叠加噪声得到仿真IMU数据')
        imuInputData = AddIMUNoise(trueTrace);  
    else
        disp('需要生成仿真IMU数据，但trueTrace中不包含理想的IMU数据，调用轨迹发生器生成')
    end
    imuInputData.flag = 'sim';
end
save([pwd,'\imuInputData.mat'],'imuInputData')

function imuInputData = GetExpIMUData()
%% 从IMU数据采集软件直接输出的IMU数据到设定格式的 imuInputData
% 得到输入数据的路径 inputDataPath
global projectDataPath
if isempty(projectDataPath) % 独立运行此函数时
    inputDataPath = pwd; 
else
    inputDataPath = [projectDataPath,'\直接采集的数据'];   % 默认图像和IMU数据保存的文件夹
    if ~isdir(inputDataPath)
        inputDataPath = projectDataPath;
    end
end
% 先提取软件采集的IMU数据, txt 格式：要求删除第一行的字符说明
[FileName,PathName] = uigetfile('*.txt','选择IMU采集数据（要求先手动删掉第一行字符说明）',[inputDataPath,'\IMUdata.txt']);
imuData = dlmread([PathName,FileName]);
if imuData(numel(imuData))==0 
    row = size(imuData,1) ;
    imuData = imuData(1:(row-1),:); % 最后一行可能无效删除掉    
end
f = imuData(:,5:7);
f = f'; % 得到加计数据 [3*N]
wib = imuData(:,2:4);
wib = wib';% 得到陀螺数据 [3*N]
%% 输出imuInputData
imuInputData = [];
earth_const = getEarthConst();
g = earth_const.g0 ;
imuInputData.f = f * (-g);  % IMU输出单位是 g=-9.8 (具体大小与当地一致)
imuInputData.wib = wib * pi/180;    % °/s转换为 rad/s
imuInputData.frequency = 100;
imuInputData.flag = 'exp';

function imuInputData = AddIMUNoise(trueTrace)
%%　为理想IMU数据叠加噪声
f_true = trueTrace.f_IMU ;
wib_true = trueTrace.wib_IMU ;
planet = trueTrace.planet;
if strcmp(planet,'m')
    dlg_title = '月面-IMU仿真噪声';
    moonConst = getMoonConst;   % 得到月球常数
    gp = moonConst.g0 ;     % 用于导航解算
else
    dlg_title = '地面-IMU仿真噪声';
    earthConst = getEarthConst;   % 得到地球常数
    gp = earthConst.g0 ;     % 用于导航解算
end

prompt = {'加计常值噪声:  (ug)   ','加计随机噪声标准差: (ug)       .','陀螺常值噪声: (°/h)   ','陀螺随机噪声标准差: (°/h)  '};
num_lines = 1;
%def = {'10 10 10','10 10 10','0.1 0.1 0.1','0.1 0.1 0.1'};
def = {'10 10 10','5 5 5','0.1 0.1 0.1','0.05 0.05 0.05'};
%def = {'1','1','0.01','0.01'};
answer = inputdlg(prompt,dlg_title,num_lines,def);
constNoise_f = sscanf(answer{1},'%f')*1e-6*gp ;   % 加计常值偏置
sigmaNoise_f = sscanf(answer{2},'%f')*1e-6*gp ;   % 加计噪声的标准差
constNoise_wib = sscanf(answer{3},'%f')*pi/180/3600 ; % 陀螺常值偏置
sigmaNoise_wib = sscanf(answer{4},'%f')*pi/180/3600 ; % 陀螺噪声的标准差

%生成干扰信息:均值为constNoise_f，标准差为sigmaNoise_f
f_noise = zeros(size(f_true));
f_noise(1,:) = normrnd(constNoise_f(1),sigmaNoise_f(1),1,max(size(f_true))) ;
f_noise(2,:) = normrnd(constNoise_f(2),sigmaNoise_f(2),1,max(size(f_true))) ; 
f_noise(3,:) = normrnd(constNoise_f(3),sigmaNoise_f(3),1,max(size(f_true))) ; 

constNoise_f = mean(f_noise,2);
sigmaNoise_f = std(f_noise,0,2);

imuInputData.f = f_true + f_noise;
imuInputData.f_noise = f_noise ;
imuInputData.pa = constNoise_f;
imuInputData.na = sigmaNoise_f;

%生成干扰信息:均值为constNoise_wib，标准差为sigmaNoise_wib
wib_noise = zeros(size(wib_true));
wib_noise(1,:) = normrnd(constNoise_wib(1),sigmaNoise_wib(1),1,max(size(wib_true))) ;
wib_noise(2,:) = normrnd(constNoise_wib(2),sigmaNoise_wib(2),1,max(size(wib_true))) ; 
wib_noise(3,:) = normrnd(constNoise_wib(3),sigmaNoise_wib(3),1,max(size(wib_true))) ;

constNoise_wib = mean(wib_noise,2);
sigmaNoise_wib = std(wib_noise,0,2);

imuInputData.wib = wib_true + wib_noise;
imuInputData.wib_noise = wib_noise ;
imuInputData.pg = constNoise_wib;
imuInputData.ng = sigmaNoise_wib ;

imuInputData.frequency = trueTrace.frequency ;
%% 将常值漂移保存为绘图格式
timeNum = length(f_noise);
accDrift = repmat([constNoise_f(1);constNoise_f(2);constNoise_f(3)],1,timeNum);
gyroDrift = repmat([constNoise_wib(1);constNoise_wib(2);constNoise_wib(3)],1,timeNum);

resultNum = 2;
realDriftResult = cell(1,resultNum);

% 有4个共同的成员
for j=1:resultNum
    realDriftResult{j}.dataFlag = 'xyz result display format';
    realDriftResult{j}.frequency = imuInputData.frequency ;
    realDriftResult{j}.project = '真实常值漂移';
    realDriftResult{j}.subName = {'x','y','z'};
end
realDriftResult{1}.data = accDrift ;     
realDriftResult{1}.name = 'accDrift(m／s^2)';
realDriftResult{1}.comment = '加计常值漂移';

realDriftResult{2}.data = gyroDrift*180/pi*3600 ;     % rad/s 转化为 °/h
realDriftResult{2}.name = 'gyroDrift(°／h)';
realDriftResult{2}.comment = '陀螺常值漂移';

imuInputData.realDriftResult = realDriftResult;