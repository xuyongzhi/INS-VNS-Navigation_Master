%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.2
% 作者：xyz
% 功能：视觉系统输入参数生成：由相机图像生成特征点
%   输出结果存储在结构体 visualInputData 中
% 输入：projectConfiguration（方案配置参数），选择实验还是仿真
%   仿真时输出的特征点已经包含噪声
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function visualInputData = GetVisualInputData(projectConfiguration,trueTrace,calibData)
%  实验提取时不需输入trueTrace
%% visualInputData
% [1*N ] cell 数组， N为时刻数，每个cell为一个时刻前后4副图的匹配成功特征点，1个cell visualInputData{i}. 中包含4个成员：
% leftLocCurrent：左相机当前图匹配成功的特征点，[2*n]，n为该帧前后4副图匹配成功的特征点个数。
% rightLocCurrent：右相机当前图匹配成功的特征点
% leftLocNext：左相机下时刻图匹配成功的特征点
% rightLocNext：右相机下时刻图匹配成功的特征点
% matchedNum ：匹配成功特征点个数 double
% aveFeatureNum ：该时刻前后4副图特征点个数的平均值（未做匹配时） double
if ~exist('projectConfiguration','var')
    projectConfiguration = importdata('projectConfiguration.mat');
    trueTrace = importdata('trueTrace.mat');
end

format long
if strcmp( projectConfiguration.visualDataSource,'e'  )
    % 实验获取 
  %  dbstop in siftDemoV4_main at 446
    visualInputData = siftDemoV4_main();
    if ~isempty(calibData)
        visualInputData.calibData=calibData;
    end
   %  [visualInputData] = GetSimuVisualData_dot( trueTrace ) ;
else
    % 仿真获取:必须给定真实地轨迹和姿态   
    visualInputData = GetSimuVisualData_RT( trueTrace );
    
end

%% 生成视觉RT仿真信息
function visualInputData = GetSimuVisualData_RT( trueTrace )
format long
% 从真实位置和姿态中计算真实的RT，然后再叠加噪声
position = trueTrace.position ;
attitude = trueTrace.attitude ;
trueFre = trueTrace.frequency;
 
button = questdlg('视觉Rbb和Tbb噪声的产生','提取现成 或 重新生成','重新生成','提取现成','重新生成') ;
if strcmp(button,'重新生成')
    %% 生成真实 Rbb和Tbb 生成Rbb和Tbb的噪声
    answer = inputdlg('视觉信息频率');
    visualFre = str2double(answer);
    visualNum = fix( (length(position)-1)*visualFre/trueFre)+1; % 视觉位置姿态的个数
    RTNum = visualNum-1;    % Rbb Tbb 的个数比 visualNum 少1
    Rbb = zeros(3,3,RTNum);
    Tbb = zeros(3,RTNum);
    % 根据真实轨迹生成真实 Tbb Rbb
    for k=1:RTNum
        k_true_last = 1+fix((k-1)*trueFre/visualFre) ;
        k_true = 1+fix((k)*trueFre/visualFre) ;
 
        Tbb(:,k) = FCbn(attitude(:,k_true))' * ( position(:,k_true)-position(:,k_true_last) ) ;
        Rbb(:,:,k) =  FCbn(attitude(:,k_true))' * FCbn(attitude(:,k_true_last)) ;     % R:b(k)->b(k+1)
        
    end
    trueTbb = Tbb ;
    trueRbb = Rbb;
    prompt = {'TbbErrorMean:  (m)                                    .','TbbErrorStd: (m)       .','AngleErrorMean: (rad)   ','AngleErrorStd: (rad)  ','RT误差标识' };
    num_lines = 1;
    def = {'[ 1 1 1 ]* 2e-3 *0','[ 1 1 1 ]* 1e-2','[ 1 1 1 ]* 2e-5 *0','[ 1 1 1 ]* 1e-4',['-T随机_负2-R随机_负4-',num2str(visualFre),'HZ'] };
    %def = {'[ 1 1 1]*0.0001 * 0','[1 1 1]*0.001 * 20','[ 1 1 1]*0.00001 * 0','[1 1 1 ]*0.0001 * 20'};
    answer = inputdlg(prompt,'设置RT噪声',num_lines,def);
    TbbErrorMean = eval(answer{1});   
    TbbErrorStd = eval(answer{2});
    AngleErrorMean = eval(answer{3});
    AngleErrorStd = eval(answer{4});
    Tbb_error = [   normrnd(TbbErrorMean(1),TbbErrorStd(1),1,RTNum);
                    normrnd(TbbErrorMean(2),TbbErrorStd(2),1,RTNum);
                    normrnd(TbbErrorMean(3),TbbErrorStd(3),1,RTNum)   ];
    Angle_error = [ normrnd(AngleErrorMean(1),AngleErrorStd(1),1,RTNum);
                    normrnd(AngleErrorMean(2),AngleErrorStd(2),1,RTNum);
                    normrnd(AngleErrorMean(3),AngleErrorStd(3),1,RTNum)   ];
                
	TbbErrorMean = mean(Tbb_error,2);
    TbbErrorStd = std(Tbb_error,0,2);
    AngleErrorMean = mean(Angle_error,2);
    AngleErrorStd = std(Angle_error,0,2);
    
	Tbb = Tbb+Tbb_error ;
    for k=1:RTNum
        Rbb(:,:,k) = FCbn(Angle_error(:,k)) * Rbb(:,:,k) ;
    end

    VisualRT.Rbb = Rbb ;
    VisualRT.Tbb = Tbb ;
    visualInputData.VisualRT = VisualRT ;
    visualInputData.frequency = visualFre;
    visualInputData.Angle_error = Angle_error ;
    visualInputData.Tbb_error = Tbb_error ;

    RTError.TbbErrorMean = TbbErrorMean ;
    RTError.TbbErrorStd = TbbErrorStd ;
    RTError.AngleErrorMean = AngleErrorMean ;
    RTError.AngleErrorStd = AngleErrorStd ;
    visualInputData.RTError = RTError ;
    
	save( ['visualInputData',answer{5}],'visualInputData');

else
    %% 提取现成的噪声
    [visualInputData_FileName,visualInputData_PathName] = uigetfile('*.mat','选择visualInputData文件');
	visualInputData = importdata([visualInputData_PathName,visualInputData_FileName]);
    % 只从 visualInputData 中提取频率和误差
    visualFre = visualInputData.frequency ;
    Angle_error = visualInputData.Angle_error ;
    Tbb_error = visualInputData.Tbb_error ;
    
    RTNum = fix( (length(position)-1)*visualFre/trueFre);
    Rbb = zeros(3,3,RTNum);
    Tbb = zeros(3,RTNum);
    % 根据真实轨迹生成真实 Tbb Rbb
    for k=1:RTNum
        k_true_last = 1+fix((k-1)*trueFre/visualFre) ;
        k_true = 1+fix((k)*trueFre/visualFre) ;
 
        Tbb(:,k) = FCbn(attitude(:,k_true))' * ( position(:,k_true)-position(:,k_true_last) ) ;
        Rbb(:,:,k) =  FCbn(attitude(:,k_true))' * FCbn(attitude(:,k_true_last)) ;     % R:b(k)->b(k+1)
    end
    
    Tbb = Tbb+Tbb_error ;
    for k=1:RTNum
        Rbb(:,:,k) = FCbn(Angle_error(:,k)) * Rbb(:,:,k) ;
    end
    % 更新 visualInputData 中的 Rbb  Tbb
    VisualRT.Rbb = Rbb ;
    VisualRT.Tbb = Tbb ;
    visualInputData.VisualRT = VisualRT ;
end 


function twoDimData = oneDim2TwoDim( oneDimData )
% 一维转成二维
format long
numTwoDim = fix(length(oneDimData)/2) ;
twoDimData = zeros(numTwoDim,2);
for i=1:numTwoDim
    twoDimData(i,1) = oneDimData(2*i-1);
    twoDimData(i,2) = oneDimData(2*i);
end
