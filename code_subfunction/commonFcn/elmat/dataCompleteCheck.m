%% 数据完整性检查

function data = dataCompleteCheck(dataStyle,data)

switch dataStyle
    case 'visualInputData'
        visualInputData = data ;
        if ~isfield(visualInputData,'frequency')
            answer = inputdlg('视觉信息频率');
            visualInputData.frequency = str2double(answer);
        end
        if ~isfield(visualInputData,'calibData')
            visualInputData.calibData = loadCalibData() ;
        end
        data = visualInputData ;
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