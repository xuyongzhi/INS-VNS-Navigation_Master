% buaa xyz 2014.20

% 查看结果
global projectDataPath
if isempty(projectDataPath)
    projectDataPath = [pwd,'\data'];
end

%projectDataPath = 'E:\惯性视觉导航\综合程序\综合程序\data\组合算法测试\仿真生成RT - 匀速圆周180m\navResult';
% oldFloder = cd([pwd,'\code_subfunction\ResultDisplay']) ; % 进入结果查看路径
%uiwait(ResultDisplay());
addpath([pwd,'\code_subfunction\ResultDisplay'])
ResultDisplay()