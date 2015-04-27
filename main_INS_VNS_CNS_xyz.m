%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.2
% 作者：xyz
% 功能：惯性/视觉/天文组合导航总接口程序：手动执行入口
%   此脚本程序为执行调用的接口程序，通过调用各种方法的子函数，实现组合导航的功能
%       要做到:（1）所有方法的执行均需只在此函数中运行一次。（2）所有的子程序才升级
%       不影响此调用函数。（3）方法的切换不需更改大量代码，只需设置一个参数或运行时读取
%   给出流程图，主/子程序中的代码分段与流程图中的流程分段一致。
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all
format long

oldfolder = cd([pwd,'\code_subfunction\commonFcn']);
add_CommonFcn_ToPath;
cd(oldfolder)
global  projectDataPath projectConfigurationPath   navResultPath	

%% 各种文件路径
% 所有的数据相关路径均在 以项目名区分的 文件夹下
if exist([pwd,'\projectConfiguration\defaultProjectDataPath.mat'],'file')
    defaultProjectDir = importdata([pwd,'\projectConfiguration\defaultProjectDataPath.mat']);
    if isempty(defaultProjectDir)
        defaultProjectDir = [pwd,'\data'];
    end
else
    defaultProjectDir = [pwd,'\data'];
end
projectDataPath = uigetdir(defaultProjectDir,'选择方案数据文件夹');  % 此方案的所有独有的输入输出数据均保存在此文件夹中
if projectDataPath==0   
    return; 
end
% 在projectPath下建立导航结果存储文件夹
navResultPath = [projectDataPath,'\navResult'];
if isdir(navResultPath)
   % button = questdlg('是否清空原结果','结果清空','否','是','否');
    button='否';
    if strcmp(button,'是')
        delete([navResultPath,'\*']);    % 清空结果文件夹
    end
else
    mkdir(navResultPath);
end

%% <1>选择导航方案
% 所有导航方案方法设置均存在方案参数结构体 projectConfiguration 中
    % projectConfiguration内成员：isUpdateVisualData=0/1,visualDataSource='e'\'s'
% 方案配置存储在“projectConfiguration”文件夹中，此处导入名为 defaultProject.mat 的文件
%% 导入默认方案设置
%    isNewPro = menu('新建方案设置还是选择现有？','新建','选择现有') ;
  isNewPro = 2;
if isNewPro==0
   return; 
end
if isNewPro==1
    % 新建
    projectConfiguration=[];
    temp = inputdlg('输入方案设置名');
    projectConfigurationPath = [pwd,'\projectConfiguration\',temp{1}];
else
    [FileName,PathName] = uigetfile([pwd,'\projectConfiguration\*.mat'],'选择方案设置');
    if FileName~=0
        projectConfigurationPath = [PathName,FileName];
        projectConfiguration = importdata(projectConfigurationPath);
    else
        projectConfiguration=[];
        temp = inputdlg('输入方案设置名');
        projectConfigurationPath = [pwd,'\projectConfiguration\',temp{1}];
    end
end
projectConfiguration = CheckProjectConfiguration(projectConfiguration); % 检查方案设置参数
% 保存当前方案设置为默认
save(projectConfigurationPath,'projectConfiguration');
save([pwd,'\projectConfiguration\defaultProjectDataPath.mat'],'projectDataPath')
%% <2>输入参数
% 根据方案参数设置选择是否更新参数设置，更新时将参数保存在 inputData 文件夹
% 设置不更新时从 inputData 中提取数据，inputData 中没有相关输出参数时强制重新生成
%%  (2.1)输入精确（真实/理想）数据
% 仿真时需要通过真实的位置和姿态得到仿真输入数据，仿真时由轨迹发生器得到。实验时需要与真实数据进行对比
trueTracePath = [projectDataPath,'\trueTrace.mat']; % 默认真实轨迹路径

%trueTracePath = [projectDataPath,'\trueTrace_reverse.mat']; % 默认真实轨迹路径

if( projectConfiguration.isUpdateTrueTrace == 1 )     % 更新 TrueTrace
    oldfolder = cd([pwd,'\code_subfunction\TrueTrace']);
    if projectConfiguration.isKnowTrueTrace==1  % 已知真实轨迹
        % 更新真实轨迹输入数据：调用轨迹放发生器，得到数据后保存到currentData文件夹中    
        disp('更新trueTrace中...')
     %   dbstop in newGetTrueTrace
     dbstop in newGetTrueTrace
        trueTrace = newGetTrueTrace(0); % 调用轨迹发生器得到真实轨迹
        disp('更新trueTrace结束')        
        trueTraceResult = GetTrueTraceResult(trueTrace);
        save([navResultPath,'\trueTraceResult.mat'],'trueTraceResult');
    else        
        trueTrace = GetInitialTrueTrace() ;        
    end    
    cd(oldfolder)
else    % 不更新 TrueTrace
    % 提取现有的trueTrace
    disp('直接提取现有trueTrace')    
  %  trueTracePath = FindMatPath(projectDataPath,'trueTrace');
    if ischar(trueTracePath)==0
        trueTracePath = FindMatPath(projectDataPath,'truetrace');
    end
    if(ischar(trueTracePath) && exist(trueTracePath,'file')~=0)
        trueTrace = importdata(trueTracePath);
    else
        errordlg('未找到真实轨迹数据！')
        return
    end
    disp('成功提取 trueTrace')
    if projectConfiguration.isKnowTrueTrace==1
        oldfolder = cd([pwd,'\code_subfunction\TrueTrace']);
        trueTraceResult = GetTrueTraceResult(trueTrace);
        cd(oldfolder)
        save([navResultPath,'\trueTraceResult.mat'],'trueTraceResult');
    end
end
oldfolder = cd([pwd,'\code_subfunction\TrueTrace']);
% 
% answer = questdlg('是否加入初始误差','初始误差','是','否','否');
answer='否';
if strcmp(answer,'是')
    trueTrace = AddInitialError(trueTrace) ;
else
    trueTrace.InitialPositionError = [0;0;0];
    trueTrace.InitialAttitudeError = [0;0;0];
end
cd(oldfolder)
save(trueTracePath,'trueTrace');

%%  (2.2)输入视觉数据：（得到含噪声的/真实的）特征点
visualInputDataPath = [projectDataPath,'\visualInputData.mat']; % 视觉系统输入数据路径，可在此更改输入参数的路径
if( projectConfiguration.isUpdateVisualData == 1 )    
    % 更新视觉输入数据：调用视觉输入数据的生成函数（由图像得到特征点），得到数据后保存到currentData文件夹中
    disp('更新视觉输入信息中...')
    oldfolder = cd([pwd,'\code_subfunction\GetVisualInputData']);
    if exist([projectDataPath,'\calibData.mat'],'file')
        calibData = importdata([projectDataPath,'\calibData.mat']);
    else
        calibData=[];
    end
   % dbstop in GetVisualInputData 
    visualInputData = GetVisualInputData(projectConfiguration,trueTrace,calibData);
    cd(oldfolder)
    disp('图像特征点提取结束')
    save(visualInputDataPath,'visualInputData');
else
    % 提取现有的视觉输入数据
    disp('直接提取现有视觉输入')
    if(exist(visualInputDataPath,'file'))
        visualInputData = importdata(visualInputDataPath);
    else
        disp('找不到视觉输入数据');
        visualInputData=[];
     %  return
    end
end

%% 输入惯导数据：（得到含噪声的）IMU
imuInputDataPath = [projectDataPath,'\imuInputData.mat']; % 惯导系统输入数据路径，可在此更改输入参数的路径
if( projectConfiguration.isUpdateIMUData == 1 )
    % 更新惯导输入数据：调用轨迹发生器（仿真）/文件选择对话框（实验），得到数据后保存到currentData文件夹中
    disp('更新IMU数据中...')
    oldfolder = cd([pwd,'\code_subfunction\GetIMUInputData']);
    imuInputData = GetIMUInputData(projectConfiguration,trueTrace);
    cd(oldfolder)
    save(imuInputDataPath,'imuInputData');
    disp('IMU数据提取结束')
else
    % 提取现有的IMU数据
    disp('直接提取现有IMU数据')
    if(exist(imuInputDataPath,'file'))
        imuInputData = importdata(imuInputDataPath);
    else
        disp('找不到IMU数据');
       % return
    end
end
if exist('imuInputData','var') 
    if isfield(imuInputData,'realDriftResult')
        realDriftResult = imuInputData.realDriftResult;
        save([navResultPath,'\realDriftResult.mat'],'realDriftResult');
    end
    if strcmp(imuInputData.flag,'exp')
        answer = questdlg('是否改变IMU的真实常漂（不更新噪声）','改变IMU真实常漂','是','否','否');
        if strcmp(answer,'是')
            [pa,na,pg,ng,imuInputData] = GetIMUdrift( imuInputData,trueTrace.planet ) ;
        end
    end
else
    imuInputData = [];
end
%% 输入天文信息
CNSInputData = GenerateCNSdata( trueTrace ) ;
%% 输入滤波参数  NavFilterParameter
NavFilterParameterPath = [projectDataPath,'\NavFilterParameter.mat'];
if(exist(NavFilterParameterPath,'file'))
    NavFilterParameter = importdata(NavFilterParameterPath);
else
    NavFilterParameter = [];
end


%% <3>选择导航方法，进入导航程序
%navMethodStr = {'VNS','SINS','SINSerror_simple_dRdT','SINSerror_subQ_subT','SINS_QT','惯性/视觉/天文`'};
navMethodStr = {'VNS','SINS','dRTw','dQTb','RTb','new_dQTb','new_dQTb_VnsErr','new_dQTc','FPc_UKF','FPc_VnsErr_UKF','new_dQTb_IVC'};
%                 1      2      3       4    5        6            7              8          9              10              11  
[navMethodSelect,isOK] = listdlg('PromptString','选择导航方法（可多选）','ListString',navMethodStr,'InitialValue',[1 5],'ListSize',[180 170],'uh',30);
if(isOK==0)
   % 放弃导航解算
   return;
end
% '惯性/视觉-dRdT为量测','惯性/视觉-RT为量测增广'方法要求先获取Rcc Tcc，因此需先进行 纯视觉解算 或 提取现有RT
if(  ~isempty(find(navMethodSelect==3, 1)) && isempty(find(navMethodSelect==2, 1)) )
   %  选择了'惯性/视觉-dRdT为量测','惯性/视觉-RT为量测增广'方法，却未选择'纯视觉'时
   % 如果选择了纯视觉，则'惯性/视觉-dRdT为量测'的VisualRT参数由纯视觉生成，否则选择加入纯视觉或者提取现有的VisualRT数据
  VisualRTSource = questdlg('INS_VNS_ZdRdT 方法的VisualRT参数如何生成','VisualRT 生成方法','提取现有','纯视觉生成','纯视觉生成') ;
   % VisualRTSource = '提取现有';
   if strcmp(VisualRTSource,'纯视觉生成')==1
        navMethodSelect = [2 navMethodSelect];
   else
        if ~isfield(visualInputData,'VisualRT')
            [VisualRTFileName,VisualRTPathName] = uigetfile([projectDataPath,'\*.mat'],'选择VisualRT文件');
            visualInputData.VisualRT = importdata([VisualRTPathName,VisualRTFileName]);
            save([projectDataPath,'\visualInputData.mat'],'visualInputData') ;
        end
   end
end
% 对选中的导航方法轮流进行解算
if exist([projectDataPath,'\recordStr_INSVNS.mat'],'file')
    recordStr_INSVNS=importdata([projectDataPath,'\recordStr_INSVNS.mat']);
else
    recordStr_INSVNS = cell(1,5) ;
end
%% 参数设置
timeShorted = 1 ;
projectConfiguration.isTrueX0=1;
%%
display(timeShorted)
for i=1:numel(navMethodSelect)
    iNavMethod = navMethodSelect(i);
    integMethod = navMethodStr{iNavMethod};
    tic;
    switch iNavMethod
        
        case 1  % 纯视觉
                visualInputData = dataCompleteCheck('visualInputData',visualInputData) ;
             oldFloder = cd([pwd,'\code_subfunction\VisualOdometerNav']) ; % 进入纯视觉仿真的程序文件路径
            disp('纯视觉实验中...')
            if isfield(trueTrace,'dataSource') && strcmp(trueTrace.dataSource,'kitti')
                dbstop in main_VONav_Geiger 
                [VOResult,visualInputData] = main_VONav_Geiger(visualInputData,trueTrace,timeShorted) ;
            else
                dbstop in main_VONavLM_Exp 
            	[VOResult,visualInputData] = main_VONavLM_Exp(visualInputData,trueTrace,timeShorted);
            end
            disp('纯视觉实验结束')
            VisualRT = visualInputData.VisualRT ;
            cd(oldFloder)
            % 导航解算结果 VOSimuResult 保存到 navResult 文件夹，
            % VisualOut_RT是中间结果，用于其它系统的输入，保存到 \inputData\currentData 文件夹
            save([projectDataPath,'\visualInputData.mat'],'visualInputData')
            save([navResultPath,'\VisualRT.mat'],'VisualRT')
            save([navResultPath,'\VOResult.mat'],'VOResult')
            
    	case 2  % 纯惯导            
            oldFloder = cd([pwd,'\code_subfunction\SINSNav']) ; % 进入纯视觉仿真的程序文件路径
  %  dbstop in main_SINSNav
            [SINS_Result,imuInputData] = main_SINSNav( imuInputData,trueTrace,timeShorted );
            cd(oldFloder) 
            save([navResultPath,'\SINS_Result.mat'],'SINS_Result')
            save([projectDataPath,'\imuInputData.mat'],'imuInputData')
        case {3,4,5}            
            
%             if iNavMethod==3 
%            	oldfolder = cd([pwd,'\code_subfunction\INSVNS']);
%   %    dbstop in main_INS_VNS  at 634
%             [INS_VNS_NavResult,check,recordStr_INSVNS{iNavMethod-2},NavFilterParameter] = main_INS_VNS_Q(integMethod,visualInputData,imuInputData,trueTrace,NavFilterParameter,projectConfiguration.isTrueX0) ;
%         %    [INS_VNS_NavResult,check,recordStr_INSVNS{iNavMethod-2},NavFilterParameter] = main_INS_VNS(integMethod,visualInputData,imuInputData,trueTrace,NavFilterParameter,0) ;  
%      %   [INS_VNS_NavResult,check,recordStr_INSVNS{iNavMethod-2}] = main_INS_VNS34(integMethod,visualInputData,imuInputData,trueTrace,0) ; 
%             INS_VNS_NavResult.recordStr_INSVNS = recordStr_INSVNS;
%             cd(oldfolder);
%             end
            if iNavMethod==3
                oldfolder = cd([pwd,'\code_subfunction\INSVNS\SINSerr_dMove']);
                Z_method = 'd_RTw' ; 
           dbstop in main_SINSerr_dMove
                [INS_VNS_NavResult,check,recordStr_INSVNS{iNavMethod-2},NavFilterParameter] = main_SINSerr_dMove(visualInputData,imuInputData,trueTrace,NavFilterParameter,projectConfiguration.isTrueX0,Z_method,integMethod,timeShorted) ;
                cd(oldfolder);
            end
            
            if iNavMethod==4
                oldfolder = cd([pwd,'\code_subfunction\INSVNS\SINSerr_dMove']);
                Z_method = 'sub_QTb' ; 
            dbstop in main_SINSerr_dMove
                [INS_VNS_NavResult,check,recordStr_INSVNS{iNavMethod-2},NavFilterParameter] = main_SINSerr_dMove(visualInputData,imuInputData,trueTrace,NavFilterParameter,projectConfiguration.isTrueX0,Z_method,integMethod,timeShorted) ;
                cd(oldfolder);
            end
            if iNavMethod==5
                oldfolder = cd([pwd,'\code_subfunction\INSVNS\ZhijieX_QbbTbb']);
           dbstop in main_augZhijie_QT
                [INS_VNS_NavResult,check,recordStr_INSVNS{iNavMethod-2},NavFilterParameter] = main_augZhijie_QT(visualInputData,imuInputData,trueTrace,NavFilterParameter,projectConfiguration.isTrueX0,integMethod,timeShorted) ;
                
                cd(oldfolder);
            end
            
            save([navResultPath,'\INS_VNS_Result_',integMethod,'.mat'],'INS_VNS_NavResult')
            save([navResultPath,'\',integMethod,'_check.mat'],'check')
            save([projectDataPath,'\NavFilterParameter.mat'],'NavFilterParameter')
            save([projectDataPath,'\recordStr_INSVNS.mat'],'recordStr_INSVNS')
            assignin('base',[integMethod,'_check'],check)
            assignin('base',[integMethod,'_NavFilterParameter'],NavFilterParameter)

            
        case{6,7,8,9,10}

            oldfolder = cd([pwd,'\code_subfunction\INSVNS\SINSerr_dMove_new']);
            integMethodDisplay = integMethod ;
            dbstop in main_SINSerr_dMove_new
            [INS_VNS_NavResult,check,recordStr_INSVNS{iNavMethod-2},NavFilterParameter] = main_SINSerr_dMove_new...
                (visualInputData,imuInputData,trueTrace,NavFilterParameter,projectConfiguration.isTrueX0,integMethod,integMethodDisplay,timeShorted) ;
            cd(oldfolder);
            
            save([navResultPath,'\INS_VNS_Result_',integMethod,'.mat'],'INS_VNS_NavResult')
            save([navResultPath,'\',integMethod,'_check.mat'],'check')
            save([projectDataPath,'\NavFilterParameter.mat'],'NavFilterParameter')
            save([projectDataPath,'\recordStr_INSVNS.mat'],'recordStr_INSVNS')
            assignin('base',[integMethod,'_check'],check)
            assignin('base',[integMethod,'_NavFilterParameter'],NavFilterParameter)
            
        case{11}
            % 三组合
            oldfolder = cd([pwd,'\code_subfunction\INSVNS\SINSerr_dMove_new']);
            integMethodDisplay = integMethod ;
            dbstop in main_SINSerr_dMove_new
            [IVC_NavResult,check,recordStr_INSVNS{iNavMethod-2},NavFilterParameter] = main_SINSerr_dMove_new...
                (visualInputData,imuInputData,trueTrace,NavFilterParameter,projectConfiguration.isTrueX0,integMethod,integMethodDisplay,timeShorted,CNSInputData) ;
            cd(oldfolder);
            
            save([navResultPath,'\Result_',integMethod,'.mat'],'IVC_NavResult')
            save([navResultPath,'\',integMethod,'_check.mat'],'check')
            save([projectDataPath,'\NavFilterParameter.mat'],'NavFilterParameter')
            save([projectDataPath,'\recordStr_IVC.mat'],'recordStr_INSVNS')
            assignin('base',[integMethod,'_check'],check)
            assignin('base',[integMethod,'_NavFilterParameter'],NavFilterParameter)
            
        otherwise
            errordlg('此方法还未编写');
            
    end
end
%% 记录输入参数
[~, namet, extt]= fileparts(projectDataPath) ;
textName=[navResultPath,'\',namet,extt,'_实验笔记.txt'];
if exist(textName,'file')
    copyfile(textName,[navResultPath,'\old_',namet,extt,'_实验笔记.txt']);
    open([navResultPath,'\old_',namet,extt,'_实验笔记.txt']);
end
fid = fopen(textName, 'w+');
RecodeInput (fid,visualInputData,imuInputData,trueTrace);

if isfield(visualInputData,'errorStr')
    recordStr_VO = visualInputData.errorStr;
    fprintf(fid,'\n*** 视觉解算误差：\n');
    fprintf(fid,'%s',recordStr_VO);
    fprintf(fid,'VNS解算耗时：%0.5g sec\n',toc);
end
if isfield(imuInputData,'errorStr')
    recordStr_SINS=imuInputData.errorStr ;
    fprintf(fid,'\n*** SINS解算误差：\n');
    fprintf(fid,'%s',recordStr_SINS);
    fprintf(fid,'SINS解算耗时：%0.5g sec\n',toc);
end


for mk=3:(length(recordStr_INSVNS)+2)
    if ~isempty(recordStr_INSVNS{mk-2})
        fprintf(fid,'\n*** %s 解算误差：\n%s',navMethodStr{mk},recordStr_INSVNS{mk-2});
    end
end

fclose(fid);
open(textName)
% 查看结果
% oldFloder = cd([pwd,'\code_subfunction\ResultDispaly']) ; % 进入结果查看路径
% uiwait(ResultDisplay());
% copyfile([pwd,'\code_subfunction\ResultDisplay_exe\ResultDisplay.exe'],[navResultPath,'\ResultDisplay.exe']);
% copyfile([pwd,'\code_subfunction\ResultDisplay_exe\displayCurrent.m'],[projectDataPath,'\displayCurrent.m']);

time_h = size(imuInputData.f,2)/imuInputData.frequency / 3600 ;  % 小时计时间
fprintf('数据长度：%0.2f h',time_h)
disp(['程序运行结束：',navResultPath])
% cd(navResultPath)
%% 退出程序
