%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.3
% 作者：xyz
% 功能：当方案设置参数中设置不全时进行提示并重新进行设置
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function projectConfiguration = CheckProjectConfiguration(projectConfiguration)

%% 检查 isUpdateVisualData
if(~isfield(projectConfiguration,'isUpdateVisualData'))
    % 无是否更新设置，实时输入
    choice = menu({'未设置是否更新视觉信息';'是否更新视觉输入信息？'},'是','否');
    if choice~=0
        if(choice==1)
            projectConfiguration.isUpdateVisualData = 1;    % 是
        else
            projectConfiguration.isUpdateVisualData = 0;    % 否
        end
    end
end
%% 检查 visualDataSource
    if(~isfield(projectConfiguration,'visualDataSource'))
        % 视觉：实验/仿真
        choice = menu({'视觉信息源设置为空';'请重新设置'},'实验','仿真');
        if choice~=0
            if(choice==1)
                projectConfiguration.visualDataSource = 'e';    % 实验
            else
                projectConfiguration.visualDataSource = 's';    % 仿真
            end
        end
    end
%% 检查 isUpdateIMUData
if(~isfield(projectConfiguration,'isUpdateIMUData'))
    % 无是否更新设置，实时输入
    choice = menu({'未设置是否更新IMU数据';'是否更新IMU数据？'},'是','否');
    if choice~=0
        if(choice==1)
            projectConfiguration.isUpdateIMUData = 1;    % 是
        else
            projectConfiguration.isUpdateIMUData = 0;    % 否
        end
    end
end
%% 检查 imuDataSource
if(~isfield(projectConfiguration,'imuDataSource'))
    % 设置惯导实验还是仿真
    choice = menu({'未设置IMU数据源';'请重新设置'},'实验','仿真');
    if choice~=0
        if(choice==1)
            projectConfiguration.imuDataSource = 'e';    % 实验
        else
            projectConfiguration.imuDataSource = 's';    % 仿真
        end
    end
end

%% isKnowTrueTrace
if(~isfield(projectConfiguration,'isKnowTrueTrace'))
    % 无是否更新设置，实时输入
    choice = menu({'未设置是否已知真实轨迹';'是否已知trueTrace？'},'是','否');
    if choice~=0
        if(choice==1)
            projectConfiguration.isKnowTrueTrace = 1;    % 是
        else
            projectConfiguration.isKnowTrueTrace = 0;    % 否
        end
    end
end
%% if projectConfiguration.isKnowTrueTrace==1
    % 检查 isUpdateTrueTrace
    if(~isfield(projectConfiguration,'isUpdateTrueTrace'))
        % 无是否更新设置，实时输入
        choice = menu({'未设置是否更新真实轨迹数据trueTrace';'是否更新trueTrace？'},'是','否');
        if choice~=0
            if(choice==1)
                projectConfiguration.isUpdateTrueTrace = 1;    % 是
            else
                projectConfiguration.isUpdateTrueTrace = 0;    % 否
            end
        end
    end
% else
%     if isfield(projectConfiguration,'isUpdateTrueTrace')
%         projectConfiguration = rmfield(projectConfiguration,'isUpdateTrueTrace');
%     end
% end
%% isTrueX0
% if(~isfield(projectConfiguration,'isTrueX0'))
    % 是否采用准确的初始状态
  %  choice = menu({'是否采用准确的初始状态';'是否采用准确的初始状态，或者采用0初始状态？'},'准确X0','陀螺加计漂移0初值');
    choice = 2;
    if choice~=0
        if(choice==1)
            projectConfiguration.isTrueX0 = 1;    % 是
        else
            projectConfiguration.isTrueX0 = 0;    % 否
        end
    end
% end