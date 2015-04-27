% buaa xyz 2013.12.26

% 由trueTrace 获取真实轨迹的绘图结果

function trueTraceResult = GetTrueTraceResult(trueTrace)

true_position = trueTrace.position;
true_attitude = trueTrace.attitude;
% true_velocity = trueTrace.velocity;
if isfield(trueTrace,'velocity')
    true_velocity = trueTrace.velocity;
else
    true_velocity = [];
end
if isfield(trueTrace,'frequency')
    trueTraeFre = trueTrace.frequency;
end
if isfield(trueTrace,'runTime')
    runTime = trueTrace.runTime;    
end
if isfield(trueTrace,'dataSource')
    dataSource = trueTrace.dataSource; 
end
if isfield(trueTrace,'acc_r')
    acc_r = trueTrace.acc_r;
end


% 存储为特定格式：每个变量一个细胞，包含成员：data，name,comment 和 dataFlag,frequency,project,subName
resultNum = 6;
trueTraceResult = cell(1,resultNum);

% 有4个共同的成员
for j=1:resultNum
    trueTraceResult{j}.dataFlag = 'xyz result display format';    
    if isfield(trueTrace,'runTime')
        trueTraceResult{j}.runTime = runTime;    
    end
    trueTraceResult{j}.frequency = trueTraeFre ;
    trueTraceResult{j}.project = 'true';
    trueTraceResult{j}.subName = {'x','y','z'};
end

res_n = 1;
trueTraceResult{res_n}.data = true_position;
trueTraceResult{res_n}.name = 'position(m)';
trueTraceResult{res_n}.comment = '位置';

res_n = res_n+1;
trueTraceResult{res_n}.data = true_attitude*180/pi ;   % 转为角度单位
trueTraceResult{res_n}.name = 'attitude(°)';
trueTraceResult{res_n}.comment = '姿态';
trueTraceResult{res_n}.subName = {'俯仰','横滚','航向'};

res_n = res_n+1;
trueTraceResult{res_n}.data = true_velocity;
trueTraceResult{res_n}.name = 'velocity(m／s)';
trueTraceResult{res_n}.comment = '速度';

if exist('acc_r','var')
    res_n = res_n+1;
    trueTraceResult{res_n}.data = acc_r;
    trueTraceResult{res_n}.name =  'acc_r(m／s^2)';
    trueTraceResult{res_n}.comment = '加速度';
end
trueTraceResult = trueTraceResult(1:res_n) ;