%% 当真实轨迹未知时，设置导航初始条件，保存到 TrueTrace 中
function trueTrace = GetInitialTrueTrace()

prompt={'初始绝对大地系位置（经纬度高度）（只用于惯性导航）','初始相对导航系姿态(°)（视觉导航与惯性导航都用）','初始速度(m/s)','天体：月球(m)、地球(e)'};
defaultanswer={'116.35178 39.98057 53.44','0 0 0','0 0 0','e'};
name='设置初始条件';
numlines=1;
answer=inputdlg(prompt,name,numlines,defaultanswer);
initialPosition_e = sscanf(answer{1},'%f');
initialPosition_e(1:2) = initialPosition_e(1:2)*pi/180 ; % 经纬度转换为弧度

initialAttitude_r = sscanf(answer{2},'%f')*pi/180;
initialVelocity_r = sscanf(answer{3},'%f') ;
planet = answer{4};

trueTrace.initialPosition_e = initialPosition_e ;
trueTrace.initialAttitude_r = initialAttitude_r ;
trueTrace.initialVelocity_r = initialVelocity_r ;
trueTrace.planet = planet ;
%% 未知参数

trueTrace.position = [];
trueTrace.attitude = [];
trueTrace.velocity = [];
trueTrace.acc_r = [];
trueTrace.frequency = [];
