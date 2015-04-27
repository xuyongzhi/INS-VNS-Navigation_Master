% 作者：xyz
% 日期：2013.12.23
% 功能：由 Cn2b到姿态角
% 输出：unit='rad'/'degree'（默认rad） 俯仰、横滚、航向
function Attitude = GetAttitude(Cn2b,unit,opintions)
%% 输入
% Cn2b : 导航系到本体系的转移矩阵
% uint : 输出单位 'rad'/'degree'
% opintions ：参数设置
%   航向角定义： opintions.headingScope = 'anticlockwise'/'clockwise   '（'逆时针'/'顺时针'为正）
%               opintions.headingScope = 360 /180（[0 360]和[-180 180])  
% 默认：逆时针，[-180 180]， rad
format long
%% 预读 unit
if ~exist('unit','var')
    unit='rad' ;
end
if isempty(unit) 
    unit='rad' ;
end
% 没有输入unit 或者 unit输入错误 则 unit='rad' 
if ~strcmp(unit,'degree') && ~strcmp(unit,'rad')
  errordlg('unit输入错误') ;
  unit='rad' ;
end
%% 预读opintinous
if ~exist('opintions','var')
    opintions.headingScope=180;
    %disp('默认使用180度范围，逆时针航向定义')
end
if ~isfield(opintions,'headingScope')
    opintions.headingScope='anticlockwise';
end
if ~isfield(opintions,'headingScope')
    opintions.headingScope=360;
end
%%
%Attitude=[fy：俯仰角,hg：横滚角,hx：航向角]
%% 俯仰角[-90,90]，无多值问题
fy = asin( Cn2b(2,3) ) ;      
hg = atan( -Cn2b(1,3)/Cn2b(3,3) ) ;
%% 横滚角定义在[-180,180],存在多值问题
if hg<0&&Cn2b(1,3)<0            %真实区间：[90,180]
    hg = hg+pi ;
end
if hg>0&&Cn2b(1,3)>0            %[-180,90]
    hg = hg-pi ;
end

%% 航向角默认定义（逆时针为正）定义在[0,360],存在多值问题
hx = atan( -Cn2b(2,1)/Cn2b(2,2) ) ;
if strcmp(opintions.headingScope,'clockwise')        % 顺时针
    hx = -hx ;
end
if opintions.headingScope==180
    % 范围：[-180 180]    
    if hx<=0
        if Cn2b(2,2)<0
            hx = hx+pi;    
        end
    else
        if Cn2b(2,2)<0
            hx = hx-pi;
        end
    end
else    
    % 范围：[0 360]
    if hx<=0  % 标准位<0 为了防止出现0到360的剧烈变动 可改用 <-1e-3
        if Cn2b(2,2)<0              %真实区间：[90,180]
            hx = hx+pi ;
        else
            if hx~=0
                hx = hx+2*pi ;          %[270,360]
            end
        end
    elseif hx>0 && Cn2b(2,2)<0        %[180,270]
            hx = hx+pi ;
    end
end


%%
Attitude = [fy;hg;hx] ; % 弧度单位
if strcmp(unit,'degree')
    Attitude = Attitude*180/pi ;  %按角度存储
end
