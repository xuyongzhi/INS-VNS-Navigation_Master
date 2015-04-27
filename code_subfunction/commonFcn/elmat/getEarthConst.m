%  buaa xyz 2014.1.9

% 地球常量
% http://www.solarsystem.nasa.gov/planets/
%% 输出地球相关常数
% 注意输出的g只有大小，不包含符号
function earth_const = getEarthConst(place)
format long

%earth_const.g0=9.7803267714;
%% 更新g 方案
% earth_const.g0=9.80665;
% 
% earth_const.gk1 = 0.0052884;
% earth_const.gk2 = 0.0000059;

%% 不更新g方案：为了简化计算，直接取实验当地的重力加速度（仿真时不这么处理）
earth_const.g0=9.80665; 

earth_const.gk1 = 0 ;
earth_const.gk2 = 0 ;

earth_const.wie=7.292115147e-5;  
earth_const.Re = 6378245 ;
% earth_const.Re = 6378137;   % kitti
earth_const.e = 1/298.3 ;

if exist('place','var')
    if strcmp(place,'kitti')
        earth_const.Re = 6378137;   % kitti
        earth_const.g0=9.80665; 
    end
end

