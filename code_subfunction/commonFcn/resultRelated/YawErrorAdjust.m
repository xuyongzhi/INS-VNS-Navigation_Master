%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              xyz
%                           2014.3.20
%                         航向角误差校正
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function yawerror_new = YawErrorAdjust(yawerror_old,unit)
%% 
%   输入的航向角误差单位为 角秒  / rad
%   当 误差值在 360*3600 / 2*pi 附近时表示需要校正

%%
if isempty(unit) 
   warndlg('未输入单位（YawErrorAdjust）'); 
   yawerror_new=[];
   return ;
else
    switch unit
        case 'ArcSec'
            peak = 360*3600 ;
        case 'rad'
            peak = 2*pi ;
        otherwise
            yawerror_new=[];
            warndlg('输入的单位无效(YawErrorAdjust)');
            return;
    end
end
if ( abs(yawerror_old)/(peak) )>0.5
   %% 需要校正
   yawerror_new = yawerror_old - sign(yawerror_old)*peak ;
else
    yawerror_new = yawerror_old ;
end
