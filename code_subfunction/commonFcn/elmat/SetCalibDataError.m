%   为视景仿真设定相机标定参数 误差
%       buaa xyz 2014.7.17
%       nuaaxuyognzhi@yeah.net

function calibData = SetCalibDataError(calibData)


%% 添加双目视觉系统误差
%             激活      Tcb_c_error    cameraSettingAngle_true     T_error           om_error       Flag
default_N1 = {'1','[ 1 -1 -1 ]/1000*5',  '[1 -1 1]*0.2',   '[1 1 -1]*0.1',   '[-1 1 -1]*0.02','噪声N1'} ; % 左相机安装误差更大
default_N2 = {'1','[ -1 1 1 ]/1000*1',  '[1 -1 -1]*0.1',   '[1 -1 -1]*0.5',   '[1 -1 1]*0.1','噪声N2'} ; % 双目外参误差更大
default_N3 = {'1','[ -1 1 1 ]/1000*5',  '[1 1 -1]*0.5',   '[1 -1 -1]*0',   '[1 -1 1]*0','噪声N3'} ;       % 仅左相机安装误差
default_N4 = {'1','[ 1 -1 -1 ]/1000*0',  '[1 -1 1]*0',   '[1 1 -1]*2',   '[-1 1 -1]*0.3','噪声N4'} ; % 仅双目外参误差
default_N5 = {'1','[ 1 -1 -1 ]/1000*5',  '[1 -1 1]*0.5',   '[1 1 -1]*2',   '[-1 1 -1]*0.3','噪声N5'} ;

default_M1 = {'1','[ -1 1 1 ]/1000*7',  '[1 1 -1]*1',   '[1 -1 -1]*0',   '[1 -1 1]*0','噪声M1'} ;       % 仅左相机安装误差
default_M2 = {'1','[ 1 -1 -1 ]/1000*0',  '[1 -1 1]*0',   '[1 1 -1]*2',   '[-1 1 -1]*1','噪声M2'} ;      %仅双目外参误差
default_M3 = {'1','[ 1 -1 -1 ]/1000*7',  '[1 -1 1]*1',   '[1 1 -1]*2',   '[-1 1 -1]*1','噪声M3'} ;
default_M4 = {'1','[ 1 -1 1 ]/1000*7',  '[1 -1 -1]*1',   '[1 -1 -1]*2',   '[1 1 1]*2','噪声M4'} ;
default_M5 = {'1','[ 1 -1 1 ]/1000*0',  '[1 -1 -1]*0',   '[1 -1 -1]*0',   '[1 1 1]*5','噪声M5'} ;

answer = inputdlg({'激活双目视觉系统误差','Tcb_c_error：左相机到右相机平移矢量误差(m)','左相机安装角 误差 (俯仰 横滚 偏航)°(cameraSettingAngle_true)',...
    '左右相机相对位置误差（T_error）/mm','右相机相对左相机安装角(om_error)/°','噪声标记'},'双目视觉系统误差设置',1,default_N5);
 
isEnableCalibError = str2double(answer{1}) ;
Tcb_c_error = eval(answer{2}) ;
cameraSettingAngle_error = eval(answer{3})*pi/180;
T_error = -eval(answer{4});
om_error = eval(answer{5})*pi/180;
calibErrorFlag = answer{6} ;

calibData.isEnableCalibError = isEnableCalibError ;
calibData.Tcb_c_error = Tcb_c_error' ;
calibData.cameraSettingAngle_error = cameraSettingAngle_error' ;
calibData.T_error = T_error' ;
calibData.om_error = om_error' ;
calibData.calibErrorFlag = calibErrorFlag ;

