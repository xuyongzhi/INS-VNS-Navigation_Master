%% 从  calibData 提取12个标定参数
%       buaaxyz

%   Tcb_c:本体系到相机系平移矢量(m)
%   Rbc:（左）相机坐标系到本体系旋转矩阵
%   cameraSettingAngle：左相机相对本体系安装角(俯仰 横滚 偏航) rad
%   om：左相机到右相机的安装角度 (俯仰 横滚 偏航) rad
%   T ： 左相机到右相机的平移矢量 = 右相机坐标系中左相机的位置 =（平行双目时） [-B 0 0] （B为基线）  m
%   cc_left,cc_right： 左右相机的主点坐标（像素）
%   fc_left,fc_right： 左右相机焦距 （像素）
%   畸变参数： alpha_c_left,alpha_c_right,kc_left,kc_right
function [ Rbc,Tcb_c,T,alpha_c_left,alpha_c_right,cc_left,cc_right,fc_left,fc_right,kc_left,kc_right,om,calibData ] = ExportCalibData( calibData )

T = calibData.T;  % mm为单位，列存储
alpha_c_left = calibData.alpha_c_left;
alpha_c_right = calibData.alpha_c_right;
cc_left = calibData.cc_left;
cc_right = calibData.cc_right;
fc_left = calibData.fc_left;
fc_right = calibData.fc_right;
kc_left = calibData.kc_left;
kc_right = calibData.kc_right;
om = calibData.om;

if ~isfield(calibData,'cameraSettingAngle')
    answer = inputdlg({'相机安装角°'},'俯仰 横滚 偏航 ',1,{'0 0 0'});
    cameraSettingAngle = sscanf(answer{1},'%f')'*pi/180;  
    calibData.cameraSettingAngle = cameraSettingAngle ;
else
    cameraSettingAngle = calibData.cameraSettingAngle ;
end

if ~isfield(calibData,'Tcb_c')
    calibData.Tcb_c = [0;0;0] ;
    disp('没有Tcb_c，取0');
end
Tcb_c = calibData.Tcb_c ;
if ~isfield(calibData,'Tcb_c_error')
    calibData.Tcb_c_error = [0;0;0] ;
    disp('Tcb_c_error，取0');
end

   if isfield(calibData,'isEnableCalibError')
        if  calibData.isEnableCalibError==1
            disp('双目视觉误差激活');
            T =  T+calibData.T_error ;
            om = om+calibData.om_error ;
            cameraSettingAngle = cameraSettingAngle+calibData.cameraSettingAngle_error ;
            Tcb_c = Tcb_c + calibData.Tcb_c_error ;
        end
    end

    Cbb1 = FCbn(cameraSettingAngle)';   % cameraSettingAngle 是从b系到c(中间)系的转动角
    Cb1c = [1, 0, 0;     % 本体系到摄像机坐标系:绕x轴转动-90度
           0, 0,-1;     % 摄像机坐标系c： x和y在相机平面，y向下，x向右，z向前
           0, 1, 0];    % 本体系b：x向右，y向前，z向上
    Rbc = Cb1c*Cbb1 ;
    calibData.Rbc = Rbc ;
