%% 从  calibData 提取各个标定参数

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
cameraSettingAngle = calibData.cameraSettingAngle ;
if ~isfield(calibData,'Tcb_c')
    calibData.Tcb_c = [0;0;0] ;
    disp('没有Tcb_c，取0');
end
Tcb_c = calibData.Tcb_c ;
if ~isfield(calibData,'Tcb_c_error')
    calibData.Tcb_c_error = [0;0;0] ;
    disp('Tcb_c_error，取0');
end
if ~isfield(calibData,'Rbc')
   if isfield(calibData,'isEnableCalibError')
        if  calibData.isEnableCalibError==1
            disp('双目视觉误差激活');
            T =  T+calibData.T_error ;
            om = om+calibData.om_error ;
            if size(cameraSettingAngle,1)~=size(calibData.cameraSettingAngle_error,1)
                cameraSettingAngle=cameraSettingAngle';
            end
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
else
    Rbc = calibData.Rbc ;
end