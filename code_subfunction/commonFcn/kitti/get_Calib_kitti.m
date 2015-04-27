
function get_Calib_kitti()

format long
dbstop error
clc
clear all
close all

% raw_data_dir = 'E:\惯性视觉导航\NAVIGATION\data\kitti\raw data\';

% raw_data_dir = 'I:\NAVIGATION\data\';
raw_data_dir='E:\惯性视觉导航\NAVIGATION\data_kitVO\';
numStr = '0034';
 dateStr = '2011_10_03';
 
% numStr = '0034';
%  dateStr = '2011_10_30';

base_dir = [raw_data_dir,sprintf('%s_drive_%s\\',dateStr,numStr)];
calib_name = sprintf('%s_drive_%s\\%s_calib\\%s\\',dateStr,numStr,dateStr,dateStr);

calib_dir = [raw_data_dir,calib_name];

calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'));
Tr_imu_to_velo = loadCalibration_imuTovelo(fullfile(calib_dir,'calib_imu_to_velo.txt')) ;
S = calib.S{1};
K_0 = calib.K{1};
K_1 = calib.K{2} ;
D_0 = calib.D{1};
D_1 = calib.D{2};
T_0 = calib.T{1};
T_1 = calib.T{2};
R_0 = calib.R{1} ;
R_1 = calib.R{2} ;
P_rect_0 = calib.P_rect{1} ;
P_rect_1 = calib.P_rect{2} ;
S_rect = calib.S_rect{1};
R_rect_0 = calib.R_rect{1} ;
R_rect_1 = calib.R_rect{2} ;

% 计算 IMU 与 cam之间的安装关系
Tr_imu_to_cam = Tr_velo_to_cam * Tr_imu_to_velo ;
R_imu_to_cam = Tr_imu_to_cam(1:3,1:3) ;
Rbc = R_imu_to_cam*[0 1 0;-1 0 0;0 0 1];
Tcb_c = Tr_imu_to_cam(1:3,4);
Cb1c = [1, 0, 0;     % 本体系到摄像机坐标系:绕x轴转动-90度
        0, 0,-1;     % 摄像机坐标系c： x和y在相机平面，y向下，x向右，z向前
        0, 1, 0];
Cbb1 = Cb1c' * Rbc ;
opintions.headingScope = 180 ;
cameraSettingAngle = GetAttitude(Cbb1,'rad',opintions) ;
Tcb_c_str = sprintf('%0.4f  ',Tcb_c);
sprintf('Tcb_c的设计值为 -0.32 0.72 -1.08，标定值为：%s\n',Tcb_c_str)
cameraSettingAngle_str = sprintf('%0.4f  ',cameraSettingAngle*180/pi);
sprintf('cam0相对IMU安装角的设计值为 0 0 0，标定值为：%s °\n',cameraSettingAngle_str)
%%%%%%%%%%%%% ??????????????????
% 为什么 K 和 P_rect 中的焦距差很多？矫正肿么会改变焦距？

% 不考虑：左相机的位置误差  T_0 R_rect_0
%% 矫正前的相机标定参数
calibData_extract.T = (T_1-T_0)*1000 ;    % 转换为 mm 单位
calibData_extract.om = R_1*R_0' ;
calibData_extract.fc_left = [K_0(1,1);K_0(2,2)] ;
calibData_extract.fc_right = [K_1(1,1);K_1(2,2)];
calibData_extract.cc_left = [K_0(1,3);K_0(2,3)];
calibData_extract.cc_right = [K_1(1,3);K_1(2,3)];
calibData_extract.kc_left = zeros(5,1);
calibData_extract.kc_right = zeros(5,1);
calibData_extract.alpha_c_left = 0 ;
calibData_extract.alpha_c_right = 0;
display(calibData_extract.fc_left)

calibData_extract.cameraSettingAngle = cameraSettingAngle ;
calibData_extract.Tcb_c = Tcb_c;
calibData_extract.Rbc = Rbc ;
%% 矫正后的相机标定参数
calibData_rect.T = T_1*1000 ;    % 转换为 mm 单位
om_R = R_rect_1*R_rect_0' ;
om_R=R_1;
om_R = [0;0;0]; % 校正后为0
calibData_rect.om = om_R ;
calibData_rect.fc_left = [P_rect_0(1,1);P_rect_0(2,2)] ;
calibData_rect.fc_right = [P_rect_1(1,1);P_rect_1(2,2)];
calibData_rect.cc_left = [P_rect_0(1,3);P_rect_0(2,3)];
calibData_rect.cc_right = [P_rect_1(1,3);P_rect_1(2,3)];
calibData_rect.kc_left = zeros(5,1);
calibData_rect.kc_right = zeros(5,1);
calibData_rect.alpha_c_left = 0 ;
calibData_rect.alpha_c_right = 0;

calibData_rect.cameraSettingAngle = cameraSettingAngle ;
calibData_rect.Tcb_c = Tcb_c;
calibData_rect.Rbc = Rbc ;

opintions.headingScope = 180 ;
om_R_rect_1 = GetAttitude(R_rect_1,'rad',opintions)*180/pi

om_rect_1_0 = GetAttitude(R_rect_1*R_rect_0','rad',opintions)*180/pi


om_1 = GetAttitude(R_1,'rad',opintions)*180/pi

display(calibData_rect.fc_left)
%% 
if exist([base_dir,'\visualInputData.mat'],'file')
    visualInputData = importdata([base_dir,'\visualInputData.mat']);
end
visualInputData.dataSource = 'kitti';
visualInputData.calibData = calibData_rect ;

calibData = calibData_rect ;
% if isfield(visualInputData,'VisualRT')
%     visualRbbTbb = backRT_to_frontRT(visualInputData,base_dir);
%     save([base_dir,'\visualRbbTbb.mat'],'visualRbbTbb')
% end

if isfield(visualInputData,'errorStr')
    errorStr = visualInputData.errorStr;
    fid=fopen([base_dir,'视觉导航结果.txt'],'w+');
    fprintf(fid,'视觉导航结果:\n%s',errorStr);
    fclose(fid);
end
save([base_dir,'\calibData.mat'],'calibData')
save([base_dir,'\visualInputData.mat'],'visualInputData')

disp('get_Calib_kitti OK:在以下路径中保存到calibData和visualInputData中 ')
display(base_dir)