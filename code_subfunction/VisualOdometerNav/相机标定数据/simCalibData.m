%   为视景仿真计算相机标定参数
%       buaa xyz 2013.12.24
%       nuaaxuyognzhi@yeah.net
% 2014.5.15 修改 fov 

%  reslution = [1392 ;1040]
function simCalibData
% 双目相机的10个相机标定参数（用于三维重建）：om,T,fc_left,cc_left,kc_left,alpha_c_left,fc_right
%                                           cc_right,kc_right,alpha_c_right
% Output:
%           om,T: rotation vector and translation vector between right and left cameras (output of stereo calibration)
%           fc_left,cc_left,...: intrinsic parameters of the left camera  (output of stereo calibration)
%           fc_right,cc_right,...: intrinsic parameters of the right camera
%           (output of stereo calibration)
% Input:
%           B,fov,reslution,u

reslution = [1392;1040];

[SceneVisualCalib] = GetCalibData(reslution) ;
save SceneVisualCalib SceneVisualCalib
return 
% 输入
B = 200;    % 基线距离 ，单位为mm
fov = zeros(2,1);
%reslution = [2048 2048] ;
fov(1,1) = 45 ;  % 水平方向的视场角 ，垂直方向的视场角是通过 fov 和分辨率解算出来的

%reslution = [500 500] ;
% 旋转角度
om = [0;0;0];
% 平移
T = [-B;0;0];   % 注意负号

% 计算焦距
fc_left(1,1) = reslution(1)/2/tan(fov(1)/2*pi/180);
fc_left(2,1) = fc_left(1,1) ;
fc_right = fc_left ;

fov(2) = atan(reslution(2)/2/fc_right(1))*180/pi*2;

cc_left = reslution/2 ;
cc_right = reslution/2 ;

kc_left = [0;0;0;0;0];
kc_right = [0;0;0;0;0];

alpha_c_left = 0;
alpha_c_right = 0;

%输出
SceneVisualCalib.om = om;
SceneVisualCalib.T = T;
SceneVisualCalib.fc_left = fc_left;
SceneVisualCalib.fc_right = fc_right;
SceneVisualCalib.cc_left = cc_left;
SceneVisualCalib.cc_right = cc_right;
SceneVisualCalib.kc_left = kc_left;
SceneVisualCalib.kc_right = kc_right;
SceneVisualCalib.alpha_c_left = alpha_c_left;
SceneVisualCalib.alpha_c_right = alpha_c_right;

save('SceneVisualCalib_data','om','T','fc_left','cc_left','kc_left','alpha_c_left','fc_right','cc_right','kc_right','alpha_c_right','fov')

disp('生成并保存 视景仿真相机标定数据成功 SceneVisualCalib_data.mat')