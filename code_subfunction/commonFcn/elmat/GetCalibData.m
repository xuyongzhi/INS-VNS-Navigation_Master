%   为视景仿真计算相机标定参数
%       buaa xyz 2013.12.24
%       nuaaxuyognzhi@yeah.net
% 2014.5.15 修改 fov 

%  reslution = [1392 ;1040]
function calibData = GetCalibData(reslution)
% 双目相机的10个相机标定参数（用于三维重建）：om(rad),T(mm),fc_left,cc_left,kc_left,alpha_c_left,fc_right
%                                           cc_right,kc_right,alpha_c_right
% Output:
%           om,T: rotation vector and translation vector between right and left cameras (output of stereo calibration)
%           fc_left,cc_left,...: intrinsic parameters of the left camera  (output of stereo calibration)
%           fc_right,cc_right,...: intrinsic parameters of the right camera
%           (output of stereo calibration)
% Input:
%           B,fov,reslution,u

if ~exist('reslution','var')
    reslution = [1024;1024];
end
reslutionStr = sprintf('%d ',reslution);
answer = inputdlg({ 'Tcb_c:本体系到相机系平移矢量(m)','左相机安装角(俯仰 横滚 偏航)°','resolution ratio','B:基线距离/mm',...
                    'om:从左相机到右相机的安装角度/°','fov[水平 垂直]/°(必须有一个为0，表示被动的视场角)               .'},...
                    '仿真相机标定参数（无畸变）',1,{'0.2 1.2 -0.8','0 0 0',reslutionStr,'200','0 0 0','0 45',});
                
str = sprintf('Tcb_c:本体系到相机系平移矢量(m) = %s\n',answer{1});                
str = sprintf('%s 左相机安装角(俯仰 横滚 偏航)° = %s\n',str,answer{2});    
str = sprintf('%s resolution ratio = %s\n',str,answer{3});  
str = sprintf('%s B:基线距离/mm = %s\n',str,answer{4});  
str = sprintf('%s om:从左相机到右相机的安装角度/° = %s \n',str,answer{5});  
str = sprintf('%s fov[水平 垂直]/°(必须有一个为0，表示被动的视场角) = %s\n',str,answer{6});  

Tcb_c = sscanf(answer{1},'%f');
cameraSettingAngle = sscanf(answer{2},'%f')*pi/180'; 
reslution = sscanf(answer{3},'%f');
B = sscanf(answer{4},'%f');
om = sscanf(answer{5},'%f')*pi/180';
fov = sscanf(answer{6},'%f')';

T = [-B;0;0];   % 注意负号
if fov(1)*fov(2)~=0
   errordlg('fov 必须有一个为0，表示被动'); 
end
if fov(1)==0
   %% 给定 垂直视场角 -> 水平视场角
%     fov(2,1) = 35 ; 
    % 计算焦距
    fc_left(2,1) = reslution(2)/2/tan(fov(2)/2*pi/180);
    fc_left(1,1) = fc_left(2,1) ;
    fov(1) = atan(reslution(1)/2/fc_left(1))*180/pi*2;
    fc_right = fc_left ;
else
    %% 给定 水平视场角 -> 垂直视场角
%     fov(1,1) = 45 ;  % 水平方向的视场角 ，垂直方向的视场角是通过 fov 和分辨率解算出来的
%     % 计算焦距
    fc_left(1,1) = reslution(1)/2/tan(fov(1)/2*pi/180);
    fc_left(2,1) = fc_left(1,1) ;
    fov(2) = atan(reslution(2)/2/fc_left(1))*180/pi*2;
    fc_right = fc_left ;
end

%% 
cc_left = reslution/2 ;
cc_right = reslution/2 ;

kc_left = [0;0;0;0;0];
kc_right = [0;0;0;0;0];

alpha_c_left = 0;
alpha_c_right = 0;


%输出
calibData.om = om;  % mm
calibData.T = T;
calibData.fc_left = fc_left;
calibData.fc_right = fc_right;
calibData.cc_left = cc_left;
calibData.cc_right = cc_right;
calibData.kc_left = kc_left;
calibData.kc_right = kc_right;
calibData.alpha_c_left = alpha_c_left;
calibData.alpha_c_right = alpha_c_right;
calibData.fov = fov;
calibData.cameraSettingAngle = cameraSettingAngle ;
calibData.Tcb_c = Tcb_c ;
calibData.str = str ;

save('SceneVisualCalib_data','om','T','fc_left','cc_left','kc_left','alpha_c_left','fc_right','cc_right','kc_right','alpha_c_right','fov','cameraSettingAngle')

disp('生成并保存 视景仿真相机标定数据成功 SceneVisualCalib_data.mat')
% 
% om=om'
% T=T'
% fc_left=fc_left'
% fc_right=fc_right'
% cc_left=cc_left'
% cc_right=cc_right'
% kc_left=kc_left'
% kc_right=kc_right'
% alpha_c_left=alpha_c_left'
% alpha_c_right=alpha_c_right'
% fov=fov'
