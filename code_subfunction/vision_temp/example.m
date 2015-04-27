%% 读视觉特征点示例

path = 'E:\惯性视觉导航\发给宁老师\特征点数据\S型轨迹' ;
path = pwd;
if ~exist([path,'\visualInputData.mat'],'file')
   error('路径不对') 
end
visualInputData = importdata([path,'\visualInputData.mat']);

%% 特征点像素坐标
[ leftLocCurrent,rightLocCurrent,leftLocNext,rightLocNext,featureCPosCurrent,featureCPosNext,matchedNum ] = ReadVisualFeature(visualInputData) ;

disp('*** 第1个时刻的特征点匹配结果：')

disp('第1个时刻左图第5个特征点：')
leftLoc_1_1 = leftLocCurrent{1}(5,:)

disp('第1个时刻右图第5个特征点：')
rightLoc_1_1 = rightLocCurrent{1}(5,:)

disp('第2个时刻左图第5个特征点：')
leftLoc_1_2 = leftLocNext{1}(5,:)

disp('第2个时刻右图第5个特征点：')
rightLoc_1_2 = rightLocNext{1}(5,:)

disp('第1次匹配 第1时刻左/右图第5个特征点的 左相机 三维坐标：')
camPostion1 = featureCPosCurrent{1}(5,:) 
disp('第1次匹配 第2时刻左/右图第5个特征点的 左相机 三维坐标：')
camPostion2 = featureCPosNext{1}(5,:) 
%% 相机标定参数

[ Rbc,Tcb_c,T,alpha_c_left,alpha_c_right,cc_left,cc_right,fc_left,fc_right,kc_left,kc_right,om,calibData ] = ExportCalibData( visualInputData.calibData ) ;
om = [0 0 0]';
%   Tcb_c:本体系到相机系平移矢量(m)
%   Rbc:（左）相机坐标系到本体系旋转矩阵
%   cameraSettingAngle：左相机相对本体系安装角(俯仰 横滚 偏航) rad
%   om：左相机到右相机的安装角度 (俯仰 横滚 偏航) rad
%   T ： 左相机到右相机的平移矢量 = 右相机坐标系中左相机的位置 =（平行双目时） [-B 0 0] （B为基线）  m
%   cc_left,cc_right： 左右相机的主点坐标（像素）
%   fc_left,fc_right： 左右相机焦距 （像素）
%   畸变参数： alpha_c_left,alpha_c_right,kc_left,kc_right

%% 三维重建过程
featureCPosCurrent = cell(1,length(leftLocCurrent));           % 前一时刻特征点在（左）摄像机坐标系下的坐标
featureCPosNext = cell(1,length(leftLocCurrent));              % 后一时刻特征点在（左）摄像机坐标系下的坐标
wh=waitbar(0,'三维重建中...');
for k=1:length(leftLocCurrent)
    P1 = zeros(matchedNum(k),3);   
   P2 = zeros(matchedNum(k),3);    
    for j=1:matchedNum(k)
        xL = [leftLocCurrent{k}(j,2);leftLocCurrent{k}(j,1)]; % 第i个时刻的第j个当前帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
        xR = [rightLocCurrent{k}(j,2);rightLocCurrent{k}(j,1)];        
        [P1(j,:),~] = stereo_triangulation(xL,xR,om,T'/1000,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right);
        
        xL = [leftLocNext{k}(j,2);leftLocNext{k}(j,1)]; % 第i个时刻的第j个下一帧帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
        xR = [rightLocNext{k}(j,2);rightLocNext{k}(j,1)];
       	[P2(j,:),~] = stereo_triangulation(xL,xR,om,T'/1000,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right);
    end
    featureCPosCurrent{k} = P1;
    featureCPosNext{k} = P2;
    
    if mod(k,fix(length(leftLocCurrent)/50))==0
        waitbar(k/length(leftLocCurrent),wh);
   end
end
close(wh)

save featureCPosCurrent featureCPosCurrent
save featureCPosNext featureCPosNext

disp('ok')