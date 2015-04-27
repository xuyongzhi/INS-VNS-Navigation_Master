%% 仿真生成视觉特征点信息
% 2014.5.15
function [visualInputData,SceneVisualCalib] = GetSimuVisualData_dot( trueTrace )
% 输入：世界坐标系下的位置position与姿态attitude
% 输出：每一帧图片的特征点
%%  功能：仿真生成视觉特征点信息
%   函数ScopeToFeaturePos：输入世界坐标系区域，输出该区域内特征点的三维世界系坐标（使得某个区域内特征点是唯一的，因此在不同位置观察到同一区域时会得到同一特征点）
%   函数CameraToScope：输入摄像机的世界坐标系下位置和姿态，输出所能观察到特征点的区域
%   1> 由 CameraToScope 和 ScopeToFeaturePos 得到每个观测点的特征点世界系坐标 watchedFeatureWPos
%   2> 提取匹配成功的特征点 MatchedFeatureWPos
%   3> 将 watchedFeatureWPos 和 MatchedFeatureWPos 转换到摄像机坐标系 watchedFeatureCPos MatchedFeatureCPos
%           检查 watchedFeatureCPos MatchedFeatureCPos 是否在图片内部
%   4> 利用 watchedFeatureCPos MatchedFeatureCPos 画出特征点分别图

if ~exist('trueTrace','var')
   load('trueTrace.mat') 
end

%% 输入： 频率，个数，范围，时间，位置曲线，姿态曲线
format long
answer = inputdlg({'视觉频率：','图像分辨率：'},'视觉仿真参数',1,{'1','1392 1040'});
vnsFre = str2double(answer{1}) ; 
reslution = sscanf(answer{2},'%d');
reslutionStr = [num2str(reslution(1)),'x',num2str(reslution(2))] ;

trueFre = trueTrace.frequency;
true_position = trueTrace.position;
true_attitude = trueTrace.attitude;

trueNum = length(true_position);
vnsNum = fix((trueNum-1)*vnsFre/trueFre+1);

left_scopeP1 = zeros(3,vnsNum);
left_scopeP2 = zeros(3,vnsNum);
right_scopeP1 = zeros(3,vnsNum);
right_scopeP2 = zeros(3,vnsNum);
left_FeatureWPos = cell(1,vnsNum); 	% 世界系坐标
right_FeatureWPos = cell(1,vnsNum);
left_FeaturePixel = cell(1,vnsNum); % 像面坐标 所有特征点
right_FeaturePixel = cell(1,vnsNum);
isInImageLeft = cell(1,vnsNum);    % 像素坐标是否在图片内标志
isInImageRight = cell(1,vnsNum);    % 像素坐标是否在图片内标志
% 计算相机标定参数
[SceneVisualCalib,fov] = GetCalibData(reslution);
fc_left = SceneVisualCalib.fc_left;
fc_right = SceneVisualCalib.fc_right;
cc_left = SceneVisualCalib.cc_left ;
cc_right = SceneVisualCalib.cc_right ;
%% 生成特征点
waitbar_h =waitbar(0,'仿真特征点生成中');
for vns_k = 1:vnsNum
    if(mod(vns_k,ceil(vnsNum/100))==0)
       waitbar(vns_k/vnsNum);% 分100次更新
    end
    true_k = 1+(vns_k-1)*trueFre/vnsFre ;
    left_bPos = true_position(:,true_k) ;
    camAttitude = true_attitude(:,true_k) ;
    Cbr = FCbn(true_attitude(:,true_k)) ;
    
    % 计算范围
    dis = 1 ;   % 长方体区域离相机 dis 米
    width = 3 ; % 长方体区域宽度
    long = 2 ;  % 长方体区域长度
    high = 3 ;  % 长方体区域高度
    cameraHigh = 1.5 ;
    
    left_camPos = left_bPos+Cbr*[0;0;cameraHigh];   % 本体系中心叠加相机高度
    right_camPos = left_camPos+Cbr*[0.2;0;0];
    
    [left_scopeP1(:,vns_k),left_scopeP2(:,vns_k)] = CameraToScope( left_camPos,camAttitude,dis,width,long,high ) ;
    [right_scopeP1(:,vns_k),right_scopeP2(:,vns_k)] = CameraToScope( right_camPos,camAttitude,dis,width,long,high ) ;
    % 计算预选特征点
    step = 1 ;
    left_FeatureWPos{vns_k} = ScopeToFeaturePos( left_scopeP1(:,vns_k),left_scopeP2(:,vns_k),left_camPos,camAttitude,fov,step ) ;
    right_FeatureWPos{vns_k} = ScopeToFeaturePos( right_scopeP1(:,vns_k),right_scopeP2(:,vns_k),right_camPos,camAttitude,fov,step ) ;
    % 把特征点转到成像平面坐标系
    [left_FeaturePixel{vns_k},isInImageLeft{vns_k}] = wPosToPixelLeft(left_FeatureWPos{vns_k},left_camPos,camAttitude,fc_left,cc_left,reslution) ;
    [right_FeaturePixel{vns_k},isInImageRight{vns_k}] = wPosToPixelRight(right_FeatureWPos{vns_k},right_camPos,camAttitude,fc_right,cc_right,reslution) ;
end
close(waitbar_h)
% 丢弃像素坐标不在图片内的点： 像素坐标 和 世界坐标 都要丢
outNumLeft = zeros(vnsNum,1);
for vns_k = 1:vnsNum
    isInImageLeft_k=isInImageLeft{vns_k};
    left_FeatureWPos_k = left_FeatureWPos{vns_k};
    left_FeaturePixel_k = left_FeaturePixel{vns_k};
    for i=1:length(left_FeatureWPos_k)
        if isInImageLeft_k(i)==0            
            left_FeatureWPos_k(i-outNumLeft(vns_k),:) = [];
            left_FeaturePixel_k(i-outNumLeft(vns_k),:) = [];
            outNumLeft(vns_k) = outNumLeft(vns_k)+1;
        end
    end
    left_FeatureWPos{vns_k}=left_FeatureWPos_k;
    left_FeaturePixel{vns_k}=left_FeaturePixel_k;
end
outNumRight = zeros(vnsNum,1);
for vns_k = 1:vnsNum
    isInImageRight_k=isInImageRight{vns_k};
    right_FeatureWPos_k = right_FeatureWPos{vns_k};
    right_FeaturePixel_k = right_FeaturePixel{vns_k};
    for i=1:length(right_FeatureWPos_k)
        if isInImageRight_k(i)==0            
            right_FeatureWPos_k(i-outNumRight(vns_k),:) = [];
            right_FeaturePixel_k(i-outNumRight(vns_k),:) = [];
            outNumRight(vns_k) = outNumRight(vns_k)+1;
        end
    end
    right_FeatureWPos{vns_k}=right_FeatureWPos_k;
    right_FeaturePixel{vns_k}=right_FeaturePixel_k;
end

leftLocCurrent = cell(1,vnsNum-1);  % 像面坐标 匹配成功的特征点
rightLocCurrent = cell(1,vnsNum-1);
leftLocNext = cell(1,vnsNum-1);
rightLocNext = cell(1,vnsNum-1);
featureWPos = cell(1,vnsNum-1);
matchedNum = zeros(1,vnsNum-1); % 四个图匹配成功的特征点个数
aveFeatureNum = zeros(1,vnsNum-1); % 匹配前，四图特征点个数平均值
%% 匹配特征点
% 检查匹配 并 重新排序
waitbar_h =waitbar(0,'仿真特征点匹配中');
for vns_k = 1:vnsNum-1
    if(mod(vns_k,ceil(vnsNum/100))==0)
       waitbar(vns_k/vnsNum);% 分100次更新
    end
    mat_n = 0 ;
    leftLocCurrent_k = zeros(100,2);
    rightLocCurrent_k = zeros(100,2);
    leftLocNext_k = zeros(100,2);
    rightLocNext_k = zeros(100,2);
    featureWPos_k = zeros(100,3);
    % 每个时刻有四副图要匹配
    % 世界系坐标
    left_FeatureWPos_k = left_FeatureWPos{vns_k};
    right_FeatureWPos_k = right_FeatureWPos{vns_k};
    left_FeatureWPos_next = left_FeatureWPos{vns_k+1};
    right_FeatureWPos_next = right_FeatureWPos{vns_k+1};
    % 像素坐标
    left_FeaturePixel_k = left_FeaturePixel{vns_k};
    right_FeaturePixel_k = right_FeaturePixel{vns_k};
    left_FeaturePixel_next = left_FeaturePixel{vns_k+1};
    right_FeaturePixel_next = right_FeaturePixel{vns_k+1};
    
    aveFeatureNum(vns_k) = (length(left_FeatureWPos_k)+length(right_FeatureWPos_k)+length(left_FeatureWPos_next)+length(right_FeatureWPos_next))/4;
    for i=1:length(left_FeatureWPos_k)
        left_FeatureWPos_k_i = left_FeatureWPos_k(i,:);
        
        ismatched = 0 ;
        for ii=1:length(right_FeatureWPos_k)
            right_FeatureWPos_k_ii = right_FeatureWPos_k(ii,:);
            if isMatchFeature(left_FeatureWPos_k_i,right_FeatureWPos_k_ii)
                ismatched=1;
                break;
            end
        end
        if ismatched==1
            ismatched = 0 ;
            for iii=1:length(left_FeatureWPos_next)
                left_FeatureWPos_next_iii = left_FeatureWPos_next(iii,:);
                if isMatchFeature(left_FeatureWPos_k_i,left_FeatureWPos_next_iii)
                    ismatched=1;
                    break;
                end
            end
        end
        if ismatched==1
            ismatched = 0 ;
            for iiii=1:length(right_FeatureWPos_next)
                right_FeatureWPos_next_iiii = right_FeatureWPos_next(iiii,:);
                if isMatchFeature(left_FeatureWPos_k_i,right_FeatureWPos_next_iiii)
                    ismatched=1;
                    break;
                end
            end
        end
        if ismatched==1
            % 记录匹配成功的四个点：按照匹配成功的情况重新存储排列
            mat_n = mat_n+1 ;
            leftLocCurrent_k(mat_n,:) = left_FeaturePixel_k(i,:) ;
            rightLocCurrent_k(mat_n,:) = right_FeaturePixel_k(ii,:);
            leftLocNext_k(mat_n,:) = left_FeaturePixel_next(iii,:);
            rightLocNext_k(mat_n,:) = right_FeaturePixel_next(iiii,:);
            featureWPos_k(mat_n,:) = right_FeatureWPos_next_iiii;
        end
    end
    % 去除无效
    leftLocCurrent_k = leftLocCurrent_k(1:mat_n,:);
    rightLocCurrent_k = rightLocCurrent_k(1:mat_n,:);
    leftLocNext_k = leftLocNext_k(1:mat_n,:);
    rightLocNext_k = rightLocNext_k(1:mat_n,:);
    featureWPos_k = featureWPos_k(1:mat_n,:);
    matchedNum(vns_k)=mat_n;
    % 每个时刻匹配成功的特征点存储到一个细胞内
    leftLocCurrent{vns_k} = leftLocCurrent_k;
    rightLocCurrent{vns_k} = rightLocCurrent_k;
    leftLocNext{vns_k} = leftLocNext_k;
    rightLocNext{vns_k} = rightLocNext_k;
    featureWPos{vns_k} = featureWPos_k ;
end
close(waitbar_h);
% 生成图片
fmt=  'bmp';
dataPath = [pwd,'\data_',reslutionStr,'_',fmt];
imagePath = [dataPath,'\featurePixelImage'];
if isdir(imagePath)
    delete([imagePath,'\*.',fmt]);
else
    mkdir(imagePath);
end

GenImage(left_FeaturePixel,reslution,[imagePath,'\leftImage'],fmt);
GenImage(right_FeaturePixel,reslution,[imagePath,'\rightImage'],fmt);

% 从 loc_xy(k,:)=[x y] 转换为loc_xy(k,:)=[y x] 
leftLocCurrentyx = xyToyx(leftLocCurrent) ;
rightLocCurrentyx = xyToyx(rightLocCurrent) ;
leftLocNextyx = xyToyx(leftLocNext) ;
rightLocNextyx = xyToyx(rightLocNext) ;
% 存储匹配点
visualInputData.leftLocCurrent = leftLocCurrentyx;
visualInputData.rightLocCurrent = rightLocCurrentyx;
visualInputData.leftLocNext = leftLocNextyx;
visualInputData.rightLocNext = rightLocNextyx;

% visualInputData.leftLocCurrent = leftLocCurrent;
% visualInputData.rightLocCurrent = rightLocCurrent;
% visualInputData.leftLocNext = leftLocNext;
% visualInputData.rightLocNext = rightLocNext;

visualInputData.featureWPos = featureWPos;
visualInputData.matchedNum = matchedNum;
visualInputData.aveFeatureNum = aveFeatureNum;
visualInputData.frequency = vnsFre;

save([dataPath,'\visualInputData.mat'],'visualInputData')
save([dataPath,'\SceneVisualCalib.mat'],'SceneVisualCalib')
disp('仿真生成特征点完成')

function locyx = xyToyx(loc_xy)
% 像素坐标，从 loc_xy(k,:)=[x y] 转换为loc_xy(k,:)=[y x] 
locyx = cell(size(loc_xy));
for t=1:length(loc_xy)
    loc_xy_t = loc_xy{t};
    locyx_t = zeros(size(loc_xy_t));
    for k=1:length(loc_xy_t)
        locyx_t(k,:) = [loc_xy_t(k,2) loc_xy_t(k,1)];
    end
    locyx{t} = locyx_t;
end

function FeatureWPos = ScopeToFeaturePos( scopeP1,scopeP2,camPos,camAttitude,fov,step )
% 输入区域长方体的两个对顶点位置 scopeP1,scopeP2
% 特征点在长方体内均匀分布
%   能被 step=0.5 整除的位置为特征点
% 要求：保证在统一空间中，特征点是固定的。只要不同的视场区域中包含了相同的空间，就会得到相同的特征点
%   方法：从整数开始，三维逐步叠加 step

% step = 0.5;
signStep = sign(scopeP2-scopeP1);
stepx = [1;0;0].*signStep ;
stepy = [0;1;0].*signStep ;
stepz = [0;0;1].*signStep ;
% 求离 scopeP1 最近的有效点
startP = round(scopeP1) ;
stopP = round(scopeP2);
stepN = abs((stopP-startP)./step);

FeatureWPos = zeros(10000,3);
n=0;    % 在长方体中搜寻
feature_k = 0; % 特征点个数
% 立体式搜索
for nx = 1:stepN(1)
   for ny = 1:stepN(2) 
      for nz = 1:stepN(3) 
          n = n+1;
          featureSearch = startP+stepx*nx+stepy*ny+stepz*nz ;
          flag = checkIsInEye(camPos,camAttitude,featureSearch,fov);
          if flag==1  
                % 判断为有效的特征点：在视场角内
                feature_k = feature_k+1;    
                FeatureWPos(feature_k,:) = featureSearch ;
       	  end
      end
   end
end

FeatureWPos = FeatureWPos(1:feature_k,:);

%%  相机 位置+姿态->可探测范围
% camPos:相机位置  camAttitude：相机姿态
function [scopeP1,scopeP2] = CameraToScope( camPos,camAttitude,dis,width,long,high )
%      
if ~exist('dis','var')
    dis = 2 ;   % 长方体区域离相机 dis 米
    width = 6 ; % 长方体区域宽度
    long = 15 ;  % 长方体区域长度
    high = 8 ;  % 长方体区域高度
end

Cbr = FCbn(camAttitude);
xDir = Cbr*[1;0;0]; % b系正x方向
yDir = Cbr*[0;1;0]; % b系正y方向
zDir = Cbr*[0;0;1]; % b系正z方向

P0 = camPos+yDir*dis ;  % 向前 dis

scopeP1 = P0-xDir*width/2 ; % 向左 width/2
scopeP1 = scopeP1-zDir*high/2 ; % 向下 high/2
scopeP1(3) = max(0,scopeP1(3)); % 不入地狱
scopeP2 = scopeP1+xDir*width+yDir*long+zDir*high ;

%% 检查是否在视场内
% camPos:相机位置  camAttitude：相机姿态  FeatureWPos：特征点位置
% fov(1):水平视场角 fov(2):垂直视场角   （度）
function flag = checkIsInEye(camPos,camAttitude,FeatureWPos,fov)

fov = fov-[1;1]*1;

Cbr = FCbn(camAttitude);
FeatureDir = FeatureWPos-camPos ;
FeatureDir = FeatureDir/norm(FeatureDir) ;

xDir = Cbr*[1;0;0]; % b系正x方向
FeatureDir1 = FeatureDir;
FeatureDir1(1) = abs(FeatureDir1(1)); % 统一求与x正方向夹角
feaFov1 = (90-acos(xDir'*FeatureDir1)*180/pi)*2 ;  % FeatureDir与yz平面夹角->水平视角

zDir = Cbr*[0;0;1]; % b系正z方向
FeatureDir2 = FeatureDir;
FeatureDir2(3) = abs(FeatureDir2(3)); % 统一求与z正方向夹角
feaFov2 = (90-acos(zDir'*FeatureDir2)*180/pi)*2 ;  % FeatureDir与xy平面夹角->垂直视角

if fov(1)>feaFov1 && fov(2)>feaFov2
    flag = 1;   % 在视场内
else
    flag = 0 ; 
end

%% 世界坐标系->摄像机坐标系
% 判断是否在图片内 isInImage
function [left_FeaturePixel,isInImageLeft] = wPosToPixelLeft(left_FeatureWPos,left_camPos,camAttitude,fc_left,cc_left,reslution)


Crb = FCbn(camAttitude)';
feaNum = length(left_FeatureWPos);
left_FeaturePixel = zeros(feaNum,2);
isInImageLeft = ones(feaNum,1);

Cbc = [1,      0,     0 ;
           0,      0,    -1 ;
           0,      1,     0 ];
for k=1:feaNum
    %世界坐标系转->b系
    left_FeatureBPos = Crb * (left_FeatureWPos(k,:)' - left_camPos);   % left_FeatureBPos：特征点在本体系下的坐标
    % b系->左c系
    left_FeatureCPos = Cbc * left_FeatureBPos ; % 特征点在摄像机坐标下的米制单位坐标
    left_FeaturePixel(k,1) = fc_left(1)*left_FeatureCPos(1)/left_FeatureCPos(3);    % % 特征点在摄像机成像平面下的像素坐标  x
    left_FeaturePixel(k,2) = fc_left(2)*left_FeatureCPos(2)/left_FeatureCPos(3);    % y

    % 将原点转到左上角
    left_FeaturePixel(k,:) = left_FeaturePixel(k,:)+cc_left' ;
    
    if left_FeaturePixel(k,1)<0 || left_FeaturePixel(k,2)<0 || left_FeaturePixel(k,1)>reslution(1) || left_FeaturePixel(k,2)>reslution(2) 
        isInImageLeft(k) = 0;
    end
    
end

function [right_FeaturePixel,isInImageRight] = wPosToPixelRight(right_FeatureWPos,right_camPos,camAttitude,fc_right,cc_right,reslution)


Crb = FCbn(camAttitude)';

feaNum = length(right_FeatureWPos);
right_FeaturePixel = zeros(feaNum,2);
isInImageRight = ones(feaNum,1);

Cbc = [1,      0,     0 ;
           0,      0,    -1 ;
           0,      1,     0 ];
  
for k=1:feaNum    
    right_FeatureBPos = Crb * (right_FeatureWPos(k,:)' - right_camPos);
    right_FeatureCPos = Cbc * right_FeatureBPos ; % 特征点在摄像机坐标下的米制单位坐标
    right_FeaturePixel(k,1) = fc_right(1)*right_FeatureCPos(1)/right_FeatureCPos(3);    % % 特征点在摄像机成像平面下的像素坐标   
    right_FeaturePixel(k,2) = fc_right(2)*right_FeatureCPos(2)/right_FeatureCPos(3);
    % 将原点转到左上角
    right_FeaturePixel(k,:) = right_FeaturePixel(k,:)+cc_right' ;

    if  right_FeaturePixel(k,1)<0 ||right_FeaturePixel(k,2)<0 ||right_FeaturePixel(k,1)>reslution(1) || right_FeaturePixel(k,2)>reslution(2)
        isInImageRight(k) = 0;
    end
    
end


%% 定义特征点是否匹配
function isMatch = isMatchFeature(P1,P2)
e=abs(P1-P2);
maxe = 1e-5;
if e(1)<maxe && e(2)<maxe && e(3)<maxe 
    isMatch = 1;
else
    isMatch = 0;
end


%% 根据成像特征点画出图片
% % pixelLoc(k,:)=[x y]
% reslution [x y] 分辨率
% imageBaseName 基本名，可包含路径
function GenImage(pixelLoc,reslution,imageBaseName,fmt)

timeNum = length(pixelLoc);
for t=1:timeNum
    pixelLoc_t = pixelLoc{t};
    featureNum = length(pixelLoc_t);
    image_t = ones(reslution');
    for k=1:featureNum
        d=round(pixelLoc_t(k,:));
        dx=d(1);
        dy=d(2);
        image_t(dx-1:dx+1,dy-1:dy+1)=0;
  
    end
    imwrite(image_t,[imageBaseName,'_',num2str(t),'.',fmt]);
end
