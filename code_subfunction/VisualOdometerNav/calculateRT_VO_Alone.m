%% 独立的 特征点->Rbb+Tbb 计算函数
% 具有分段存储和断点续算功能
function [visualInputData] = calculateRT_VO_Alone(visualInputData)

if ~exist('visualInputData','var')
   load('visualInputData.mat') 
end
visualInputData = RejectUselessFeaturePoint(visualInputData,5);    % 特征点视差检查（1024x1024,45°角，景深=247/dX，dX=5时景深为49m，dX=6时41.2m，dX=7时35m，dX=8时31m）

%% 视觉数据
leftLocCurrent = visualInputData.leftLocCurrent ;
rightLocCurrent = visualInputData.rightLocCurrent ;
leftLocNext = visualInputData.leftLocNext ;
rightLocNext = visualInputData.rightLocNext ;
matchedNum = visualInputData.matchedNum ;

%aveFeatureNum = visualInputData.aveFeatureNum ;
timeNum = length(leftLocCurrent);   % 图像采样时刻数
% 存储计算的摄像机三维坐标结果
featureCPosCurrent = cell(1,timeNum);
featureCPosNext = cell(1,timeNum);
%% 相机标定的10个参数

cameraCalib = visualInputData.calibData ;
T = cameraCalib.T;  % mm为单位，列存储
alpha_c_left = cameraCalib.alpha_c_left;
alpha_c_right = cameraCalib.alpha_c_right;
cc_left = cameraCalib.cc_left;
cc_right = cameraCalib.cc_right;
fc_left = cameraCalib.fc_left;
fc_right = cameraCalib.fc_right;
kc_left = cameraCalib.kc_left;
kc_right = cameraCalib.kc_right;
om = cameraCalib.om;
%% 视觉图像的特征点信息

Rot = zeros(3,3,timeNum);  % whole rotation 
Trs = zeros(3,1,timeNum);  % whole translation
Rbb = zeros(3,3,timeNum);
Tbb = zeros(3,timeNum);
sm = 100;   % the number of Monte Carlo sample
q = 3;   % the number of matching point for each sample
Rcc_sm = zeros(3,3,sm);
Tcc_sm = zeros(3,1,sm);
Median = zeros(1,sm);
S = diag([1,1,-1]);
spixel = cell(1,timeNum);

% navigation parameter in world frame
% VOsta = zeros(3,timeNum+1);
% VOpos = zeros(3,timeNum+1);
% VOvel = zeros(3,timeNum+1);
% VOsta(:,1) = [0;0;0] ;  % 初始位置:以初始时刻左摄像机坐标系为原点
% VOpos(:,1) = initialAttitude_r;   %初始姿态
% VOvel(:,1) = initialVelocity_r;    % 初始速度

Rk = zeros(6,6,timeNum); % 误差协方差
RELMOV = zeros(7,timeNum);
qRk = zeros(7,7,timeNum); % 误差协方差

% Cbr=FCbn(initialAttitude_r);
% Crb=Cbr';
% VOfre = frequency_VO;  % Hz
% install information

cameraSettingAngle=  visualInputData.cameraSettingAngle;

if exist('Rbb_temp.mat','file')
   load('Rbb_temp.mat') 
   load('Tbb_temp.mat') 
else
    Rbb_temp = zeros(3,3,timeNum);
    Tbb_temp = zeros(3,timeNum);
end

Cbb1 = FCbn(cameraSettingAngle)';
Cb1c = [1, 0, 0;     % 本体系到摄像机坐标系:绕x轴转动-90度
       0, 0,-1;     % 摄像机坐标系c： x和y在相机平面，y向下，x向右，z向前
       0, 1, 0];    % 本体系b：x向右，y向前，z向上
Cbc = Cb1c*Cbb1 ;
Ccb = Cbc';

% 显示进度条
% h = waitbar(0,'从匹配特征点计算RbbTbb中...');
steps = timeNum;

for i = 1:timeNum
   %% 三维重建
   %% 判断是否更新
   isupdate=1;
   for ri=1:3
      for rj=1:3
          if Rbb_temp(ri,rj,i)~=0
              isupdate=0;
          end
      end
   end
   for ti=1:3
      if  Tbb_temp(ti,i)~=0
          isupdate=0;
      end
   end
   if isupdate==0
       Rbb(:,:,i)=Rbb_temp(:,:,i);
       Tbb(:,i)=Tbb_temp(:,i);
       disp([num2str(i),'时刻提取现成，不计算特征点'])
       continue;
   end
   %%
   % Three-dimension restruction to get dots' position in world coordinate 
   P1 = zeros(matchedNum(i),3);    % store position information in previous time
   P2 = zeros(matchedNum(i),3);    % store position information in present time
   N = matchedNum(i);    % the number of features
%    for j = 1:N
%        P1(j,1) = B * (visualInputData{i}.leftLocCurrent(j,2) - u0) / (visualInputData{i}.leftLocCurrent(j,2) - visualInputData{i}.rightLocCurrent(j,2));               % X
%        P1(j,2) = B * ax * (visualInputData{i}.leftLocCurrent(j,1) - v0) / (ay * (visualInputData{i}.leftLocCurrent(j,2) - visualInputData{i}.rightLocCurrent(j,2)));   % Y
%        P1(j,3) = B * ax / (visualInputData{i}.leftLocCurrent(j,2) - visualInputData{i}.rightLocCurrent(j,2));                                       % Z
%        P2(j,1) = B * (visualInputData{i}.leftLocNext(j,2) - u0) / (visualInputData{i}.leftLocNext(j,2) - visualInputData{i}.rightLocNext(j,2));               % X
%        P2(j,2) = B * ax * (visualInputData{i}.leftLocNext(j,1) - v0) / (ay * (visualInputData{i}.leftLocNext(j,2) - visualInputData{i}.rightLocNext(j,2)));   % Y
%        P2(j,3) = B * ax / (visualInputData{i}.leftLocNext(j,2) - visualInputData{i}.rightLocNext(j,2));                                       % Z
%    end
    
    for j = 1:N
%         P1(j,3) = fl * (fr*Tlr(1) - (visualInputData{i}.rightLocCurrent(j,2) - ru0)*Tlr(3)) / ((visualInputData{i}.rightLocCurrent(j,2) - ru0)*(Rlr(3,1)*(visualInputData{i}.leftLocCurrent(j,2) - lu0)...
%                 + Rlr(3,2)*(visualInputData{i}.leftLocCurrent(j,1) - lv0) + fl*Rlr(3,3)) - fr*(Rlr(1,1)*(visualInputData{i}.leftLocCurrent(j,2) - lu0) + Rlr(1,2)*(visualInputData{i}.leftLocCurrent(j,1) - lv0) + fl*Rlr(1,3)));
%         P1(j,3) = (frx*Tlr(1) - (visualInputData{i}.rightLocCurrent(j,2) - ru0)*Tlr(3)) / ((visualInputData{i}.rightLocCurrent(j,2) - ru0)*(Rlr(3,1)*(visualInputData{i}.leftLocCurrent(j,2) - lu0)/flx...
%                   + Rlr(3,2)*(visualInputData{i}.leftLocCurrent(j,1) - lv0)/fly + Rlr(3,3)) - frx*(Rlr(1,1)*(visualInputData{i}.leftLocCurrent(j,2) - lu0)/flx + Rlr(1,2)*(visualInputData{i}.leftLocCurrent(j,1) - lv0)/fly + Rlr(1,3)));
%         P1(j,1) = P1(j,3) * (visualInputData{i}.leftLocCurrent(j,2) - lu0) / flx;
%         P1(j,2) = P1(j,3) * (visualInputData{i}.leftLocCurrent(j,1) -
%         lv0) / fly;
          xL = [leftLocCurrent{i}(j,2);leftLocCurrent{i}(j,1)]; % 第i个时刻的第j个当前帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
          xR = [rightLocCurrent{i}(j,2);rightLocCurrent{i}(j,1)];
          
          [P1(j,:),~] = stereo_triangulation(xL,xR,om,T'/1000,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right);
          % 得到当前特征点的左摄像机三维坐标
%         P2(j,3) = fl * (fr*Tlr(1) - (visualInputData{i}.rightLocNext(j,2) - ru0)*Tlr(3)) / ((visualInputData{i}.rightLocNext(j,2) - ru0)*(Rlr(3,1)*(visualInputData{i}.leftLocNext(j,2) - lu0)...
%                 + Rlr(3,2)*(visualInputData{i}.leftLocNext(j,1) - lv0) + fl*Rlr(3,3)) - fr*(Rlr(1,1)*(visualInputData{i}.leftLocNext(j,2) - lu0) + Rlr(1,2)*(visualInputData{i}.leftLocNext(j,1) - lv0) + fl*Rlr(1,3)));
%         P2(j,3) = (frx*Tlr(1) - (visualInputData{i}.rightLocNext(j,2) - ru0)*Tlr(3)) / ((visualInputData{i}.rightLocNext(j,2) - ru0)*(Rlr(3,1)*(visualInputData{i}.leftLocNext(j,2) - lu0)/flx...
%                   + Rlr(3,2)*(visualInputData{i}.leftLocNext(j,1) - lv0)/fly + Rlr(3,3)) - frx*(Rlr(1,1)*(visualInputData{i}.leftLocNext(j,2) - lu0)/flx + Rlr(1,2)*(visualInputData{i}.leftLocNext(j,1) - lv0)/fly + Rlr(1,3)));
%         P2(j,1) = P2(j,3) * (visualInputData{i}.leftLocNext(j,2) - lu0) / flx;
%         P2(j,2) = P2(j,3) * (visualInputData{i}.leftLocNext(j,1) - lv0) / fly;
          xL = [leftLocNext{i}(j,2);leftLocNext{i}(j,1)]; % 第i个时刻的第j个下一帧帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
          xR = [rightLocNext{i}(j,2);rightLocNext{i}(j,1)];
          [P2(j,:),~] = stereo_triangulation(xL,xR,om,T'/1000,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right);
          % 得到（与以上当前时刻特征点匹配的）下一时刻特征点的左摄像机三维坐标
    end
    featureCPosCurrent{i} = P1;
    featureCPosNext{i} = P2;
    %% 运动估计
   % Motion estimation to get coordinate translate matrix: LMedS
   for j = 1:sm
       ind = randi(N,1,q);
       % SVD method
       M0 = zeros(3,1);
       M1 = zeros(3,1);
       for k = 1:q
           M0 = M0 + P1(ind(k),:)';
           M1 = M1 + P2(ind(k),:)';
       end
       M0 = M0 / q;
       M1 = M1 / q;
       Pset0 = zeros(3,q);
       Pset1 = zeros(3,q);
       for k = 1:q
           Pset0(:,k) = P1(ind(k),:)' - M0;
           Pset1(:,k) = P2(ind(k),:)' - M1;
       end
       Q = Pset1*Pset0'/q;
       [U,~,V] = svd(Q);
       if abs(det(U)*det(V)-1) < 1e-10
           Rcc = U*V';
       elseif abs(det(U)*det(V)+1) < 1e-10
           Rcc = U*S*V';
       end
       
    %    Tcc = M1 - Rcc * M0;
      Tcc =- M1 + Rcc * M0;
       
       Rcc_sm(:,:,j) = Rcc;
       Tcc_sm(:,:,j) = Tcc;
       % compute regression variance and find Median
       r = zeros(1,N);
       for k = 1:N
           r(k) = norm(P2(k,:)' - (Rcc * P1(k,:)' + Tcc));
       end
% %        rr = isnan(r);
% %        indexr =  rr == 1;
% %        r(indexr) = Inf;
       Median(j) = median(r);
   end
   
   % find the minimum Median
   mMed = min(Median);
   ord = find( Median == min(Median));
   Rcc = Rcc_sm(:,:,ord(1));
   Tcc = Tcc_sm(:,:,ord(1));
   
   % compute robust standrad deviation
   sigma = 1.4826 * (1 + 5 / (N - q)) * sqrt(mMed);
   % exstract matching point
   P1new = zeros(3,matchedNum(i));
   P2new = zeros(3,matchedNum(i));
   leftLocCurrentNew = zeros(matchedNum(i),2);
   rightLocCurrentNew = zeros(matchedNum(i),2);
   leftLocNextNew = zeros(matchedNum(i),2);
   rightLocNextNew = zeros(matchedNum(i),2);
   enum = 0;
   for j = 1:N
       res = norm(P2(j,:)' - (Rcc * P1(j,:)' + Tcc));
       if res ^ 2 <= (2.5 * sigma) ^ 2
           enum = enum + 1;
           P1new(:,enum) = P1(j,:)';
           P2new(:,enum) = P2(j,:)';
           leftLocCurrentNew(enum,:) = leftLocCurrent{i}(j,:);
           rightLocCurrentNew(enum,:) = rightLocCurrent{i}(j,:);
           leftLocNextNew(enum,:) = leftLocNext{i}(j,:);
           rightLocNextNew(enum,:) = rightLocNext{i}(j,:);
       end
   end
   % 选取残差最小的20个点
%    res = zeros(1,N);
%    for j = 1:N
%        res(j) = norm(P2(j,:)' - (Rcc * P1(j,:)' + Tcc));
%    end
%    [vals,indx] = sort(res);
%    for enum = 1:20
%        P1new(:,enum) = P1(indx(enum),:)';
%        P2new(:,enum) = P2(indx(enum),:)';
%        leftLocCurrent(enum,:) = visualInputData{i}.leftLocCurrent(indx(enum),:);
%        rightLocCurrent(enum,:) = visualInputData{i}.rightLocCurrent(indx(enum),:);
%        leftLocNext(enum,:) = visualInputData{i}.leftLocNext(indx(enum),:);
%        rightLocNext(enum,:) = visualInputData{i}.rightLocNext(indx(enum),:);
%    end
   P1new(:,enum+1:N) = [];
   P2new(:,enum+1:N) = [];
   leftLocCurrentNew(enum+1:N,:) = [];
   rightLocCurrentNew(enum+1:N,:) = [];
   leftLocNextNew(enum+1:N,:) = [];
   rightLocNextNew(enum+1:N,:) = [];
   spixel{i}.leftLocCurrent = leftLocCurrentNew;
   spixel{i}.rightLocCurrent = rightLocCurrentNew;
   spixel{i}.leftLocNext = leftLocNextNew;
   spixel{i}.rightLocNext = rightLocNextNew;
   % SVD method to get the final motion estimation (R,T)
   M0 = zeros(3,1);
   M1 = zeros(3,1);
   for k = 1:enum
       M0 = M0 + P1new(:,k);
       M1 = M1 + P2new(:,k);
   end
   M0 = M0 / enum;
   M1 = M1 / enum;
   Pset0 = zeros(3,enum);
   Pset1 = zeros(3,enum);
   for k = 1:enum
       Pset0(:,k) = P1new(:,k) - M0;
       Pset1(:,k) = P2new(:,k) - M1;
   end
   Q = Pset1*Pset0'/enum;
   [U,D,V] = svd(Q);
   if abs(det(U)*det(V)-1) < 1e-10
       Rcc = U*V';
   elseif abs(det(U)*det(V)+1) < 1e-10
       Rcc = U*S*V';
   end
  Tcc = - M1 + Rcc * M0;
 %   Tcc = M1 - Rcc * M0;
   % 根据姿态矩阵Rcc计算姿态四元数
   q1=1/2*sqrt(abs(1+Rcc(1,1)-Rcc(2,2)-Rcc(3,3)));
   q2=1/2*sqrt(abs(1-Rcc(1,1)+Rcc(2,2)-Rcc(3,3)));
   q3=1/2*sqrt(abs(1-Rcc(1,1)-Rcc(2,2)+Rcc(3,3)));
   q0=sqrt(abs(1-q1^2-q2^2-q3^2));
   if Rcc(2,3)-Rcc(3,2)<0
       q1=-q1;
   end
   if Rcc(3,1)-Rcc(1,3)<0
       q2=-q2;
   end
   if Rcc(1,2)-Rcc(2,1)<0
       q3=-q3;
   end
   Q0=[q0;q1;q2;q3];
   Q0=Q0/norm(Q0);
   X = LMalgorithm1(P2new,P1new,Q0,-Tcc);
   
   Rcc = [X(1)^2+X(2)^2-X(3)^2-X(4)^2,    2*(X(2)*X(3)+X(1)*X(4)),        2*(X(2)*X(4)-X(1)*X(3));
         2*(X(2)*X(3)-X(1)*X(4)),    X(1)*X(1)-X(2)*X(2)+X(3)*X(3)-X(4)*X(4),    2*(X(3)*X(4)+X(1)*X(2));
         2*(X(2)*X(4)+X(1)*X(3)),        2*(X(3)*X(4)-X(1)*X(2)),    X(1)*X(1)-X(2)*X(2)-X(3)*X(3)+X(4)*X(4)];
   Tcc = -X(5:7);
   Rot(:,:,i) = Rcc;
   Trs(:,:,i) = Tcc;
   % 计算重投影误差目标函数的Jacobi矩阵
   % 以计算相对运动参数的误差协方差矩阵
   Rbb(:,:,i) = Ccb * Rcc * Cbc; % Rbb
   Tbb(:,i) = Ccb * Tcc;
   % 计算量测噪声方差阵
    % 计算姿态角
    pos(1) = asin(Rbb(2,3,i));  % 俯仰角  
    if Rbb(3,3,i)>0
        pos(2)=atan(-Rbb(1,3,i)/Rbb(3,3,i)); % roll
    elseif Rbb(3,3,i)<0
        if Rbb(1,3,i)>0
            pos(2)=pos(2)-pi;
        else
            pos(2)=pos(2)+pi;
        end
    elseif Rbb(3,3,i)==0
        if Rbb(1,3,i)>0
            pos(2)=-pi/2;
        else
            pos(2)=1/2*pi;
        end
    end
    if Rbb(2,2,i)>0   % 航向角
        if Rbb(2,1,i)>=0
            pos(3) = atan(-Rbb(2,1,i)/Rbb(2,2,i)); % + 2 * pi
        elseif Rbb(2,1,i)<0
            pos(3) = atan(-Rbb(2,1,i)/Rbb(2,2,i));
        end
    elseif Rbb(2,2,i)<0
        pos(3) = pi + atan(-Rbb(2,1,i)/Rbb(2,2,i));
    elseif Rbb(2,2,i)==0
        if Rbb(2,1,i)>0
            pos(3) = 1.5 * pi;
        elseif Rbb(2,1)<0
            pos(3) = pi / 2;
        end
    end
%     Rk(:,:,i) = R_covEuler(P1new,pos);
    Rk(:,:,i) = R_covEuler1(P2new,P1new,Rbb(:,:,i),pos,Tbb(:,i));

   % 计算量测噪声方差阵
   % 根据姿态矩阵Rbb计算姿态四元数
   q1=1/2*sqrt(abs(1+Rbb(1,1,i)-Rbb(2,2,i)-Rbb(3,3,i)));
   q2=1/2*sqrt(abs(1-Rbb(1,1,i)+Rbb(2,2,i)-Rbb(3,3,i)));
   q3=1/2*sqrt(abs(1-Rbb(1,1,i)-Rbb(2,2,i)+Rbb(3,3,i)));
   q0=sqrt(abs(1-q1^2-q2^2-q3^2));
   if Rbb(2,3,i)-Rbb(3,2,i)<0
       q1=-q1;
   end
   if Rbb(3,1,i)-Rbb(1,3,i)<0
       q2=-q2;
   end
   if Rbb(1,2,i)-Rbb(2,1,i)<0
       q3=-q3;
   end
   Q0=[q0;q1;q2;q3];
   Q0=Q0/norm(Q0);
%    Rk(:,:,i) = R_cov1(P1new,Q0);
   qRk(:,:,i) = R_cov2(P2new,P1new,Q0,Rbb(:,:,i),Tbb(:,i));
   RELMOV(:,i) = [Q0;Tbb(:,i)];
   
   if mod(i,5)==0
       i
       Rbb_temp=Rbb;
       Tbb_temp=Tbb;
       save Rbb_temp Rbb_temp
       save Tbb_temp Tbb_temp
      save i i
   end
%    if mod(i,ceil(steps/30))==0
%         waitbar(i/steps,h);
%    end
end
% close(h);

VisualRT.Rbb = Rbb ;
VisualRT.Tbb = Tbb ;

visualInputData.VisualRT = VisualRT;
visualInputData.matchedNum = matchedNum;
visualInputData.featureCPosCurrent = featureCPosCurrent;
visualInputData.featureCPosNext = featureCPosNext;

save visualInputData visualInputData
