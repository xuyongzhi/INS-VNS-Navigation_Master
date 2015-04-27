%% 导航误差：平移和旋转误差
% 参考 kitti 的评价
% rTerror: 相对平移矢量误差(%)  rAerror：相对旋转角度误差（deg/m）  stepLength：计算点的行程
% nav_method='ins'/'vns'/'int'
function [errorRTStr,gh,rTerror,rAerror,pathLength] = CalError_TR( position_true,attitude_true,position_nav,attitude_nav,step,nav_method,dataName )

% step = 100 ;        %  位置姿态误差评价指标 计算 步长 m
 rTbais = 'T相对于路程';
%rTbais = 'T相对于距离';
rAbais = 'R相对于路程';
% rAbais = 'R相对于距离';

%  method='逐段计算';  % 则每个T 和R为 INS_VNS
method = '从原点算'; % 每个T即 position

pos = [position_nav;attitude_nav];
truePos = [position_true;attitude_true];
[T,R,trueT,trueR,stepLength] = PosToTR(truePos,pos,step,method) ;   % 计算导航解算的 T R
N = size(T,2);
if strcmp(method,'逐段计算') 
    pathLength = zeros(1,N);
    pathLength(1) = stepLength(1);
    for k=2:N
        pathLength(k) = pathLength(k-1)+stepLength(k);
    end
else
    pathLength=stepLength;
end
% 求绝对误差　
Terror = T-trueT ;              % 得到绝对的 转移 矢量 误差 

EulerAngleError = zeros(3,N);   % 三个欧拉角误差
RotAngleError = zeros(1,N);     % 整体三维旋转角误差 （四元数形式得到）
opintions.headingScope = 180 ;
for k=1:N
    dR=R(:,:,k)*trueR(:,:,k)';
    EulerAngleError(:,k) = GetAttitude(dR,'rad',opintions) ;
    dQ = FCnbtoQ(dR) ;
    RotAngleError(k)=2*acos(dQ(1));
end
% 求相对误差
rTerror = zeros(5,N) ;  % 第4行是二维平移相对误差 第5行是三维维平移相对误差
L = zeros(1,N);
for k=1:N
   if strcmp(method,'逐段计算') 
       % 逐段解算
       if strcmp(rTbais,'T相对于路程')
            L(k) = stepLength(k) ;        % 相对于 路程
        else
            L(k) = GetLength(trueT(:,k));  % 相对于直线距离
        end
   else
       % 从原点算
       if strcmp(rTbais,'T相对于路程')
            L(k) = stepLength(k) ;        % 相对于 路程
        else
            L(k) = GetLength(trueT(:,k));  % 相对于直线距离
        end
   end
end
for k=1:N
    
    rTerror(1:3,k) = abs(Terror(1:3,k))/L(k) ;       % 取绝对值
    rTerror(4,k) = GetLength(Terror(1:2,k))/L(k) ;
    rTerror(5,k) = GetLength(Terror(1:3,k))/L(k) ;
end
rAerror = zeros(4,N);   % 第4行是三维旋转相对误差
% 角度的相对误差： 角度误差/真实路程 deg/m
for k=1:N
    rAerror(1:3,k) = EulerAngleError(:,k)/L(k)*180/pi ;
    rAerror(4,k) = RotAngleError(k)/L(k)*180/pi ;
end

%% 输出结果
% 平均平移误差
ave_rTerror = mean(rTerror,2)*100 ;
Terror_str = sprintf('[%s] step=%dm 时的平均 平移相对误差 绝对值（去符号）(%s)(%s)：\n',nav_method,step,rTbais,method );
Terror_str = sprintf('%s\tx,y,z方向相对误差：%0.5g, %0.5g, %0.5g (%%)\n',Terror_str,ave_rTerror(1),ave_rTerror(2),ave_rTerror(3));
Terror_str = sprintf('%s\t二维平面：%0.5g%%\t三维平面：%0.5g%%',Terror_str,ave_rTerror(4),ave_rTerror(5));
%平均旋转角误差
% ave_rAerror = mean(abs(rAerror),2) ;
% Aerror_str = sprintf('[%s] step=%dm 时的平均 旋转角误差 绝对值（去符号）(%s)(%s)：\n',nav_method,step,rAbais,method );
% Aerror_str = sprintf('%s\t俯仰，横滚，偏航方向：%0.5g, %0.5g, %0.5g (deg/m)\n',Aerror_str,ave_rAerror(1),ave_rAerror(2),ave_rAerror(3));
% Aerror_str = sprintf('%s\t三维旋转角：%0.5g deg/m ',Aerror_str,ave_rAerror(4) );

ave_rAerror = mean(abs(rAerror),2)*3600 ;
Aerror_str = sprintf('[%s] step=%dm 时的平均 旋转角误差 绝对值（去符号）(%s)(%s)：\n',nav_method,step,rAbais,method );
Aerror_str = sprintf('%s\t俯仰，横滚，偏航方向：%0.5g, %0.5g, %0.5g (″/m)\n',Aerror_str,ave_rAerror(1),ave_rAerror(2),ave_rAerror(3));
Aerror_str = sprintf('%s\t三维旋转角：%0.5g ″/m ',Aerror_str,ave_rAerror(4) );

errorRTStr = sprintf('%s\n%s',Terror_str,Aerror_str);
display(errorRTStr);

lineWidth=2.5;
labelFontSize = 16;
axesFontsize = 13;
maker='-s';

ghk=1;
gh(ghk)=figure;
set(cla,'fontsize',axesFontsize)
plot(pathLength,rTerror(1:3,:)*100,maker,'linewidth',lineWidth)
title([nav_method,' Tanslation error'],'fontsize',labelFontSize)
xlabel('Path length (m)','fontsize',labelFontSize)
ylabel('Tanslation error (%)','fontsize',labelFontSize)
legend('x','y','z');
saveas(gcf,[dataName,'\',nav_method,' Tanslation error xyz.fig'])

ghk=ghk+1;
gh(ghk)=figure;
set(cla,'fontsize',axesFontsize)
plot(pathLength,rTerror(5,:)*100,maker,'linewidth',lineWidth)
title([nav_method,' Tanslation error'],'fontsize',labelFontSize)
xlabel('Path length (m)','fontsize',labelFontSize)
ylabel('Tanslation error (%)','fontsize',labelFontSize)
saveas(gcf,[dataName,'\',nav_method,' Tanslation error 3D.fig'])

% ghk=ghk+1;
% gh(ghk)=figure;
% set(cla,'fontsize',axesFontsize)
% plot(pathLength,rTerror(4,:)*100,maker,'linewidth',lineWidth)
% title([nav_method,'Tanslation error'],'fontsize',labelFontSize)
% xlabel('Path length (m)','fontsize',labelFontSize)
% ylabel('Tanslation error (%)','fontsize',labelFontSize)
% saveas(gcf,[dataName,'\',nav_method,' Tanslation error 2D.fig'])

ghk=ghk+1;
gh(ghk)=figure;
set(cla,'fontsize',axesFontsize)
plot(pathLength,rAerror(4,:),maker,'linewidth',lineWidth)
title([nav_method,' Rotation error'],'fontsize',labelFontSize)
xlabel('Path length (m)','fontsize',labelFontSize)
ylabel('Rotation error (deg/m)','fontsize',labelFontSize)
saveas(gcf,[dataName,'\',nav_method,' Rotation error 3D.fig'])

ghk=ghk+1;
gh(ghk)=figure;
set(cla,'fontsize',axesFontsize)
plot(pathLength,abs(rAerror(1:3,:)),maker,'linewidth',lineWidth)
title([nav_method,' Rotation error'],'fontsize',labelFontSize)
xlabel('Path length (m)','fontsize',labelFontSize)
ylabel('Rotation error (deg/m)','fontsize',labelFontSize)
legend('pitch','roll','head');
saveas(gcf,[dataName,'\',nav_method,' Rotation error.fig'])

% ghk=ghk+1;
% gh(ghk)=figure;
% subplot(3,1,1)
% set(cla,'fontsize',axesFontsize)
% plot(pathLength,abs(rAerror(1,:)),maker,'linewidth',lineWidth)
% title([nav_method,' Rotation error(deg/m)'],'fontsize',labelFontSize)
% xlabel('Path length (m)','fontsize',labelFontSize)
% ylabel('pitch error','fontsize',labelFontSize)
% subplot(3,1,2)
% set(cla,'fontsize',axesFontsize)
% plot(pathLength,abs(rAerror(2,:)),maker,'linewidth',lineWidth)
% xlabel('Path length (m)','fontsize',labelFontSize)
% ylabel('roll error','fontsize',labelFontSize)
% subplot(3,1,3)
% set(cla,'fontsize',axesFontsize)
% plot(pathLength,abs(rAerror(3,:)),maker,'linewidth',lineWidth)
% xlabel('Path length (m)','fontsize',labelFontSize)
% ylabel('head error','fontsize',labelFontSize)
% saveas(gcf,[dataName,'\',nav_method,' Rotation error xyz.fig'])

function TL=GetLength(T)
if length(T)==2
    TL=sqrt(T(1)^2+T(2)^2);
elseif length(T)==3
    TL=sqrt(T(1)^2+T(2)^2+T(3)^2);
else
    TL=nan;
end

%% 导航位置 姿态结果->转移和旋转角
% 每 step m 计算一次
% pos =[位置;姿态]
% stepLength: 实际计算的路程步长
%  method='逐段计算' 则每个T 和R为 逐段计算
%  method = '从原点算' 每个T即 position
function [T,R,trueT,trueR,stepLength] = PosToTR(truePos,pos,step,method)
N = length(pos);
R = zeros(3,3,N);
T = zeros(3,N);
trueR = zeros(3,3,N);
trueT = zeros(3,N);
stepLength = zeros(1,N);
[~,routeLength,~] = CalRouteLength( truePos );
if step>routeLength(length(routeLength)-1)
    step=routeLength(length(routeLength)-1) ;
end
k_sr_last = 1 ; 
k_TR = 0;   % 计算的T R 序号
for k_sr=2:N
   if  routeLength(k_sr)-routeLength(k_sr_last) >= step 
       if routeLength(N)-routeLength(k_sr_last)<2*step && k_sr~=N
           % 避免最后一段不满足 step 时不被计算
          continue;  
       end       
       
        % 往前走 step m 就计算一次 转移和旋转
        k_TR = k_TR+1 ;
        if strcmp(method,'逐段计算')
            base_k = k_sr_last ;
        else
            base_k = 1;
        end
        stepLength(k_TR) = routeLength(k_sr)-routeLength(base_k) ;
      	trueT(:,k_TR) = truePos(1:3,k_sr)-truePos(1:3,base_k);
        T(:,k_TR) = pos(1:3,k_sr)-pos(1:3,base_k);
        R1 = pos(4:6,k_sr);
        R2 = pos(4:6,base_k);
        R(:,:,k_TR) = FCbn(R1)' * FCbn(R2) ;
        R1 = truePos(4:6,k_sr);
        R2 = truePos(4:6,base_k);
        trueR(:,:,k_TR) = FCbn(R1)' * FCbn(R2) ;

        k_sr_last = k_sr ;
   end
end
T = T(:,1:k_TR) ;
trueT = trueT(:,1:k_TR) ;
R = R(:,:,1:k_TR) ;
trueR = trueR(:,:,1:k_TR) ;
stepLength = stepLength(1:k_TR);