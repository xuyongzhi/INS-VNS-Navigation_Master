%% IMU 反推
% 由 姿态和速度 微分 得到近似的IMU数据中间值
% 载入 trueTrace 并把计算真实的IMU的中间参数“dif_wrbb,dif_arbr”保存到 该trueTrae
%   执行完此函数后 再执行 newGetTrueTrace 可得到 真实IMU

function [trueTrace,dif_wrbb,dif_arbr] = get_trueIMU_wr(trueTrace)

% close all
if ~exist('trueTrace','var')
    raw_data_dir = 'E:\惯性视觉导航\NAVIGATION\data_kitVO\';
    pro_name = '2011_09_30_drive_0028';
    prodir = [raw_data_dir,pro_name];
    trueTrace = importdata([prodir,'\trueTrace.mat']);
else
    prodir = pwd;
end

position_w = trueTrace.position ;
attitude_w = trueTrace.attitude ;
velocity_w = trueTrace.velocity ;
runTime = trueTrace.runTime_IMU ;

stepTime = runTime_to_setpTime(runTime) ;

dif_wrbb = get_differential(attitude_w,runTime) ;% 微分得到的角速度 wrbb
dif_arbr = get_differential(velocity_w,runTime) ;% 微分得到的加速度 arbr

trueTrace.dif_wrbb = dif_wrbb ;
trueTrace.dif_arbr = dif_arbr ;

save([prodir,'\dif_wrbb'],'dif_wrbb')
save([prodir,'\dif_arbr'],'dif_arbr')
save([prodir,'\trueTrace'],'trueTrace')
%% 对比 微分得到的速度与实际的速度
dif_velocity_w = get_differential(position_w,runTime) ;% 微分得到的速度
N_dif = length(dif_velocity_w) ;
figure
hold on
plot(runTime(1:N_dif)',dif_velocity_w(1,:),'r')
plot(runTime',velocity_w(1,:),'b')
legend('dif_v','v')
% saveas(gcf,'v1')

figure
hold on
plot(runTime(1:N_dif)',dif_velocity_w(2,:),'r')
plot(runTime',velocity_w(2,:),'b')
legend('dif_v','v')
% saveas(gcf,'v2')

figure
hold on
plot(runTime(1:N_dif)',dif_velocity_w(3,:),'r')
plot(runTime',velocity_w(3,:),'b')
legend('dif_v','v')
% saveas(gcf,'v3')
%% 对比 微分得到的姿态与重新积分的姿态
attitude_w_from_dif = zeros(size(attitude_w)) ;
attitude_w_from_dif(:,1) = attitude_w(:,1);
for k=2:length(attitude_w)
    attitude_w_from_dif(:,k) = attitude_w_from_dif(:,k-1)+dif_wrbb(:,k-1)*stepTime(k-1);
end
figure
hold on
plot(runTime',attitude_w_from_dif(1,:),'r')
plot(runTime',attitude_w(1,:),'b')
legend('from\_dif\_pitch','true\_pitch')
% saveas(gcf,'v1')

figure
hold on
plot(runTime',attitude_w_from_dif(2,:),'r')
plot(runTime',attitude_w(2,:),'b')
legend('from\_dif\_roll','true\_roll')
% saveas(gcf,'v2')

figure
hold on
plot(runTime',attitude_w_from_dif(3,:),'r')
plot(runTime',attitude_w(3,:),'b')
legend('from\_dif\_yaw','true\_yaw')
% saveas(gcf,'v3')
%% 对比 微分得到的速度与重新积分的速度
velocity_w_from_dif = zeros(size(velocity_w)) ;
velocity_w_from_dif(:,1) = velocity_w(:,1);
for k=2:length(attitude_w)
    velocity_w_from_dif(:,k) = velocity_w_from_dif(:,k-1)+dif_arbr(:,k-1)*stepTime(k-1);
end
figure
hold on
plot(runTime',velocity_w_from_dif(1,:),'r')
plot(runTime',velocity_w(1,:),'b')
legend('from\_dif\_vx','true\_vx')
% saveas(gcf,'v1')

figure
hold on
plot(runTime',velocity_w_from_dif(2,:),'r')
plot(runTime',velocity_w(2,:),'b')
legend('from\_dif\_vy','true\_vy')
% saveas(gcf,'v2')

figure
hold on
plot(runTime',velocity_w_from_dif(3,:),'r')
plot(runTime',velocity_w(3,:),'b')
legend('from\_dif\_vz','true\_vz')
% saveas(gcf,'v3')

disp('')

%% 取数值微分
% 取每个点附近的5个点进行2次多项式拟合，取拟合曲线在该点的斜率为微分值
function difData = get_differential(data,runTime)

fitNum = 13;         % data 拟合步长
fitDegree = 4;      % data 拟合阶次
smoothSpan = 5;     % difData 平滑步长
method = 'rloess'; % difData 平滑阶次 2
%method = 'rlowess'; % difData 平滑阶次 1

smoothSpan_data = 50 ;
method_data = 'sgolay' ;
degree_data = 3 ;
smooth_data = zeros(size(data)) ;
smooth_data(1,:) = smooth(data(1,:),smoothSpan_data,method_data,degree_data);
smooth_data(2,:) = smooth(data(2,:),smoothSpan_data,method_data,degree_data);
smooth_data(3,:) = smooth(data(3,:),smoothSpan_data,method_data,degree_data);

%% 对比 平滑前后的 data
% figure
% hold on
% plot(runTime',smooth_data(1,:),'r')
% plot(runTime',data(1,:),'b')
% legend('smooth\_data\_x','data\_x')
% % saveas(gcf,'v1')
% 
% figure
% hold on
% plot(runTime',smooth_data(2,:),'r')
% plot(runTime',data(2,:),'b')
% legend('smooth\_data\_y','data\_y')
% % saveas(gcf,'v2')
% 
% figure
% hold on
% plot(runTime',smooth_data(3,:),'r')
% plot(runTime',data(3,:),'b')
% legend('smooth\_data\_z','data\_z')
% % saveas(gcf,'v3')

data = smooth_data ;
N = length(data) ;
difData = zeros(3,N-1);
P = zeros(3,fitDegree+1,N);        %  data 多项式拟合参数
dif_P = zeros(3,fitDegree,N);    %  difData 多项式 拟合参数
for n=1:N-1
    if n<5 || N-n<5
        fitNum_n = 30 ;
        fitDegree_n = 4 ;
    else
        fitNum_n = fitNum ;
        fitDegree_n = fitDegree ;
    end
   nStart = max(n-2,1) ;
   nEnd = nStart+fitNum_n-1 ;
   if nEnd>N
       nEnd = N ;
       nStart = nEnd-fitNum_n+1 ;
   end
   P(1,:,n) = [zeros(1,fitDegree-fitDegree_n),polyfit(runTime(nStart:nEnd)',data(1,nStart:nEnd),fitDegree_n)];
   P(2,:,n) = [zeros(1,fitDegree-fitDegree_n),polyfit(runTime(nStart:nEnd)',data(2,nStart:nEnd),fitDegree_n)];
   P(3,:,n) = [zeros(1,fitDegree-fitDegree_n),polyfit(runTime(nStart:nEnd)',data(3,nStart:nEnd),fitDegree_n)];
   % 计算 dif_P 
   dif_P(:,:,n) = P(:,1:fitDegree,n) ;
   for i=1:fitDegree
       dif_P(:,i,n) = dif_P(:,i,n)*(fitDegree-i+1) ;
   end
end
for n=1:N-1
    difData(1,n) = polyval(dif_P(1,:,n),runTime(n));
    difData(2,n) = polyval(dif_P(2,:,n),runTime(n));
    difData(3,n) = polyval(dif_P(3,:,n),runTime(n));
end

% % 将 difData进行轻度平滑
% difData(1,:) = smooth(difData(1,:),smoothSpan,method);
% difData(2,:) = smooth(difData(2,:),smoothSpan,method);
% difData(3,:) = smooth(difData(3,:),smoothSpan,method);
