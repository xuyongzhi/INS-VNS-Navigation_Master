
planet='m';

load trace4_10n
% poser stationr(经;纬;高)
N = size(poser,2);

% 将大地经纬坐标 转换到 直角坐标
station_e = zeros(3,N);
for k=1:N
    station_e(:,k) = FJWtoZJ(stationr(:,k),planet);
end

% 将大地直角坐标 转到 初始时刻坐标系
station_w = zeros(3,N);
Cew = FCen(stationr(1,1),stationr(2,1));
for k=1:N
    station_w(:,k) = Cew*( station_e(:,k)-station_e(:,1) );
end

time = (1:N)/10;

trueTrace.position = station_w;
trueTrace.attitude = poser ;
trueTrace.initialAttitude_r = poser(:,1);
trueTrace.initialVelocity_r = [0.02;0;0];
trueTrace.initialPosition_e = stationr(:,1);
trueTrace.planet='m';

% velocity_true  = trueTrace.velocity  ;
trueTrace.frequency = 10;
figure
plot(station_w(1,:),station_w(2,:))
saveas(gcf,'position')
figure
plot(time,poser)
saveas(gcf,'attitude')
save trueTrace trueTrace
disp('trueTrace ok')
