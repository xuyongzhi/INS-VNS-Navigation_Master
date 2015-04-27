% 将 经纬高度 转换为 初始时刻地理直角系（世界坐标系）
% 初始时刻地理直角系: 东北天->x指向初始东,y指向初始北，z指向初始天

function position_w = lon_lat_alt_to_Wxyz(lon_lat_alt,planet)

% poser stationr(经;纬;高)
N = size(lon_lat_alt,2);

% 将大地经纬坐标 转换到 直角坐标
station_e_xyz = zeros(3,N);
for k=1:N
    station_e_xyz(:,k) = FJWtoZJ(lon_lat_alt(:,k),planet);
end

% 将大地直角坐标 转到 初始时刻坐标系
position_w = zeros(3,N);
Cew = FCen(lon_lat_alt(1,1),lon_lat_alt(2,1));
for k=1:N
    position_w(:,k) = Cew*( station_e_xyz(:,k)-station_e_xyz(:,1) );
end
