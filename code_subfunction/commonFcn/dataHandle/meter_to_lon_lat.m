%% 世界系->大地经纬系
% 米——>经纬度

function station_vns_lon_lat = meter_to_lon_lat(station_vns_w_m,lon_lat_start,planet) 

% 世界系 -> 大地直角系
N = size(station_vns_w_m,2);
station_vns_e_m = zeros(3,N) ;
e_start = FJWtoZJ(lon_lat_start,planet);
Cwe = FCen(lon_lat_start(1,1),lon_lat_start(2,1))';
for k=1:N
    station_vns_e_m(:,k) = e_start + Cwe*station_vns_w_m(:,k) ;    
end

% 大地直角系 -> 大地经纬系
station_vns_lon_lat = zeros(3,N);
for k=1:N
    station_vns_lon_lat(:,k) = FZJtoJW(station_vns_e_m(:,k),planet);
end
