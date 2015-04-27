%% 地理系速度 to 世界系速度
% 暂未考虑科氏项

function  velocity_w = velocity_t_to_velocity_w(velocity_t,lat_lon_alt)
N = length(velocity_t);
velocity_w = zeros(3,N);

Cew = FCen(lat_lon_alt(1,1),lat_lon_alt(2,1));
for k=1:N
   Cet = FCen(lat_lon_alt(1,k),lat_lon_alt(2,k));
   Cwt = Cet*Cew' ;
   velocity_w(:,k) = Cwt' * velocity_t(:,k);
end
