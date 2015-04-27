%% 从地理系的姿态 到 世界系（初始时刻地理系）的姿态
% rad 俯仰、横滚、航向 （方向定义见程序说明文档）
function attitude = attitude_t_to_attitude_w(attitude_t,lat_lon_alt) 
N = length(attitude_t);
attitude = zeros(3,N);

Cew = FCen(lat_lon_alt(1,1),lat_lon_alt(2,1));
for k=1:N
   Cet = FCen(lat_lon_alt(1,k),lat_lon_alt(2,k));
  % Cet = Cew ;
   Cwt = Cet*Cew' ;
   Ctb = FCbn(attitude_t(:,k))';
   Cwb = Ctb*Cwt ;
   
   opintions.headingScope = 180;
   attitude(:,k) = GetAttitude(Cwb,'rad',opintions);
end