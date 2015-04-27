%% 获取动态变化的 加速度 和 姿态变化率
function fb = GetDynamicData_fb_s(runTimeNum,frequency)
% runTimeNum ：采样个数,频率100

fb = zeros(3,runTimeNum);

for k=1:runTimeNum
    fb(1,k) = 0 + 0.05 * sin(2*pi/(60*5*frequency*2)*k) + 0.0005*0 * cos(2*pi/(60*0.5*frequency*2)*k) ;
    fb(2,k) = 0 + 0.03 * sin(2*pi/(60*6*frequency*2)*k) + 0.0005*0 * cos(2*pi/(60*0.5*frequency*2)*k) ;
    fb(3,k) = 0 + 0.0001 * cos(2*pi/(60*4*frequency*2)*k) +0.00003  * cos(2*pi/(60*1*frequency*2)*k);
end
% time = 1:runTimeNum ;
% figure
% h1 = plot(time,data(1,:));
% figure
% h2 = plot(time,data(2,:));
% figure
% h3 = plot(time,data(3,:));