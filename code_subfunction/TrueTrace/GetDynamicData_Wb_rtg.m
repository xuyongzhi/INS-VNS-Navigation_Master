function [realTimefb,realTimeWb,runTimeNum] = GetDynamicData_Wb_rtg(v_front_const,frequency) 
%% 生成长方形轨迹
% v_front_const ：向前速度
% 100HZ 惯导 0.1HZ采图
% 直线部分 0.03m/s  30cm采图  
%  转弯部分  保持0.03m/s，转弯半径3.5m ->0.4297°/s ,   4.297°+30cm 采图

%   realTimefb 始终 加速度为0

%% 设定长方形各部分大小
rt=1;
Line1 = 60 /rt  ;       % 一条边的长度
Line2 = 30 /rt ;        % 另一条边的长度
w = -0.5*pi/180 *rt ;    % 转弯部分的角速度 w*1

%%
t_Line1 = Line1/v_front_const ;
t_Line2 = Line2/v_front_const ;
t_round = abs(pi/2/w) ;

runTimeNum_Lin1 = fix(t_Line1*frequency) ;
runTimeNum_Lin2 = fix(t_Line2*frequency);
runTimeNum_round = fix(t_round*frequency);
runTimeNum = runTimeNum_Lin1*2+runTimeNum_Lin2*2+runTimeNum_round*4 ;

%%
realTimefb = zeros(3,runTimeNum);

realTimeWb_Line1 = zeros(3,runTimeNum_Lin1);
realTimeWb_Line2 = zeros(3,runTimeNum_Lin2);
realTimeWb_round = zeros(3,runTimeNum_round) ;
realTimeWb_round(3,:) = ones(1,runTimeNum_round)*w ;

realTimeWb = [realTimeWb_Line1 realTimeWb_round realTimeWb_Line2 realTimeWb_round realTimeWb_Line1 realTimeWb_round realTimeWb_Line2 realTimeWb_round];

