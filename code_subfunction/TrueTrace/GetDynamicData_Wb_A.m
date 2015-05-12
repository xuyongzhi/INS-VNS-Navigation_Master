function [realTimefb,realTimeWb,runTimeNum] = GetDynamicData_Wb_A(v_front_const,frequency) 
%% 生成特殊轨迹A
% v_front_const ：向前速度
% 100HZ 惯导 0.1HZ采图
% 直线部分 0.03m/s  30cm采图  
%  转弯部分  保持0.03m/s，转弯半径3.5m ->0.4297°/s ,   4.297°+30cm 采图

%   realTimefb 始终 加速度为0
% 原理：直线部分realTimefb和realTimeWb都是0。转弯部分realTimefb为0，realTimeWb的航向有值


%%
% A1-2
% w = 0.3*pi/180 ;    % 转弯部分的角速度 w*1   % 正为左转，负为右转
% A3
w = 0.4*pi/180 ;    % 转弯部分的角速度 w*1   % 正为左转，负为右转

t_1m = fix(1/v_front_const) ; % 1m的时间
t_turn90 = pi/2/w  ;    % 转90度弯的时间
longTurn90 = t_turn90*v_front_const ;   % 转90度弯的路程长度
%  trace: 上一行为转动的角度，下一行为向前直线的距离
% trace = [ 0         90          -90        -90        90       90       90       -90      -90     % 正为左转，负为右转
%           t_1m*20   t_1m*10     t_1m*10    t_1m*10    t_1m*10  t_1m*10  t_1m*10  t_1m*10  t_1m*20        ] ;
      
% trace = [ 0         90          -90        -90        90          % 正为左转，负为右转
%   t_1m*20   t_1m*10     t_1m*10    t_1m*10    t_1m*10         ] ;

% A2
% trace = [   0         -90          90        90         90       90       90           % 正为左转，负为右转
%             t_1m*15   t_1m*10     t_1m*20    t_1m*15    t_1m*10  t_1m*5  t_1m*40   ] ;
      
% A3
trace = [   0         -90          -90        -90           -90       -90          90          90         -90                 % 正为左转，负为右转
            t_1m*20   t_1m*30     t_1m*10    t_1m*10      t_1m*20  t_1m*10      t_1m*10      t_1m*30     t_1m*10            ] ;
        %      20      50          60          70           90      100         110            140          150        
        
% trace = [   0         -90          -90                 % 正为左转，负为右转
%             t_1m*7   t_1m*6     t_1m*2      ] ;     


trace(1,:) = trace(1,:)*pi/180 ;
trace(2,:) = trace(2,:)*frequency ;

line_N = size(trace,2) ;      

realTimefb_line1 = zeros(3,trace(2,1)) ;
realTimeWb_line1 = zeros(3,trace(2,1)) ;
realTimefb = realTimefb_line1 ;
realTimeWb = realTimeWb_line1 ;
for k=2:line_N
   % 先转弯再直线
   % 转弯
   w_k=w;

   t_turn_k = abs(fix(trace(1,k)/w_k))*frequency ;
   realTimefb_turnk = zeros(3,t_turn_k);
   realTimeWb_turnk = zeros(3,t_turn_k);
   flag = abs(trace(1,k))/trace(1,k) ;
   realTimeWb_turnk(3,:) = ones(1,t_turn_k)*w_k*flag ;
   % 直线
   realTimefb_linek = zeros(3,trace(2,k));
   realTimeWb_linek = zeros(3,trace(2,k));
   
   realTimefb = [ realTimefb realTimefb_turnk realTimefb_linek ]; %#ok<AGROW>
   realTimeWb = [ realTimeWb realTimeWb_turnk realTimeWb_linek ];
end

% realTimeWb = AddSlope( realTimeWb,t_1m,frequency,4,5,350 ) ;
% realTimeWb = AddSlope( realTimeWb,t_1m,frequency,50+2*longTurn90+2,4,-300 ) ;
% realTimeWb = AddSlope( realTimeWb,t_1m,frequency,110+7*longTurn90+5,4,300 ) ;
% 
% realTimeWb = AddRollChange( realTimeWb,t_1m,frequency,20+1*longTurn90+1,4,350 ) ;
% realTimeWb = AddRollChange( realTimeWb,t_1m,frequency,70+4*longTurn90+8,4,300 ) ;

runTimeNum = size(realTimefb,2);
sprintf('时长：%0.3f min',runTimeNum/60/frequency)
sprintf('路程：%0.3f m',runTimeNum*v_front_const/frequency)

%% 添加一个斜坡 -> 俯仰和高度的变化
% 从路程的第 start_route 米开始
% slopeLength:坡的曲线长度
% A_z 用于控制坡的高度，具体关系比较复杂（双重sin积分） (负的就变成个坑)
function realTimeWb = AddSlope( realTimeWb,t_1m,frequency,start_route,slopeLength,A_z )

% 模拟一个坡： cos 的俯仰角变化率，俯仰角的变化会导致高度的变化
start_N = t_1m*start_route * frequency ; % 坡开始的地方
T_N = t_1m*slopeLength * frequency ;   % 坡的长度 正好一个正弦周期
pitch_w = zeros(1,T_N+1);      % 坡段的  俯仰变化速率
% A_z = 500 ; % 幅度
for k=1:T_N
    pitch_w(k) = (2*pi/T_N)^2 * A_z * cos(2*pi/T_N * (k-1) ) ;
end
realTimeWb(1,start_N:start_N+T_N) = pitch_w ;

sprintf('坡时长：%d sec',t_1m*slopeLength )

%% 添加一次横滚角变化
function realTimeWb = AddRollChange( realTimeWb,t_1m,frequency,start_route,Length,A_z )

% 模拟一个坡： cos 的俯仰角变化率，俯仰角的变化会导致高度的变化
start_N = t_1m*start_route * frequency ; % 坡开始的地方
T_N = t_1m*Length * frequency ;   % 坡的长度 正好一个正弦周期
pitch_w = zeros(1,T_N+1);      % 坡段的  俯仰变化速率
% A_z = 500 ; % 幅度
for k=1:T_N
    pitch_w(k) = (2*pi/T_N)^2 * A_z * cos(2*pi/T_N * (k-1) ) ;
end
realTimeWb(2,start_N:start_N+T_N) = pitch_w ;