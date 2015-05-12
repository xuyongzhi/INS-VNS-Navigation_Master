%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.6
% 作者：xyz
% 功能：轨迹发生器
%   参考系为世界坐标系
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

function trueTrace = GetTrueTrace_i(isPlotFigure)
format long
% 世界坐标系轨迹发生器
if ~exist('isPlotFigure','var')
    isPlotFigure  = 1 ;
end
%% 输入 5个输入参数
prompt={'初始绝对大地位置(经度/°纬度/°高度/m)：            -','初始相对姿态（地理系/世界系）(度)：','初始速度（本体系下分解）(m/s)：','实时加速度（本体系下分解）(m/s^2)：','实时姿态变化率（本体系下分解）(度/s)：','数据频率(HZ)','时间(s)','轨迹名称','月面(m)/地面(e)'};
%defaultanswer={'116.35178 39.98057 53.44','-3 0 0','0.5 0.5 0.1','0.3 0.3 0.1','-0.3 0.5 -6','100','60*1','视景仿真测试-六维全动-1.4','m'};
% defaultanswer={'116.35178 39.98057 53.44','0 0 0','0 0.5 0','0 0 0','0 0 -2','100','60*1','平面匀速弧线','m'};
%defaultanswer={'116.35178 39.98057 53.44','0 0 0','0 0.03 0','0 0 0','0 0 0','100','60*21.17','对比实验_38.1m_30cm一图_直线','m'};
defaultanswer={'336.66 3 0','0 0 0','0 0.15 0','0 0 0','0 0 0.15','100','60*5','圆弧5min','m'};
%defaultanswer={'116.35178 39.98057 53.44','0 0 0','0 0.5 0','0 0 0','0 0 -0.0','100','60*5','匀速直线','m'};
%defaultanswer={'116.35178 39.98057 53.44','0 0 0','0 0  0','0 0 -0.0','0.2 0.3 0.2','100','60*2','姿态测试','m'};
% defaultanswer={'336.66 3 0','0 0 0','0 0.15 0','0 0  0','0 0 0','100','60*20','向前匀速直线_180m','m'};
%defaultanswer={'336.66 3 0','0 0 0','0 0.15 0','0 0 0','0 0 0.3','100','60* 20','向前匀速圆周_180m','m'};
%defaultanswer={'336.66 3 0','0 0 0','0 0.3 0','0 0 0','0 0 0.6','100','60*10','向前匀速圆周_180m','m'};
%defaultanswer={'336.66 3 0','0 0 0','0 0.15 0','0 0 0','0 0 0.15','100','60*40','匀速圆周360m','m'};
%defaultanswer={'336.66 3 0','0 0 0','0 0.03 0','0 0  0','0 0 0','100','60*22','向前匀速直线_38m','m'};
%defaultanswer={'336.66 3 0','0 0 0','0 0 0','0 0 0','0 0 0','100','60*5','静止5min','m'};
name='输入轨迹发生器的参数设置';
numlines=1;
answer=inputdlg(prompt,name,numlines,defaultanswer);

if isempty(answer)
    trueTrace = [];
    return; 
end
initialPosition_e = sscanf(answer{1},'%f');
initialPosition_e(1:2) = initialPosition_e(1:2)*pi/180 ;
initialAttitude_r = sscanf(answer{2},'%f')*pi/180;
initialVelocity_r = sscanf(answer{3},'%f');
realTimefb_const = sscanf(answer{4},'%f');   
realTimeWb_const = sscanf(answer{5},'%f')*pi/180; 
frequency = str2double(answer{6});
runTimeSec = eval(answer{7})+2.02;
traceName = answer{8};
planet = answer{9};
if ~strcmp(planet,'m') && ~strcmp(planet,'e')
    errordlg('星体设置错误！默认月面')
    planet = 'm';
end

%初始条件
runTimeNum=runTimeSec*frequency;
T=1/frequency;     % sec
%%%%% 如果“实时加速度”或“实时姿态变化率”输入“S”时加载动态数据,使用二次多项式生成
if strcmp(answer{4},'s')
    realTimefb = GetDynamicData_fb(runTimeNum) ;
else
    realTimefb = repmat(realTimefb_const,1,runTimeNum);
end
if strcmp(answer{5},'s')
    realTimeWb = GetDynamicData_Wb(runTimeNum) ;
else
    realTimeWb = repmat(realTimeWb_const,1,runTimeNum);
end

trueTrace.planet = planet;
trueTraceInput.initialPosition_e = initialPosition_e;
trueTraceInput.initialAttitude_r = initialAttitude_r;
trueTraceInput.initialVelocity_r = initialVelocity_r;
trueTraceInput.realTimefb = realTimefb;
trueTraceInput.realTimeWb = realTimeWb;
trueTraceInput.traceName = traceName;
trueTraceInput.frequency = frequency;
trueTraceInput.runTimeSec = runTimeSec;
trueTrace.trueTraceInput = trueTraceInput;  % 将轨迹的输入信息保存在trueTrace中便于查看

% 用字符串记录轨迹发生器的设置
str = sprintf('轨迹发生器:\t%s （星体：%s）\n',answer{8},planet);
str = sprintf('%s初始绝对大地位置(经度/°纬度/°高度/m)：\t%s\n初始相对姿态（地理系/世界系）(度)：\t%s\n初始速度（本体系下分解）(m/s)：\t\t%s\n',str,answer{1},answer{2},answer{3});
str = sprintf('%s实时加速度（本体系下分解）(m/s^2)：\t%s\n实时姿态变化率（本体系下分解）(度/s)：\t%s\n数据频率(HZ):%s\t\t时间(s):\t\t%s',str,answer{4},answer{5},answer{6},answer{7});
display(str)
trueTrace.traceRecord = str;
%% 星体常数
if strcmp(planet,'m')
    moonConst = getMoonConst;   % 得到月球常数
    gp = moonConst.g0 ;     % 用于导航解算
    wip = moonConst.wim ;
    Rp = moonConst.Rm ;
    e = moonConst.e;
    gk1 = moonConst.gk1;
    gk2 = moonConst.gk2;
    disp('轨迹发生器：月球')
else
    earthConst = getEarthConst;   % 得到地球常数
    gp = earthConst.g0 ;     % 用于导航解算
    wip = earthConst.wie ;
    Rp = earthConst.Re ;
    e = earthConst.e;
    gk1 = earthConst.gk1;
    gk2 = earthConst.gk2;
    disp('轨迹发生器：地球')
end

% 相对导航系的运动参数
eul_vect = zeros(3,runTimeNum);
attitude_r=zeros(3,runTimeNum); % 相对世界系的姿态
attitude_i=zeros(3,runTimeNum);
%Vn=zeros(3,runTimeNum);  % 相对地理系的速度
head=zeros(1,runTimeNum); % 传统定义的航向角

velocity_t =zeros(3,runTimeNum); % 相对地理系速度，在地理系分解 Vet_t
velocity_r =zeros(3,runTimeNum); % 相对世界系速度，在世界系分解 Vrt_r
                                    % velocity_r 与 velocity_t 相差一个Ctr
position_r=zeros(3,runTimeNum); % 相对世界系位置，在世界系分解 （x,y,z）
position_e = zeros(3,runTimeNum); % 大地系位置，在地理系分解（经纬高度）
acc_r = zeros(3,runTimeNum);    % 相对世界系加速度
%% 读设置的初始参数
% 需要设置/给定初始值得变量（即已知）：position_e,position_r,attitude_r,velocity_t,
% 需要用到初始值的变量：attitude_r，position_e,velocity_r,

% 设置初始经纬度和高度，用于惯导解算
position_e(1:2,1) = initialPosition_e(1:2);    % 经纬度 ° -> rad （大地坐标系的绝对初始位置）
position_e(3,1) = initialPosition_e(3) ;
position_r(:,1)=[0;0;0];  % 相对初始位置设为0：将初始时刻的地理系作为导航坐标系    position_r(1): x轴   position_r(2):y轴  position_r(3):z轴

attitude_r(:,1)=initialAttitude_r;    %初始姿态 sita ,gama ,fai
attitude_i(:,1)=initialAttitude_r;    %初始姿态 sita ,gama ,fai
Wrbb = realTimeWb;    % 姿态变化率  弧度/s ：载体相对世界系的角速度在本体系下的分解  即 Wrbb
fb=realTimefb;      % 行驶加速度  m/s/s
                        % initialVelocity_r 为本体系下分解的行驶初速度  m/s
Cbt=FCbn(attitude_r(:,1)); % 初始本体系 到 地理系/世界系 的转移矩阵
velocity_t(:,1) = Cbt*initialVelocity_r ;
Cbr=Cbt;
velocity_r(:,1) = Cbr*initialVelocity_r ;
Crb=Cbr';
Crb_last = Crb; % 记录上一时刻的Crb，用于计算Rbb

Cer=FCen(position_e(1,1),position_e(2,1));
Cre=Cer';
position_ini_er = FJWtoZJ(position_e(:,1),planet);  %初始时刻地固坐标系中的位置

wib_INSc=zeros(3,runTimeNum);
f_INSc=zeros(3,runTimeNum);

Q0 = FCnbtoQ(Crb);
Qir = [1 0 0 0]';

Wiee=[0;0;wip];
Wirr=Cer*Wiee;

waitbar_h=waitbar(0,'轨迹发生器');
for t=1:runTimeNum-1
    if mod(t,ceil(runTimeNum/200))==0
        waitbar(t/runTimeNum)
    end
    
    Crb = FCbn(attitude_r(:,t))';
    
    wib_INSc(:,t) = Crb*Wirr + Wrbb(:,t);
    Q0=Q0+0.5*T*[    0    ,-Wrbb(1,t),-Wrbb(2,t),-Wrbb(3,t);
                 Wrbb(1,t),     0    , Wrbb(3,t),-Wrbb(2,t);
                 Wrbb(2,t),-Wrbb(3,t),     0    , Wrbb(1,t);
                 Wrbb(3,t), Wrbb(2,t),-Wrbb(1,t),     0    ]*Q0;
    Q0=Q0/norm(Q0);
    Crb = FQtoCnb(Q0);
    Cbr=Crb';
%     
%     %output  attitude_r information
%     eul_vect(:,t) = dcm2eulr(Crb);
    opintions.headingScope=180;
    attitude_r(:,t+1) = GetAttitude(Crb,'rad',opintions);

    g = gp * (1+gk1*sin(position_e(2,t))^2-gk2*sin(2*position_e(2,t))^2);
    gn = [0;0;-g];
    Cen = FCen(position_e(1,t),position_e(2,t));
    Cnr = Cer * Cen';
    Cnb = Crb * Cnr;
    gb = Cnb * gn;
    gr = Cbr * gb;
    
    %%%%%%%%%%% 速度方程 %%%%%%%%%%
    a_rbr = Cbr * fb(:,t)+getCrossMarix(  Cbr * Wrbb(:,t) ) * velocity_r(:,t) ;
    %%%%%%%%%%% 比力方程 %%%%%%%%%%
%    f_INSc(:,t) = fb(:,t) + getCrossMarix( 2*Crb*Wirr )* Crb*velocity_r(:,t) - gb; % 比力方程在r系下的描述：fb是直接输入在本体系的驱动力
    f_INSc(:,t) = Crb * a_rbr + getCrossMarix( 2*Crb*Wirr )* Crb*velocity_r(:,t) - gb; 
%     %%%%%%%%%%% 速度方程 %%%%%%%%%%
%    % a_rbr = Cbr * f_INSc(:,t) - getCrossMarix( 2*Wirr )*velocity_r(:,t) + gr;    % 载体相对世界系的加速度，在世界系下的描述
%     a_rbr = Cbr * fb(:,t) ;
    acc_r(:,t) = a_rbr ;
    
    velocity_r(:,t+1) = velocity_r(:,t) + a_rbr * T;
    velocity_t(:,t+1) = Cnr' * velocity_r(:,t+1);   % 载体相对世界系和地理系的速度 转化
    position_r(:,t+1) = position_r(:,t) + velocity_r(:,t+1) * T;
    positione0 = Cre * position_r(:,t+1) + position_ini_er; % 将发射点惯性系中的位置转化到初始时刻地固系
    position_e(:,t+1) = FZJtoJW(positione0,planet);
    
    % Cir
    Qir=Qir+0.5*T*[    0    ,-Wirr(1 ),-Wirr(2 ),-Wirr(3 );
                 Wirr(1 ),     0    , Wirr(3 ),-Wirr(2 );
                 Wirr(2 ),-Wirr(3 ),     0    , Wirr(1 );
                 Wirr(3 ), Wirr(2 ),-Wirr(1),     0    ]*Qir;
    Qir=Qir/norm(Qir);
    Cir = FQtoCnb(Qir);
    Cib = Crb*Cir ;
    attitude_i(:,t+1) = GetAttitude(Cib,'rad',opintions);
    
end
close(waitbar_h)

%% 最后一个位置和速度是有效的，但加计合陀螺数据无效，统一去掉
runTimeNum = runTimeNum-1;  
time=zeros(1,runTimeNum);
for i=1:runTimeNum
    time(i)=i/frequency/60;
end
position_r = position_r(:,1:runTimeNum);
attitude_r = attitude_r(:,1:runTimeNum);
velocity_r = velocity_r(:,1:runTimeNum);
f_INSc = f_INSc(:,1:runTimeNum);
wib_INSc = wib_INSc(:,1:runTimeNum);
acc_r = acc_r(:,1:runTimeNum);

attitude_i = attitude_i(:,1:runTimeNum);
%% 输出

trueTrace.position = position_r ;
trueTrace.attitude = attitude_r ;
trueTrace.attitude_i = attitude_i ;
trueTrace.velocity = velocity_r ;
trueTrace.f_IMU = f_INSc ;
trueTrace.wib_IMU = wib_INSc ;
trueTrace.frequency = frequency;
trueTrace.initialPosition_e = initialPosition_e;
trueTrace.initialVelocity_r = initialVelocity_r;

    trueTrace.initialAttitude_r = initialAttitude_r;

trueTrace.acc_r=acc_r;

% 保存
savePath = [pwd,'\',traceName];
if isdir(savePath)
    delete([savePath,'\*']);
else
    mkdir(savePath) ;
end
save([savePath,'\trueTrace.mat'],'trueTrace')
save( 'trueTrace','trueTrace')
%% 绘图

if isPlotFigure ==1
    figure(1),plot(time,position_r);
    legend('x','y','z')
    title('月球车轨迹','fontsize',16);
    xlabel('x轴(m)','fontsize',12);
    ylabel('y轴(m)','fontsize',12);
    
    figure,plot(position_r(1,:),position_r(2,:));
    title('月球车轨迹','fontsize',16);
    xlabel('x轴(m)','fontsize',12);
    ylabel('y轴(m)','fontsize',12);
    
    figure,plot3(position_r(1,:),position_r(2,:),position_r(3,:));
    title('月球车轨迹','fontsize',16);
    xlabel('x轴(m)','fontsize',12);
    ylabel('y轴(m)','fontsize',12);
    zlabel('z轴(m)','fontsize',12);

    figure,plot(time,velocity_r(1,:),'k:',time,velocity_r(2,:),'b',time,velocity_r(3,:),'r--');
    title('三轴速度','fontsize',16);
    xlabel('时间(min)','fontsize',12);
    ylabel('速度(m/s)','fontsize',12);
    legend('X','Y','Z');

    figure,plot(time,acc_r(1,:),'k:',time,acc_r(2,:),'b',time,acc_r(3,:),'r--');
    title('三轴加速度','fontsize',16);
    xlabel('时间(min)','fontsize',12);
    ylabel('加速度(m/s^2)','fontsize',12);
    legend('X','Y','Z');
    
    figure,plot(time,attitude_r(1,:)*180/pi,'k:',time,attitude_r(2,:)*180/pi,'b',time,attitude_r(3,:)*180/pi,'r--');
    title('三轴姿态','fontsize',16);
    xlabel('时间(min)','fontsize',12);
    ylabel('姿态(度)','fontsize',12);
    legend('俯仰角','横滚角','航向角' );
    
    figure,plot(time,attitude_i(1,:)*180/pi,'k:',time,attitude_i(2,:)*180/pi,'b',time,attitude_i(3,:)*180/pi,'r--');
    title('三轴姿态','fontsize',16);
    xlabel('时间(min)','fontsize',12);
    ylabel('姿态(度)','fontsize',12);
    legend('俯仰角','横滚角','航向角' );

%     figure,plot(time,eul_vect(1,:)*180/pi,'k:',time,eul_vect(2,:)*180/pi,'b',time,eul_vect(3,:)*180/pi,'r--');
%     title('三轴姿态','fontsize',16);
%     xlabel('时间(min)','fontsize',12);
%     ylabel('姿态(度)','fontsize',12);
%     legend('横滚角','俯仰角','航向角' );
end

disp('轨迹发生器计算结束')

