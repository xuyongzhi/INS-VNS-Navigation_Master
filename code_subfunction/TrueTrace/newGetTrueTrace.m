%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.6
% 2014.5.5修改：
% 作者：xyz
% 功能：轨迹发生器
%   参考系为世界坐标系
% 5.18改 velocity_t(:,1) = Cbt*initialVelocity_r ;velocity_r(:,1) =
%       Cbr*initialVelocity_r ; 为 velocity_t(:,1) =initialVelocity_r ; velocity_r(:,1) = initialVelocity_r ;
% 6.7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

function trueTrace = newGetTrueTrace(isPlotFigure,trueTrace)
format long
% 世界坐标系轨迹发生器
if ~exist('isPlotFigure','var')
    isPlotFigure  = 1 ;
end
if ~exist('trueTrace','var')
    load('trueTrace.mat')
else
    trueTrace=[];
end
isUseOld=0;
if isfield(trueTrace,'trueTraceInput')
   button = questdlg('是否采用trueTrace中的轨迹参数设置？'); 
   if strcmp(button,'Yes');
        planet = trueTrace.planet ;
        trueTraceInput = trueTrace.trueTraceInput ;
        initialPosition_e = trueTraceInput.initialPosition_e;
        initialAttitude_r = trueTraceInput.initialAttitude_r ;
        initialVelocity_r = trueTraceInput.initialVelocity_r ;
        realTimefb = trueTraceInput.realTimefb ;
        realTimeWb = trueTraceInput.realTimeWb ;
        traceName = trueTraceInput.traceName ;
        frequency = trueTraceInput.frequency ;
        runTimeSec = trueTraceInput.runTimeSec ;
        runTimeNum=runTimeSec*frequency+1;
        T=1/frequency*ones(1,runTimeNum);     % sec
        isUseOld=1;
   end
end
isReverseIMU = 0 ;  % 是否通过速度和姿态反推出IMU
if isUseOld==0
    button = questdlg('是否通过速度和姿态反推出IMU？'); 
   if strcmp(button,'Yes');
       trueTrace=[];
        isReverseIMU = 1 ;
        [FileName,PathName] = uigetfile('*.mat','载入包含轨迹设置的 trueTrace ');
        trueTrace_mesr = importdata([PathName,FileName]);
        
        initialPosition_e = trueTrace_mesr.initialPosition_e ;
        initialPosition_r = trueTrace_mesr.initialPosition_r ;
        initialVelocity_r = trueTrace_mesr.initialVelocity_r  ;
        initialAttitude_r=  trueTrace_mesr.initialAttitude_r  ;
        frequency = trueTrace_mesr.frequency  ;
        traceName = 'kitti';
        planet = 'e';
        
        dif_wrbb = trueTrace_mesr.dif_wrbb ;
        dif_arbr = trueTrace_mesr.dif_arbr ;
        runTime_IMU = trueTrace_mesr.runTime_IMU ;
        position_In = trueTrace_mesr.position ; 
        attitude_In = trueTrace_mesr.attitude ;
        velocity_In = trueTrace_mesr.velocity ;
        runTimeNum = length(attitude_In);
        
        T = runTime_to_setpTime(runTime_IMU) ;
        T = [T; T(runTimeNum-1)];
        realTimeWb = dif_wrbb ;
        realTimefb=[];
   end
end
%% 输入 5个输入参数
% 玉兔号：西经19.5度（340.5°）,北纬44.1°
if isUseOld==0 && isReverseIMU==0
    prompt={'初始绝对大地位置(经度/°纬度/°高度/m)：            -','初始姿态（初始地理系/世界系）（俯仰、横滚、偏航）(度)：','初始速度（本体系下分解）(m/s)：','实时加速度（本体系下分解）(m/s^2)：','实时姿态变化率（本体系下分解）(度/s)：','数据频率(HZ)','时间(s)','轨迹名称','月面(m)/地面(e)'};

    % defaultanswer={'116.35178 39.98057 53.44','0 0 0','0 0.5 0','0 0 0','0 0 -2','100','60*1','平面匀速弧线','m'};
    %defaultanswer={'336.66 3 0','0 0 0','0 0.15 0','0 0 0','0 0 0.15','100','60*5','圆弧5min','m'};
    %defaultanswer={'116.35178 39.98057 53.44','0 0 0','0 0.5 0','0 0 0','0 0 -0.0','100','60*5','匀速直线','m'};
    %defaultanswer={'116.35178 39.98057 53.44','0 0 0','0 0  0','0 0 -0.0','0.2 0.3 0.2','100','60*2','姿态测试','m'};
    %defaultanswer={'336.66 3 0','0 0 0','0 0.03 0','0 0 0','0 0 0','100','60*60*5','向前匀速直线5h480m','m'};
    %defaultanswer={'336.66 3 0','0 0 0','0 0.03 0','0 0 0','s','100','60*60*5','向前曲线5h480m','m'};
    %defaultanswer={'336.66 3 0','0 0 0','0 0.03 0','0 0 0','0 0 0.02','100','60*10','匀速圆周10min','m'};
    %defaultanswer={'336.66 3 0','0 0 0','0 0.15 0','0 0 0','0 0 0.15','100','60*40','匀速圆周360m','m'};
    %defaultanswer={'336.66 3 0','0 0 0','0 0.03 0','0 0  0','0 0 0','100','60*22','向前匀速直线_38m','m'};
    % defaultanswer={'340.5 44.1 0','-5 -5 0','0.3 0.3 0','0 0 0','2 2 2','10','5','横滚测试','m'};
    
    %defaultanswer={'340.5 44.1 0','0 0 0','0 0.03 0','0 0 0','0 0 0','20','60*60*2','向前216m_2h','m'};
    %defaultanswer={'340.5 44.1 0','0 0 0','0 0.03 0','rtg','0 0 0','10','0','长方形_有噪声','m'};
    % defaultanswer={'340.5 44.1 0','0 0 0','0 0.03 0','A','0 0 0','10','0','特殊轨迹A','m'};
    defaultanswer={'340.5 44.1 0','0 0 10','0 0.03 0','A','0 0 0','10','0','轨迹A6平视0点1HZ','m'};
    %defaultanswer={'340.5 44.1 0','0 0 0','0 0.03 0','s','0 0 0','10','60*60*0.2','S轨迹','m'};
    
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
    runTimeSec = eval(answer{7});
    traceName = answer{8};
    planet = answer{9};
    if ~strcmp(planet,'m') && ~strcmp(planet,'e')
        errordlg('星体设置错误！默认月面')
        planet = 'm';
    end

    %初始条件
    runTimeNum=runTimeSec*frequency+1;
    T=1/frequency*ones(1,runTimeNum);     % sec
    %%%%% 如果“实时加速度”或“实时姿态变化率”输入“S”时加载动态数据,使用二次多项式生成

    switch answer{4}
        case 's' 
            % S形曲线
            realTimefb = GetDynamicData_fb_s(runTimeNum,frequency) ; 
            [realTimeWb] = GetDynamicData_Wb_s(runTimeNum,frequency) ;
        case 'rtg'
            % 长方形
            [realTimefb,realTimeWb,runTimeNum] = GetDynamicData_Wb_rtg(initialVelocity_r(2),frequency) ;
            T=1/frequency*ones(1,runTimeNum);
            display(T)
        case 'A'
            [realTimefb,realTimeWb,runTimeNum] = GetDynamicData_Wb_A(initialVelocity_r(2),frequency) ;
            T=1/frequency*ones(1,runTimeNum);
            
        otherwise
            realTimefb = repmat(realTimefb_const,1,runTimeNum);
            realTimeWb = repmat(realTimeWb_const,1,runTimeNum);
    end
  % 给 realTimefb realTimeWb 加噪声
    bN = length(realTimeWb);
    realTimefb_Noise=zeros(3,bN);
    realTimeWb_Noise=zeros(3,bN);
    for i=1:3
        realTimefb_Noise(i,:) = normrnd(0,2e-5 ,1,bN) ;         
        realTimeWb_Noise(i,:) = normrnd(0,0.02*pi/180,1,bN) ; 
    end
    realTimefb_Noise(3,:) = normrnd(0,3e-7 ,1,bN) ;
    realTimeWb_Noise(3,:) = normrnd(0,0.03*pi/180,1,bN) ; 
% %     
    realTimefb = realTimefb+realTimefb_Noise ;
    realTimeWb = realTimeWb+realTimeWb_Noise ;
    
    
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
    time_h = size(realTimeWb,2)/frequency / 3600 ;  % 小时计时间
    % 用字符串记录轨迹发生器的设置
    str = sprintf('轨迹发生器:\t%s （星体：%s）\n',answer{8},planet);
    str = sprintf('%s初始绝对大地位置(经度/°纬度/°高度/m)：\t%s\n初始相对姿态（地理系/世界系）(度)：\t%s\n初始速度（本体系下分解）(m/s)：\t\t%s\n',str,answer{1},answer{2},answer{3});
    str = sprintf('%s实时加速度（本体系下分解）(m/s^2)：\t%s\n实时姿态变化率（本体系下分解）(度/s)：\t%s\n数据频率(HZ):%s\t\t时间(s):\t\t%0.2f h',str,answer{4},answer{5},answer{6},time_h);
    display(str)
    trueTrace.traceRecord = str;
end
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
attitude_t=zeros(3,runTimeNum); % 相对地理系的姿态
%Vn=zeros(3,runTimeNum);  % 相对地理系的速度
head=zeros(1,runTimeNum); % 传统定义的航向角

velocity_t =zeros(3,runTimeNum); % 相对地理系速度，在地理系分解 Vet_t
velocity_r =zeros(3,runTimeNum); % 相对世界系速度，在世界系分解 Vrt_r
                                    % velocity_r 与 velocity_t 相差一个Ctr
position_r=zeros(3,runTimeNum); % 相对世界系位置，在世界系分解 （x,y,z）
position_e = zeros(3,runTimeNum); % 大地系位置，在地理系分解（经纬高度）
acc_r = zeros(3,runTimeNum-1);    % 相对世界系加速度
%% 读设置的初始参数
% 需要设置/给定初始值得变量（即已知）：position_e,position_r,attitude_r,velocity_t,
% 需要用到初始值的变量：attitude_r，position_e,velocity_r,

% 设置初始经纬度和高度，用于惯导解算
position_e(1:2,1) = initialPosition_e(1:2);    % 经纬度 ° -> rad （大地坐标系的绝对初始位置）
position_e(3,1) = initialPosition_e(3) ;
position_r(:,1)=[0;0;0];  % 相对初始位置设为0：将初始时刻的地理系作为导航坐标系    position_r(1): x轴   position_r(2):y轴  position_r(3):z轴

attitude_r(:,1)=initialAttitude_r;    %初始姿态 sita ,gama ,fai
Wrbb = realTimeWb;    % 姿态变化率  弧度/s ：载体相对世界系的角速度在本体系下的分解  即 Wrbb
fb=realTimefb;      % 行驶加速度  m/s/s
                        % initialVelocity_r 为本体系下分解的行驶初速度  m/s
Cbt=FCbn(attitude_r(:,1)); % 初始本体系 到 地理系/世界系 的转移矩阵
Cbr=Cbt;

velocity_t(:,1) = initialVelocity_r ;
velocity_r(:,1) = initialVelocity_r ;

Crb=Cbr';
Crb_last = Crb; % 记录上一时刻的Crb，用于计算Rbb

Cer=FCen(position_e(1,1),position_e(2,1));
Cre=Cer';
position_ini_er = FJWtoZJ(position_e(:,1),planet);  %初始时刻地固坐标系中的位置

wib_INSc=zeros(3,runTimeNum-1);
f_INSc=zeros(3,runTimeNum-1);

Q0 = FCnbtoQ(Crb);

Wiee=[0;0;wip];
Wirr=Cer*Wiee;

waitbar_h=waitbar(0,'轨迹发生器');
for t=1:runTimeNum-1
    if mod(t,ceil(runTimeNum/100))==0
        waitbar(t/runTimeNum)
    end
                    
    wib_INSc(:,t) = Crb*Wirr + Wrbb(:,t);
%     Q0=Q0+0.5*T(t)*[    0    ,-Wrbb(1,t),-Wrbb(2,t),-Wrbb(3,t);
%                  Wrbb(1,t),     0    , Wrbb(3,t),-Wrbb(2,t);
%                  Wrbb(2,t),-Wrbb(3,t),     0    , Wrbb(1,t);
%                  Wrbb(3,t), Wrbb(2,t),-Wrbb(1,t),     0    ]*Q0;
%     Q0=Q0/norm(Q0);
    Q0  = QuaternionDifferential( Q0,Wrbb(:,t),T(t) ) ;
%             Crb = FQtoCnb(Q0);
%             Cbr=Crb';
%         %     
%         %     %output  attitude_r information
%         %     eul_vect(:,t) = dcm2eulr(Crb);
%             opintions.headingScope=180;
%             attitude_r(:,t+1) = GetAttitude(Crb,'rad',opintions);

    g = gp * (1+gk1*sin(position_e(2,t))^2-gk2*sin(2*position_e(2,t))^2);
    gn = [0;0;-g];
    Cen = FCen(position_e(1,t),position_e(2,t));
    Cnr = Cer * Cen';
    Cnb = Crb * Cnr;
    gb = Cnb * gn;
    gr = Cbr * gb;
    
    %%%%%%%%%%% 速度方程 %%%%%%%%%%
    if isReverseIMU==0
        a_rbr = Cbr * fb(:,t)+getCrossMarix(  Cbr * Wrbb(:,t) ) * velocity_r(:,t) ;
    else
        a_rbr = dif_arbr(:,t);
    end
    %%%%%%%%%%% 比力方程 %%%%%%%%%%
%    f_INSc(:,t) = fb(:,t) + getCrossMarix( 2*Crb*Wirr )* Crb*velocity_r(:,t) - gb; % 比力方程在r系下的描述：fb是直接输入在本体系的驱动力
    f_INSc(:,t) = Crb * a_rbr + getCrossMarix( 2*Crb*Wirr )* Crb*velocity_r(:,t) - gb; 
%     %%%%%%%%%%% 速度方程 %%%%%%%%%%
%    % a_rbr = Cbr * f_INSc(:,t) - getCrossMarix( 2*Wirr )*velocity_r(:,t) + gr;    % 载体相对世界系的加速度，在世界系下的描述
%     a_rbr = Cbr * fb(:,t) ;
    acc_r(:,t) = a_rbr ;
    
    Crb = FQtoCnb(Q0);
    Cbr=Crb';
%     
%     %output  attitude_r information
%     eul_vect(:,t) = dcm2eulr(Crb);
    opintions.headingScope=180;
    attitude_r(:,t+1) = GetAttitude(Crb,'rad',opintions);
    
    % 求相对地理系的姿态
    Cnb = Crb*Cnr ;
    attitude_t(:,t+1) = GetAttitude(Cnb,'rad',opintions);
    
    velocity_r(:,t+1) = velocity_r(:,t) + a_rbr * T(t);
    velocity_t(:,t+1) = Cnr' * velocity_r(:,t+1);   % 载体相对世界系和地理系的速度 转化
    position_r(:,t+1) = position_r(:,t) + velocity_r(:,t) * T(t);
    positione0 = Cre * position_r(:,t+1) + position_ini_er; % 将发射点惯性系中的位置转化到初始时刻地固系
    position_e(:,t+1) = FZJtoJW(positione0,planet);
    
end
close(waitbar_h)

%% 输出

trueTrace.position = position_r ;
trueTrace.position_e = position_e ;

trueTrace.attitude = attitude_r ;
trueTrace.attitude_t = attitude_t;

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
if  isReverseIMU == 1 
    % 将IMU数据保存到原来的 trueTrace 中
    trueTrace_mesr.f_IMU = f_INSc ;
    trueTrace_mesr.wib_IMU = wib_INSc ;
    save([PathName,'\trueTrace.mat'],'trueTrace_mesr')
end
%% 绘图

if isPlotFigure ==1
    if isReverseIMU==0
        time = (1:length(position_r))/frequency ;
    else        
        time = (1:runTimeNum)/frequency ;
    end
    
    figure,plot(time,position_r);
    legend('x','y','z')
    title('三维月球车轨迹','fontsize',16);
    xlabel('时间(sec)','fontsize',12);
    ylabel('位置(m)','fontsize',12);
    saveas(gcf,[savePath,'\三维月球车轨迹.fig'])
    
    figure,plot(position_r(1,:),position_r(2,:),'b');
    title('二维月球车轨迹','fontsize',16);
    xlabel('x轴(m)','fontsize',12);
    ylabel('y轴(m)','fontsize',12);
    if isReverseIMU==1
        hold on
        plot(position_In(1,:),position_In(2,:),'-.r');
        legend('反推解算','实际测量')
    end
    saveas(gcf,[savePath,'\二维月球车轨迹.fig'])
    
    figure,plot3(position_r(1,:),position_r(2,:),position_r(3,:));
    title('月球车轨迹','fontsize',16);
    xlabel('x轴(m)','fontsize',12);
    ylabel('y轴(m)','fontsize',12);
    zlabel('z轴(m)','fontsize',12);

    figure,plot(time,velocity_r(1,:),'k:',time,velocity_r(2,:),'b',time,velocity_r(3,:),'r--');
    title('三轴速度','fontsize',16);
    xlabel('时间(sec)','fontsize',12);
    ylabel('速度(m/s)','fontsize',12);
    legend('X','Y','Z');

    figure,plot(time(1:length(acc_r)),acc_r(1,:),'k:',time(1:length(acc_r)),acc_r(2,:),'b',time(1:length(acc_r)),acc_r(3,:),'r--');
    title('三轴加速度','fontsize',16);
    xlabel('时间(sec)','fontsize',12);
    ylabel('加速度(m/s^2)','fontsize',12);
    legend('X','Y','Z');
    
    figure,plot(time,attitude_r(1,:)*180/pi,'k:',time,attitude_r(2,:)*180/pi,'b',time,attitude_r(3,:)*180/pi,'r--');
    title('三轴姿态','fontsize',16);
    xlabel('时间(sec)','fontsize',12);
    ylabel('姿态(度)','fontsize',12);
    legend('俯仰角','横滚角','航向角' );
    
    if isReverseIMU==1
        figure;
        plot(time,attitude_r(1,:)*180/pi,'--b');
        hold on
        plot(time,attitude_In(1,:)*180/pi,'-.r');
        title('pitch','fontsize',16);
        legend('反推解算','实际测量')
        
        figure;
        plot(time,attitude_r(2,:)*180/pi,'--b');
        hold on
        plot(time,attitude_In(2,:)*180/pi,'-.r');
        title('roll','fontsize',16);
        legend('反推解算','实际测量')
        
        figure;
        plot(time,attitude_r(3,:)*180/pi,'--b');
        hold on
        plot(time,attitude_In(3,:)*180/pi,'-.r');
        title('yaw','fontsize',16);
        legend('反推解算','实际测量')
    end
    
%     figure,plot(time,eul_vect(1,:)*180/pi,'k:',time,eul_vect(2,:)*180/pi,'b',time,eul_vect(3,:)*180/pi,'r--');
%     title('三轴姿态','fontsize',16);
%     xlabel('时间(sec)','fontsize',12);
%     ylabel('姿态(度)','fontsize',12);
%     legend('横滚角','俯仰角','航向角' );
end

disp('轨迹发生器计算结束')

