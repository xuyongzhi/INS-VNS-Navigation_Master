%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.3
% 作者：xyz
% 功能：纯视觉的仿真解算
%    特征点+初始位置和姿态 -> Rcc Tcc 和纯视觉解算的位置、速度、姿态
% 源于 白师姐的程序“VOnav0411”
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [VisualSimu_RT,VOSimuResult] = main_VisualSimuNav( visualInputData,initialPosition,initialAttitude )
%% 导入数据 
%% visualInputData
% [1*N ] cell 数组， N为时刻数，每个cell为一个时刻前后4副图的匹配成功特征点，1个cell visualInputData{i}. 中包含4个成员：
% leftLocCurrent：左相机当前图匹配成功的特征点，[2*n]，n为该帧前后4副图匹配成功的特征点个数。
% rightLocCurrent：右相机当前图匹配成功的特征点
% leftLocNext：左相机下时刻图匹配成功的特征点
% rightLocNext：右相机下时刻图匹配成功的特征点
% matchedNum ：匹配成功特征点个数 double
% aveFeatureNum ：该时刻前后4副图特征点个数的平均值（未做匹配时） double

% 输入：左右特征点（匹配成功的），每个时刻特征点个数
pl = visualInputData.pl ;% 每个cell中一个结构体，其中包含 pl{i}.plp[例1*246] 和
        % pl{i}.plc（格式同plp），即左相机前后时刻的匹配特征点像面坐标。奇数为X，偶数为Y，1*246中包含123个特征点。
pr = visualInputData.pr ;
N = visualInputData.num;    % N[例1*200]：每一步运动中匹配成功的特征点个数

VOfre = 1/10;  % Hz 相机数据更新频率 (要求图像时均有采集的，不然解算的速度不适用)
f = 0.008;  % 焦距
B = 0.2;    % 基线间距 0.2m 
%% 常值
Re = 1737400;
e = 0.006;
num = size(pl,2);   % 输入数据个数

%INSfre = 100;
%runtime = size(f_INSc,2);

% camera parameter
m = 1040;
n = 1392;   % 分辨率：1392*1040（144万CCD）像素尺寸 4.65um * 4.65um
u0 = n/2;
v0 = m/2;
x = 6.4e-3; % 像元尺寸
y = 4.8e-3;
% install information
Cbc = [1, 0, 0;     % b到c系：Rx(-90)（绕x轴转-90°）
       0, 0,-1;
       0, 1, 0];
Ccb = Cbc';

S = diag([1,1,-1]);     % 奇异值最小二乘法中间参数
Rbb = zeros(3,3,num);   % 旋转矩阵：前一帧到当前帧
Tbb = zeros(3,num);
RELMOV = zeros(7,num);
Rk = zeros(6,6,num);    % 误差协方差(欧拉角、平移矢量量测量)
qRk = zeros(7,7,num);   % 误差协方差(四元数、平移矢量量测量)
%--最小平方中值定理参数--%
sm = 100;  % the number of Monte Carlo sample
q = 3;  % the number of matching point for each sample
    % 蒙特卡洛
Rot = zeros(3,3,sm);
Trs = zeros(3,sm);
Median = zeros(1,sm);

% navigation parameter in world frame
VOsta = zeros(3,num);
VOpos = zeros(3,num);
VOvel = zeros(3,num);
VOsta(:,1) = initialPosition ;
%初始姿态误差
dinit_att = [0/3600/180*pi;0/3600/180*pi;0/3600/180*pi];
% initial condition
pose0 = initialAttitude + dinit_att; % 初始姿态角+初始姿态误差
Cbr=FCbn(pose0(:,1));   % 初始方向余弦矩阵
Crb=Cbr';

%% 由方向余弦矩阵求姿态角初值  VOpos（:,1）
VOpos(1,1) = asin(Crb(2,3));  % 俯仰角
if Crb(3,3)>0
    VOpos(2,1)=atan(-Crb(1,3)/Crb(3,3)); % roll
elseif Crb(3,3)<0
    if Crb(1,3)>0
        VOpos(2,1)=VOpos(2,1)-pi;
    else
        VOpos(2,1)=VOpos(2,1)+pi;
    end
elseif Crb(3,3)==0
    if Crb(1,3)>0
        VOpos(2,1)=-pi/2;
    else
        VOpos(2,1)=1/2*pi;
    end
end
if Crb(2,2)>0   % 航向角
    if Crb(2,1)>0
        VOpos(3,1) = atan(-Crb(2,1)/Crb(2,2))  + 2 * pi;
    elseif Crb(2,1)<0
        VOpos(3,1) = atan(-Crb(2,1)/Crb(2,2));
    end
elseif Crb(2,2)<0
    VOpos(3,1) = pi + atan(-Crb(2,1)/Crb(2,2));
elseif Crb(2,2)==0
    if Crb(2,1)>0
        VOpos(3,1) = 1.5 * pi;
    elseif Crb(2,1)<0
        VOpos(3,1) = pi / 2;
    end
end

%% 显示进度条
h = waitbar(0,'纯视觉仿真导航解算中...');

for t = 1:num
    if(mod(t,ceil(num/100))==0)
       % 分100次更新
       waitbar(t/num);
    end
%% 3D restruction
    % 第t步运动中含有 N(t)个匹配特征点，求出这些特征点的摄像机三维坐标
    Point0 = zeros(3,N(t));
    Point1 = zeros(3,N(t));
    for i = 1:N(t)
        % compute disparity
        % 按平行透射模型计算，参考：周佳立.《双目立体视觉及三维反求研究》
        disp = pl{t}.plp(2*i-1) - pr{t}.prp(2*i-1); % pr{t}.prp(2*i-1)为右相机 u2-u0
        Point0(3,i) = B * f / disp;
        Point0(1,i) = B * pl{t}.plp(2*i-1) / disp;  % pl{t}.plp(2*i-1)为左相机 u1-10
        Point0(2,i) = B * pl{t}.plp(2*i) / disp;    % pl{t}.plp(2*i)为左相机 v1-v0
        disp = pl{t}.plc(2*i-1) - pr{t}.prc(2*i-1);
        Point1(3,i) = B * f / disp;
        Point1(1,i) = B * pl{t}.plc(2*i-1) / disp;
        Point1(2,i) = B * pl{t}.plc(2*i) / disp;
    end
    % 计算结果：Point0为前一帧所有匹配点的三维摄像机坐标，Point1为当前帧所有匹配点的三维摄像机坐标，
%%  Motion estimation to get coordinate translate matrix: LMedS
   % S = diag([ones(1,N(t)-1),-1]);
    for j = 1:sm
        while 1
            flag = 0;
            ind = randi(N(t),1,q);
            for p = 1:q-1
                for s = p+1:q
                    if ind(p) == ind(q)
                        flag = 1;
                    end
                end
            end
            if flag == 0
                break;
            end
        end
        % SVD method
        M0 = zeros(3,1);
        M1 = zeros(3,1);
        for i = 1:q
            M0 = M0 + Point0(:,ind(i));
            M1 = M1 + Point1(:,ind(i));
        end
        M0 = M0/q;
        M1 = M1/q;
        Pset0 = zeros(3,q);
        Pset1 = zeros(3,q);
        for i = 1:q
            Pset0(:,i) = Point0(:,ind(i)) - M0;
            Pset1(:,i) = Point1(:,ind(i)) - M1;
        end
        Q = Pset1*Pset0'/N(t);
        [U,A,V] = svd(Q);
        if abs(det(U)*det(V)-1) < 1e-10
            Rot(:,:,j) = U*V';
        elseif abs(det(U)*det(V)+1) < 1e-10
            Rot(:,:,j) = U*S*V';
        end
        Trs(:,j) = M1 - Rot(:,:,j) * M0;
        % compute regression variance and find Median
        r = zeros(1,N(t));
        for k = 1:N(t)
            r(k) = norm(Point1(:,k) - (Rot(:,:,j) * Point0(:,k) + Trs(:,j)));
        end
        Median(j) = median(r);
    end
    
    % find the minimum Median
    mMed = min(Median);
    ord = find( Median == min(Median));
    Rcc = Rot(:,:,ord(1));
    Tcc = Trs(:,ord(1));
    
    % compute robust standrad deviation
    sigma = 1.4826 * (1 + 5 / (N(t) - q)) * sqrt(mMed);
    % extract the inner matching points
    P1new = zeros(3,N(t));
    P2new = zeros(3,N(t));
    enum = 0;
    for j = 1:N(t)
        res = norm(Point1(:,j) - (Rcc * Point0(:,j) + Tcc));
        if res ^ 2 <= (2.5 * sigma) ^ 2
            enum = enum + 1;
            P1new(:,enum) = Point0(:,j);
            P2new(:,enum) = Point1(:,j);
        end
    end
    P1new(:,enum+1:N(t)) = [];
    P2new(:,enum+1:N(t)) = [];
    
    % SVD method to get the final motion estimation (R,T)
    M0 = zeros(3,1);
    M1 = zeros(3,1);
    for k = 1:enum
        M0 = M0 + P1new(:,k);
        M1 = M1 + P2new(:,k);
    end
    M0 = M0 / enum;
    M1 = M1 / enum;
    Pset0 = zeros(3,enum);
    Pset1 = zeros(3,enum);
    for k = 1:enum
        Pset0(:,k) = P1new(:,k) - M0;
        Pset1(:,k) = P2new(:,k) - M1;
    end
    Q = Pset1*Pset0'/enum;
    [U,D,V] = svd(Q);
    if abs(det(U)*det(V)-1) < 1e-10
        Rcc = U*V';
    elseif abs(det(U)*det(V)+1) < 1e-10
        Rcc = U*S*V';
    end
    Tcc = M1 - Rcc * M0;
   
    %%%%%%%%%%%%%%%%% LM优化 %%%%%%%%%%%%%%%%
    % 根据姿态矩阵Rcc计算姿态四元数
    q1=1/2*sqrt(abs(1+Rcc(1,1)-Rcc(2,2)-Rcc(3,3)));
    q2=1/2*sqrt(abs(1-Rcc(1,1)+Rcc(2,2)-Rcc(3,3)));
    q3=1/2*sqrt(abs(1-Rcc(1,1)-Rcc(2,2)+Rcc(3,3)));
    q0=sqrt(abs(1-q1^2-q2^2-q3^2));
    if Rcc(2,3)-Rcc(3,2)<0
        q1=-q1;
    end
    if Rcc(3,1)-Rcc(1,3)<0
        q2=-q2;
    end
    if Rcc(1,2)-Rcc(2,1)<0
        q3=-q3;
    end
    Q0=[q0;q1;q2;q3];
    Q0=Q0/norm(Q0);
    X = LMalgorithm1(P2new,P1new,Q0,Tcc);   
    Rcc = [X(1)^2+X(2)^2-X(3)^2-X(4)^2,    2*(X(2)*X(3)+X(1)*X(4)),        2*(X(2)*X(4)-X(1)*X(3));
           2*(X(2)*X(3)-X(1)*X(4)),    X(1)*X(1)-X(2)*X(2)+X(3)*X(3)-X(4)*X(4),    2*(X(3)*X(4)+X(1)*X(2));
           2*(X(2)*X(4)+X(1)*X(3)),        2*(X(3)*X(4)-X(1)*X(2)),    X(1)*X(1)-X(2)*X(2)-X(3)*X(3)+X(4)*X(4)];
    Tcc = X(5:7);
    
    %% 坐标系转换关系
%     % c'为当前，c为前一帧。b'为当前，b为前一帧
%     % c'= Rcc'*c+Tcc'
%     % b'= Rbb'*b+Tbb'
%     % c' = Cb'c'*b' = Cbc*b'    c = Cbc*b
%     % Cbc*b' = Rcc'*Cbc*b+Tcc'
%     % b' = Ccb*Rcc'*Cbc*b+Ccb*Tcc' => (1)Rbb' =  Ccb*Rcc'*Cbc   (2)Tbb'=Ccb*Tcc'
%     % Cr'b' = Rbb'*Crb*Crr' = Rbb'*Crb（Crr'=I）
                                    % Rcc：p（前一帧摄像机系）到c（当前帧摄像机系）的转移矩阵
    Rbb(:,:,t) = Ccb * Rcc * Cbc;   % Rbb：前一帧本体系到当前帧本体系的转移矩阵 Ccb为常值
    Tbb(:,t) = Ccb * Tcc;           % Tbb
    Crb = Rbb(:,:,t) * Crb;
    % 位置更新
    VOsta(:,t+1) = VOsta(:,t) - Crb' * Tbb(:,t);        % 推导过程参见xyz文档阅读笔记
    VOvel(:,t+1) = (VOsta(:,t+1) - VOsta(:,t)) * VOfre;
    
    %% 计算量测噪声方差阵
    % 根据姿态矩阵Rbb计算姿态四元数
    q1=1/2*sqrt(abs(1+Rbb(1,1,t)-Rbb(2,2,t)-Rbb(3,3,t)));
    q2=1/2*sqrt(abs(1-Rbb(1,1,t)+Rbb(2,2,t)-Rbb(3,3,t)));
    q3=1/2*sqrt(abs(1-Rbb(1,1,t)-Rbb(2,2,t)+Rbb(3,3,t)));
    q0=sqrt(abs(1-q1^2-q2^2-q3^2));
    if Rbb(2,3,t)-Rbb(3,2,t)<0
        q1=-q1;
    end
    if Rbb(3,1,t)-Rbb(1,3,t)<0
        q2=-q2;
    end
    if Rbb(1,2,t)-Rbb(2,1,t)<0
        q3=-q3;
    end
    Q0=[q0;q1;q2;q3];
    Q0=Q0/norm(Q0);
%     R(:,:,t) = R_cov1(Point0,Q0);
    qRk(:,:,t) = R_cov2(Point1,Point0,Rbb(:,:,t),Q0,Tbb(:,t));
    RELMOV(:,t) = [Q0;Tbb(:,t)];
    
    % 计算量测噪声方差阵
    % 计算姿态角
    pos(1) = asin(Rbb(2,3,t));  % 俯仰角  
    if Rbb(3,3,t)>0
        pos(2)=atan(-Rbb(1,3,t)/Rbb(3,3,t)); % roll
    elseif Rbb(3,3,t)<0
        if Rbb(1,3,t)>0
            pos(2)=pos(2)-pi;
        else
            pos(2)=pos(2)+pi;
        end
    elseif Rbb(3,3,t)==0
        if Rbb(1,3,t)>0
            pos(2)=-pi/2;
        else
            pos(2)=1/2*pi;
        end
    end
    if Rbb(2,2,t)>0   % 航向角
        if Rbb(2,1,t)>=0
            pos(3) = atan(-Rbb(2,1,t)/Rbb(2,2,t)); % + 2 * pi
        elseif Rbb(2,1,t)<0
            pos(3) = atan(-Rbb(2,1,t)/Rbb(2,2,t));
        end
    elseif Rbb(2,2,t)<0
        pos(3) = pi + atan(-Rbb(2,1,t)/Rbb(2,2,t));
    elseif Rbb(2,2,t)==0
        if Rbb(2,1,t)>0
            pos(3) = 1.5 * pi;
        elseif Rbb(2,1)<0
            pos(3) = pi / 2;
        end
    end
%     R(:,:,t) = R_covEuler(Point0,pos);
    Rk(:,:,t) = R_covEuler1(Point1,Point0,Rbb(:,:,t),pos,Tbb(:,t));
   
    % 由方向余弦矩阵求姿态角
    VOpos(1,t+1) = asin(Crb(2,3));  % 俯仰角  
    if Crb(3,3)>0
        VOpos(2,t+1)=atan(-Crb(1,3)/Crb(3,3)); % roll
    elseif Crb(3,3)<0
        if Crb(1,3)>0
            VOpos(2,t+1)=VOpos(2,t+1)-pi;
        else
            VOpos(2,t+1)=VOpos(2,t+1)+pi;
        end
    elseif Crb(3,3)==0
        if Crb(1,3)>0
            VOpos(2,t+1)=-pi/2;
        else
            VOpos(2,t+1)=1/2*pi;
        end
    end
    if Crb(2,2)>0   % 航向角
        if Crb(2,1)>=0
            VOpos(3,t+1) = atan(-Crb(2,1)/Crb(2,2)); % + 2 * pi
        elseif Crb(2,1)<0
            VOpos(3,t+1) = atan(-Crb(2,1)/Crb(2,2));
        end
    elseif Crb(2,2)<0
        if Crb(2,1) > 0
            VOpos(3,t+1) = - pi + atan(-Crb(2,1)/Crb(2,2));
        elseif Crb(2,1) < 0
            VOpos(3,t+1) = pi + atan(-Crb(2,1)/Crb(2,2));
        end
    elseif Crb(2,2)==0
        if Crb(2,1)>0
            VOpos(3,t+1) = 1.5 * pi;
        elseif Crb(2,1)<0
            VOpos(3,t+1) = pi / 2;
        end
    end
    
end
close(h);
%% 输出
% VisualSimu_RT ：中间结果 R 和 T 
VisualSimu_RT.Rbb = Rbb;
VisualSimu_RT.Tbb = Tbb;
% VOSimuResult：最终结果 速度 位置 姿态
VOSimuResult.VOPosition = VOsta;
VOSimuResult.VOAttitude = VOpos;
VOSimuResult.VOVelocity = VOvel;
% 保存数据
% save('F:\INS_Vision program\INS_VNS_correct\Simu_data\iVOtraceline60_1PixelError.mat','VOpos','VOsta','VOvel');
% save('F:\INS_Vision program\INS_VNS_correct\Simu_data\RTline60_1PixelError.mat','Rbb','Tbb');
% save('F:\INS_Vision program\INS_VNS_correct\Simu_data\Rkline60_1PixelError.mat','Rk');
% save('F:\INS_Vision program\INS_VNS_correct\Simu_data\pixelline60_1PixelError.mat','pl','pr');
% save('F:\INS_Vision program\INS_VNS_correct\Simu_data\QTline60_1PixelError.mat','RELMOV');
% save('F:\INS_Vision program\INS_VNS_correct\Simu_data\qRkline60_1PixelError.mat','qRk');


% 
% time=zeros(1,num+1);
% for i=1:num+1
%     time(i) = (i-1)/VOfre;
% end
% 
% figure(1),plot(VOsta(1,:),VOsta(2,:),'bo-',istar(1,1:INSfre/VOfre:runtime),istar(2,1:INSfre/VOfre:runtime),'r.-');
% title('月球车轨迹VO','fontsize',16);
% xlabel('x轴(m)','fontsize',12);
% ylabel('y轴(m)','fontsize',12);
% legend('真实轨迹','理想轨迹');
% 
% figure(2);
% plot(time,(VOsta(1,:) - istar(1,1:INSfre/VOfre:runtime)),'r');
% xlabel('时间(s)','fontsize',12);
% ylabel('x轴位置误差/m','fontsize',12);
% figure;
% plot(time,(VOsta(2,:) - istar(2,1:INSfre/VOfre:runtime)),'b:');
% xlabel('时间(s)','fontsize',12);
% ylabel('y轴位置误差/m','fontsize',12);
% figure;
% plot(time,(VOsta(3,:) - istar(3,1:INSfre/VOfre:runtime)),'k--');
% xlabel('时间(s)','fontsize',12);
% ylabel('z轴位置误差/m','fontsize',12);
% 
% figure;
% subplot(3,1,1);
% plot(time,VOpos(1,:)*180/pi,'b:',time,iposr(1,1:INSfre/VOfre:runtime)*180/pi,'r--');
% title('三轴姿态VO','fontsize',16);
% xlabel('时间(s)','fontsize',12);
% ylabel('俯仰角/°','fontsize',12);
% legend('实际值','理想值');
% subplot(3,1,2);
% plot(time,VOpos(2,:)*180/pi,'b:',time,iposr(2,1:INSfre/VOfre:runtime)*180/pi,'r--');
% xlabel('时间(s)','fontsize',12);
% ylabel('横滚角/°','fontsize',12);
% legend('实际值','理想值');
% subplot(3,1,3);
% plot(time,VOpos(3,:)*180/pi,'b:',time,iposr(3,1:INSfre/VOfre:runtime)*180/pi,'r--');
% xlabel('时间(s)','fontsize',12);
% ylabel('航向角/°','fontsize',12);
% legend('实际值','理想值');
% 
% figure;
% plot(time,(VOpos(1,:) - iposr(1,1:INSfre/VOfre:runtime))*180/pi,'r',time,(VOpos(2,:) - iposr(2,1:INSfre/VOfre:runtime))*180/pi,'b:',time,(VOpos(3,:) - iposr(3,1:INSfre/VOfre:runtime))*180/pi,'k--');
% xlabel('时间(s)','fontsize',12);
% ylabel('姿态角误差/°','fontsize',12);
% legend('俯仰角','横滚角','航向角');
% figure;
% plot(time,(VOpos(1,:) - iposr(1,1:INSfre/VOfre:runtime))*180/pi,'r');
% xlabel('时间(s)','fontsize',12);
% ylabel('俯仰角误差/°','fontsize',12);
% figure;
% plot(time,(VOpos(2,:) - iposr(2,1:INSfre/VOfre:runtime))*180/pi,'b:');
% xlabel('时间(s)','fontsize',12);
% ylabel('横滚角误差/°','fontsize',12);
% figure;
% plot(time,(VOpos(3,:) - iposr(3,1:INSfre/VOfre:runtime))*180/pi,'k--');
% xlabel('时间(s)','fontsize',12);
% ylabel('航向角误差/°','fontsize',12);
% 




