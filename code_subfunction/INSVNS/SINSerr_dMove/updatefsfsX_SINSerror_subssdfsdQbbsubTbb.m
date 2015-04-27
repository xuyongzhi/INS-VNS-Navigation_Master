%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          buaa xyz 
%                          2014 7 23
%               SINS误差状态模型，dRbb dTbb 
%                         状态更新函数
% 量测量: dQbb（更正了四元数误差定义）,dTbb
%  视觉误差在本体系建立：  X=[dat dv dr gyroDrift accDrift RbDrift_vns TbDrift_vns] 
%  视觉误差在摄像机系建立：X=[dat dv dr gyroDrift accDrift RcDrift_vns TcDrift_vns] 
% EKF   % 状态方程线性，量测方程非线性   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ newX,newP,X_correct,X_pre,Zinteg_error,Zinteg,Zinteg_pre,R_INS,T_INS,R_VNS,T_VNS ] = updateX_SINSerr_dMove_new...
( X,P,Q_ini,R,Wirr,fb,cycleT_VNS,Crb,position,Rbb,Tbb,Crb_SINSpre,position_SINSpre,isDebudMode,calZMethod )


% F,G,Fai：扩展调用惯导误差方程的结果
[F_INS,G_INS] = GetF_StatusErrorSINS(Crb',Wirr,fb);  % 注意取滤波周期，而不是惯导解算周期
F_k = [F_INS zeros(15,6);zeros(6,21)];
% G_k = [G_INS;zeros(6,6)];
Fai_k = FtoFai(F_k,cycleT_VNS);

% 选择状态预测的方式
X_pre = Fai_k * X;   % 状态一步预测：描述的是 新时刻 惯导递推的结果与真实轨迹之间的误差

% 上一时刻的（增广部分）位置和姿态做法一致，均采用上一时刻的估计值

    X_pre(16:21) = zeros(6,1);  % 上一时刻的估计值 即为 0

    %% 两种不同的量测计算方法 Z 和 H 不同
 % 量测信息：用SINS预测的结果和上一时刻的估值计算              
 switch calZMethod.Z_method
     case 'new_dQTb'
%           dbstop in calZ_new_dQTb
         [Z,R_INS,T_INS,R_VNS,T_VNS] = calZ_new_dQTb( position_SINSpre,position,Crb_SINSpre,Crb,Rbb,Tbb );   % Z=Z_INS-Z_VNS 
         % 量测矩阵 H  （雅克比求）
         Qrb_SINSpre = FCnbtoQ(Crb_SINSpre) ;
         Qrb_SINSpre_Inv = [Qrb_SINSpre(1);Qrb_SINSpre(2:4)];
         
         H1 = 1/2*FM( GetQinvM(Qrb_SINSpre)*GetQM(Qrb_SINSpre_Inv) );
         H = [  H1,zeros(3,12),zeros(3,6);
                zeros(3,6), Crb, zeros(3,6), zeros(3,6)  ];
                 

     case 'new_dQTb_VnsErr'
         [Z,R_INS,T_INS,R_VNS,T_VNS] = calZ_new_dQTb( position_SINSpre,position,Crb_SINSpre,Crb,Rbb,Tbb );   % Z=Z_INS-Z_VNS
         % 量测矩阵 H  （雅克比求）
         Qrb_SINSpre = FCnbtoQ(Crb_SINSpre) ;
         Qrb_SINSpre_Inv = [Qrb_SINSpre(1);Qrb_SINSpre(2:4)];
         
         RbDrift_vns = FCbn(X(16:18))';
         QbDrift_vns = FCnbtoQ(RbDrift_vns) ;
         QbDrift_vns_Inv = [QbDrift_vns(1);QbDrift_vns(2:4)];
                  
         H1 = 1/2*FM( GetQinvM(Qrb_SINSpre)*GetQM(QbDrift_vns_Inv)*GetQM(Qrb_SINSpre_Inv) );
         Qdat = FCnbtoQ(FCbn(X(1:3))') ;
         Q1 = QuaternionMultiply(Qrb_SINSpre_Inv,Qdat);
         Q2 = QuaternionMultiply(Q1,Qrb_SINSpre) ;
         H6 = -1/2*FM(GetQinvM(Q2)) ;
         H = [  H1,zeros(3,12),H6,zeros(3);
                zeros(3,6), Crb, zeros(3,6), zeros(3),eye(3)  ];
         
     case 'new_dQTc'
       
         
 end
%%
Zinteg=Z;
% Q：系统噪声方差阵
%  Q_k = calQ( Q_ini,F_k,cycleT_VNS,G_k );
Q_k = Q_ini ;
% 均方差一步预测
%         P_pre = Fai_k*P_k*Fai_k' + P_new_diag;
Poo = P(1:15,1:15);
Pod = P(1:15,16:21);
PodT = P(16:21,1:15);
Pdd = P(16:21,16:21);
Poo10 = Fai_k(1:15,1:15) * Poo * Fai_k(1:15,1:15)' + Q_k(1:15,1:15);
Pod10 = Fai_k(1:15,1:15) * Pod;
PodT10 = PodT * Fai_k(1:15,1:15)';
Pdd10 = Pdd;
P_pre = [Poo10 Pod10;PodT10 Pdd10];   % 均方误差一步预测
%         P10a = Fai_k * P * Fai_k' + [P_new_diag,zeros(15,6);zeros(6,15),1e-8*eye(6,6)];
% 滤波增益
K_t = P_pre * H' / (H * P_pre * H' + R);   
% 状态估计
Zinteg_pre = H * X_pre;
Zinteg_error = Z - H * X_pre;
X_correct = K_t * (Z - H * X_pre); 
newX = X_pre + X_correct ;
%X(:,k_integ+1) = X_pre + K_t * (Z - H * X_pre);   
XNum = length(X);
newP_temp=(eye(XNum,XNum)-K_t*H)*P_pre*(eye(XNum,XNum)-K_t*H)'+K_t*R*K_t';
%         P(16:21,16:21,k_integ+1) = Ts(16:21,:) * P(1:15,1:15,k_integ+1) * Ts(16:21,:)';
Ts = [eye(15);eye(3),zeros(3,12);zeros(3,6),eye(3),zeros(3,6)];
newP = Ts * newP_temp(1:15,1:15) * Ts';
if isfield(isDebudMode,'onlyStateFcn') && isDebudMode.onlyStateFcn==1
    % 纯状态递推，不作量测的修正
    newX = X_pre;
end


function MH = FM(M)
MH = M(2:4,2:4);

%% q。p=M(q)p
function M = GetQM(Q)

M = [ Q(1) -Q(2:4)';
     Q(2:4) Q(1)*eye(3)+getCrossMarix(Q(2:4)) ];
 
 %% q。p=M(p)q
function invM = GetQinvM(Q)

invM = [ Q(1) -Q(2:4)';
     Q(2:4) Q(1)*eye(3)-getCrossMarix(Q(2:4)) ];
