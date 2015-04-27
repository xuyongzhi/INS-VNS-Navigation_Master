%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          buaa xyz 
%                          2014 5 23
%               SINS误差状态模型，dRbb dTbb 
%                         状态更新函数
% 量测量: dQbb（经抓换因子转换）,dTbb（在前一时刻分解）
% X=[dat dv dr gyroDrift accDrift dat_last dr_last] 
% EKF   % 状态方程线性，量测方程非线性   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ newX,newP,X_correct,X_pre,Zinteg_error,Zinteg,Zinteg_pre,R_INS,T_INS,R_VNS,T_VNS ] = updateX_SINSerror_subQbbsubTbb...
( X,P,Q_ini,R,Wirr,fb,cycleT_VNS,Crb,position,Rbb,Tbb,isTbb_last,Crb_SINSpre,position_SINSpre,isDebudMode,calZMethod )

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
     case 'sub_QTb'
%          dbstop in calZ_subQbbsubTbb
         [Z,R_INS,T_INS,R_VNS,T_VNS] = calZ_subQbbsubTbb( position_SINSpre,position,Crb_SINSpre,Crb,Rbb,Tbb,calZMethod,isTbb_last );   % Z=Z_INS-Z_VNS
         % 量测矩阵 H  （雅克比求）
        H1 = 1 / 2 * eye(3);
        H2 = - 1 / 2 * Crb_SINSpre * Crb';
        % 7.16改
        H = [H1,zeros(3,12),H2,zeros(3);
             zeros(3,6),Crb,zeros(3,6),zeros(3),-Crb];

%         % 7.16前
%          H = [H1,zeros(3,12),H2,zeros(3);
%              zeros(3,6),Crb,zeros(3,6),-Crb,zeros(3)];
%          if isTbbLast==1
%             H = [H1,zeros(3,12),H2,zeros(3);
%                 zeros(3,6),Crb_SINSpre,zeros(3,6),-Crb_SINSpre,zeros(3)];
%         end
     case 'd_RTw'
         [Z,R_INS,T_INS,R_VNS,T_VNS] = calZ_dRbbdTbb( position_SINSpre,position,Crb_SINSpre,Crb,Rbb,Tbb,isTbb_last );   % Z=Z_INS-Z_VNS
         H_SINSerror = [ eye(3) zeros(3,12) ; zeros(3,6) eye(3) zeros(3,6)  ];      % 和报告里的 TINS 定义相反
         H = [ H_SINSerror zeros(6,6)];
     case 'd_RTb'
         [Z,R_INS,T_INS,R_VNS,T_VNS] = calZ_dRbbsubTbb_b( position_SINSpre,position,Crb_SINSpre,Crb,Rbb,Tbb,isTbb_last ); 
         % 量测矩阵 H  （雅克比求）
        H = [   eye(3) zeros(3,18) ;
             zeros(3,6),Crb,zeros(3,6),-Crb,zeros(3)];
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