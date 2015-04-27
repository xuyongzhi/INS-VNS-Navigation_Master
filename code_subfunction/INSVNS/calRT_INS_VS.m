%% 计算 R/T_INS/VNS
function [R_INS,T_INS,R_VNS,T_VNS] = calRT_INS_VS (SINSposition_t_imu,SINSposition_last,Crb,Crb_last,RbbVision_k_integ,TbbVision_k_integ)
format long
% INS VNS量测信息
% RbbVision(:,:,k_integ)是当前帧(k_integ)的本体系到下一帧(k_integ+1)的本体系的旋转矩阵C_bcurrent_to_bnext
% TbbVision(:,:,k_integ)是VNS解算的k_integ+1本体坐标系下，当前帧(k_integ)的本体系到下一帧(k_integ+1)的本体系的平移矩阵 
% 
T_INS = SINSposition_last-SINSposition_t_imu ;  % 世界坐标系下，真实的k_integ帧位置到k_integ+1帧位置的平移
R_INS = (Crb * Crb_last');
R_VNS = RbbVision_k_integ;   % (VNS)的VNS解算b(k_integ)到VNS解算b(k_integ+1)的旋转矩阵，以cycle_TVNS为周期
T_VNS = (R_VNS * Crb_last)' * TbbVision_k_integ;    %% 将TccVision从视觉(k_integ+1)本体系转换到世界系

%% 3.17 修改之前备份
% T_INS = -(SINSposition_t_imu-SINSposition_last) ;  % 世界坐标系下，真实的k_integ帧位置到k_integ+1帧位置的平移
% R_INS = (Crb * Crb_last')';
% R_VNS = RbbVision_k_integ';   % (VNS)的VNS解算b(k_integ)到VNS解算b(k_integ+1)的旋转矩阵，以cycle_TVNS为周期
% T_VNS =- (R_VNS * Crb_last) * TbbVision_k_integ;    %% 将TccVision从视觉(k_integ+1)本体系转换到世界系
%% 原师姐版
% R_INS = Crb * Crb_last';
% T_INS = SINSposition_last - SINSposition_t_imu;
% R_VNS = RbbVision_k_integ;
% T_VNS = (R_VNS * Crb_last)' * TbbVision_k_integ;