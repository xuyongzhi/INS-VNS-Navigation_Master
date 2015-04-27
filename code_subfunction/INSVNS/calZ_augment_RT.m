%% 计算量测量：augment_QT 方法
function Z = calZ_augment_RT( SINSposition_t_imu,SINSposition_last,Crb,Crb_last,RbbVision_k_integ,TbbVision_k_integ )
% 视觉输出为实际量测，将R转换为Q
format long
QVision = FCnbtoQ(RbbVision_k_integ) ;
T_VNS = (R_VNS * Crb_last)' * TbbVision_k_integ;
Z_vision = [QVision;T_VNS] ;
% 模型预测量测

