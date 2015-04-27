%% DRbb标定陀螺常值漂移法A
function [gyroDriftEsmA_new,P_gyroDRbbKFA_new] = gyroDRbbAKF(R_INS,R_VNS,T,gyroDriftEsmA_old,P_gyroDRbbKFA_old,Q_gyroDRbbKFA,R_gyroDRbbKFA)

% 归一化R_VNS、R_INS


DR = R_VNS-R_INS ;
Z_gyroDRbbA = [ (DR(3,2)-DR(2,3));(DR(1,3)-DR(3,1));(DR(2,1)-DR(1,2))]/(2*T) ;

P_pre = P_gyroDRbbKFA_old + Q_gyroDRbbKFA ;

K = P_pre/(P_pre+R_gyroDRbbKFA) ;
gyroDriftEsmA_new = gyroDriftEsmA_old+K*(Z_gyroDRbbA-gyroDriftEsmA_old) ;

P_gyroDRbbKFA_new = (eye(3)-K)*P_pre*(eye(3)-K)'+K*R_gyroDRbbKFA*K' ;
