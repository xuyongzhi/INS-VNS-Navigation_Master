% buaa xyz 2014 5 23
%% 计算量测量：
% % 量测量: dQbb（纠正的四元数误差定义）,dTbb（在前一时刻b(k-1)分解）

function [Z,Rbb_INS,Tbb_INS] = calZ_new_dQTb( position_new,position,Crb_new,Crb,Rbb_VNS,Tbb_VNS_in,isTbb_last )

format long


Qbb_VNS =  FCnbtoQ(Rbb_VNS);
Qbb_VNS_Inv = [Qbb_VNS(1);Qbb_VNS(2:4)];
if isTbb_last==0
    Tbb_last_VNS = Rbb_VNS'*Tbb_VNS_in ;  % 由视觉运动估计得到的Tb(k)b(k+1)是在k+1时刻表达的，在此转到 k (运动前一)时刻
else
    Tbb_last_VNS = Tbb_VNS_in ;
end
Rbb_INS = Crb_new*Crb' ;
Qbb_INS = FCnbtoQ(Rbb_INS);
Tbb_INS = Crb*(position_new-position) ;

dq = QuaternionMultiply(Qbb_VNS_Inv,Qbb_INS) ;
Zq = dq(2:4);
Zr = Tbb_INS-Tbb_last_VNS;

Z = [Zq;Zr];
