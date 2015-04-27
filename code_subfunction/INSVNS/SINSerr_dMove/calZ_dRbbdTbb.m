% buaa xyz 2014 6 9
%% 计算量测量：
% % 量测量: dRbb,dTbb（在世界系下分解）

function [Z,Rbb_INS,Tbb_INS_w,Rbb_VNS,Tbb_VNS_w] = calZ_dRbbdTbb( position_new,position,Crb_new,Crb,Rbb_VNS,Tbb_VNS_in,isTbb_last )

format long
% 世界系的 Tbb
if isTbb_last==0
    Tbb_VNS_w = Crb_new' * Tbb_VNS_in ; % 由视觉运动估计得到的Tb(k)b(k+1)是在k+1时刻表达的，在此转到 w 下分解
else
    Tbb_VNS_w = Crb' * Tbb_VNS_in ;
end
Tbb_INS_w = position_new-position ;
Rbb_INS = Crb_new*Crb' ;

Z_Tbb = Tbb_INS_w - Tbb_VNS_w ;

opintions.headingScope = 180 ;
Z_Rbb = GetAttitude(Rbb_INS*Rbb_VNS','rad',opintions) ;

Z = [Z_Rbb;Z_Tbb];

