% buaa xyz 2014 5 23
%% 计算量测量：
% % 量测量: dQbb（经转换因子转换）,dTbb（在前一时刻b(k-1)分解）

function [Z,Rbb_INS,Tbb_INS,Rbb_VNS,Tbb_VNS] = calZ_dRbbsubTbb_b( position_new,position,Crb_new,Crb,Rbb_VNS,Tbb_VNS_in,isTbb_last )

format long

Rbb_INS = Crb_new*Crb' ;
opintions.headingScope = 180 ;
Z_Rbb = GetAttitude(Rbb_INS*Rbb_VNS','rad',opintions) ;

if isTbb_last == 0
    Tbb_VNS = Rbb_VNS'*Tbb_VNS_in ;  % 由视觉运动估计得到的Tb(k)b(k+1)是在k+1时刻表达的，在此转到 k时刻
else
    Tbb_VNS = Tbb_VNS_in ;
end
Tbb_INS = Crb*(position_new-position) ;

Z_Tbb = Tbb_INS-Tbb_VNS;

Z = [Z_Rbb;Z_Tbb];
