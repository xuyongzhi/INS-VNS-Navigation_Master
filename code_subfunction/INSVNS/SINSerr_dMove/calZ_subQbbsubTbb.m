% buaa xyz 2014 5 23
%% 计算量测量：
% % 量测量: dQbb（经转换因子转换）,dTbb（在前一时刻b(k-1)分解）

function [Z,Rbb_INS,Tbb_INS,Rbb_VNS,Tbb_VNS] = calZ_subQbbsubTbb( position_new,position,Crb_new,Crb,Rbb_VNS,Tbb_VNS_in,calZMethod,isTbb_last )

format long

Z_subQT_methodFlag = calZMethod.Z_subQT_methodFlag ;

Qbb_VNS =  FCnbtoQ(Rbb_VNS);
% 使 Tbb_VNS 在 前一时刻 分解
if isTbb_last==0
    Tbb_VNS = Rbb_VNS'*Tbb_VNS_in ;  % 由视觉运动估计得到的Tb(k)b(k+1)是在k+1时刻表达的，在此转到 k时刻
else
    Tbb_VNS = Tbb_VNS_in ;
end
Rbb_INS = Crb_new*Crb' ;
Qbb_INS = FCnbtoQ(Rbb_INS);
Tbb_INS = Crb*(position_new-position) ; % 前一时刻

% subQbb
% 求转换因子　　7.17改
QM_INS = [-Qbb_INS(2:4),Qbb_INS(1)*eye(3) + getCrossMatrix(Qbb_INS(2:4))];
QM_VNS = [-Qbb_VNS(2:4),Qbb_VNS(1)*eye(3) + getCrossMatrix(Qbb_VNS(2:4))];
% % % 求转换因子　　7.17前
% QM_INS = [-Qbb_INS(2:4),Qbb_INS(1)*eye(3) - getCrossMarix(Qbb_INS(2:4))];
% QM_VNS = [-Qbb_VNS(2:4),Qbb_VNS(1)*eye(3) - getCrossMarix(Qbb_VNS(2:4))];

Zq = QM_VNS*(Qbb_INS-Qbb_VNS);
Zr = Tbb_INS-Tbb_VNS;
% 法A
 Z0 = [Zq;Zr];

% 猜想拓展1 : 法B
Zq1 = QM_INS*Qbb_INS-QM_VNS*Qbb_VNS;
Z1 = [Zq1;Zr];
% Z = Z1;         %  比法A好
% 猜想拓展2 ： 法C
Zq2 = QM_INS*(Qbb_INS-Qbb_VNS);
Z2 = [Zq2;Zr];

switch Z_subQT_methodFlag
    case 0
        Z = Z0 ;
    case 1
        Z = Z1 ;
    case 2
        Z = Z2 ;        
end

