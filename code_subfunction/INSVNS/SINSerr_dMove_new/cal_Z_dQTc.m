%% º∆À„¡ø≤‚¡ø£∫ Qcc Tcc_last
% nuaaxyz 2014.7.28

function [Z,Rcc_INS,Tcc_INS] = cal_Z_dQTc( Rbc,Tcb_c,position_new,position,Crb_new,Crb,Rcc_VNS,Tcc_last_VNS )

Qcc_VNS =  FCnbtoQ(Rcc_VNS);
Qcc_VNS_Inv = [Qcc_VNS(1);Qcc_VNS(2:4)];

Rcc_INS = Rbc*Crb_new*Crb'*Rbc' ;
Qcc_INS = FCnbtoQ(Rcc_INS);
Tcc_INS = Rbc*Crb*(position_new-position)+( eye(3)-Rbc*Crb_new*Crb'*Rbc' )*Tcb_c ;

dq = QuaternionMultiply(Qcc_VNS_Inv,Qcc_INS) ;
Zq = dq(2:4);
Zr = Tcc_INS-Tcc_last_VNS ;

Z = [Zq;Zr];