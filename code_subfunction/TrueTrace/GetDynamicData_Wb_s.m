% 

function [realTimeWb] = GetDynamicData_Wb_s(runTimeNum,fre) 

realTimeWb = zeros(3,runTimeNum);

% A = 0.1*pi/180;
% realTimeWb(3,1:fix(runTimeNum/2))=A;
% realTimeWb(3,fix(runTimeNum/2):runTimeNum)=-A;
% head=0;
% return

for k=1:runTimeNum
    realTimeWb(1,k) = 0 + 0.00 * sin(2*pi/(60*fre*2)*k) ;
    realTimeWb(2,k) = 0 ;
    T = 60*10;
    A = 0.7*pi/180 ;
    realTimeWb(3,k) = 0 + A * sin(2*pi/(T*fre)*k-pi/2+pi/20*0) ;
end

headA = -A*T/pi *180/pi ;

display(headA)

% 
% headT = 60*1 ;  % 一段圆弧的周期 sec
% headA = 30 ;        % 一段圆弧的角度
% w = headA/
% 
% for k=kStart:kStart+headT*fre-1
%     realTimeWb(3,k) = ;
% end

