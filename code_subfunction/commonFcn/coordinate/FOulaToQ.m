%% 欧拉角->四元数
%   2.14.9.17
% angle： 俯仰、横滚、航向 rad
function Q = FOulaToQ(angle)
Cnb = FCbn(angle)' ;
Q = FCnbtoQ(Cnb) ;
