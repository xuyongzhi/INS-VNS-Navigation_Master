%计算矩阵Ce_n

% 输入
%   lamd：经度（rad） [ -180 180 ]
%   L：纬度（rad）   [-90 90]
   
function [Ce_n]=FCen(lamd,L)
format long
Ce_n(1,1)=-sin(lamd); 
Ce_n(1,2)=cos(lamd);
Ce_n(1,3)=0;                             
Ce_n(2,1)=-sin(L)*cos(lamd);
Ce_n(2,2)=-sin(L)*sin(lamd); 
Ce_n(2,3)=cos(L);
Ce_n(3,1)=cos(L)*cos(lamd);                          
Ce_n(3,2)=cos(L)*sin(lamd);  
Ce_n(3,3)=sin(L);

% 或
% Cen = [1 0 0;0 sin(L) cos(L) ;0 -cos(L) sin(L)]*[-sin(lamd) cos(lamd) 0;-cos(lamd) -sin(lamd) 0;0 0 1];

 
