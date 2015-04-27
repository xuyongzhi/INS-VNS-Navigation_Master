% buaaxyz 2014.1.6

% 计算累积行程：输入一个二维矢量

function  routeLength = CalRouteLength( route )
route = makeRow(route,'行为时',3);
% 一行一个时刻
N = size(route,1);
routeLength = zeros(N,1);
for k=1:N
    routeLength(k) = CalRouteLength_OneDim(route(k,:));
end

function routeLength = CalRouteLength_OneDim( route )
% 输入route为一维矢量
routeLength = 0;
pos1=route(1);
for k=2:length(route)-1
    % 找到一个拐点
    if ( route(k)-route(k-1) )*( route(k+1)-route(k) )<0
        pos2 = route(k);
        routeLength = routeLength+abs(pos2-pos1);
        pos1 = pos2;        
    end
end
if routeLength==0
    routeLength = abs(route(length(route))-route(1));
end