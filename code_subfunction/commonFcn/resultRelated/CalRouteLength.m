% buaaxyz 2014.1.6

% 计算累积行程：输入 位置
% routeLength_3dim：每点的空间行程
% routeLength_2dim：每点处的平面行程
%　routeLength_xyz：每点处的xyz各维、平面、三维行程 

function  [routeLength_xyz,routeLength_3dim,routeLength_2dim] = CalRouteLength( route )

N = size(route,2) ;
routeLength_3dim = zeros(1,N);% 空间总行程
routeLength_2dim = zeros(1,N); % 平面总行程

for k=1:N-1
    routeLength_2dim(k+1) = routeLength_2dim(k) + sqrt( (route(1,k+1)-route(1,k))^2 + (route(2,k+1)-route(2,k))^2 );
end

for k=1:N-1
    routeLength_3dim(k+1) = routeLength_3dim(k) + sqrt( (route(1,k+1)-route(1,k))^2 + (route(2,k+1)-route(2,k))^2 + (route(3,k+1)-route(3,k))^2 );
end

routeLength_xyz  = zeros(5,1);
for k=1:N-1
    routeLength_xyz(1) = routeLength_xyz(1) + sqrt( (route(1,k+1)-route(1,k))^2   );
    routeLength_xyz(2) = routeLength_xyz(2) + sqrt( (route(2,k+1)-route(2,k))^2   );
    routeLength_xyz(3) = routeLength_xyz(3) + sqrt( (route(3,k+1)-route(3,k))^2   );
end
routeLength_xyz(4) = routeLength_2dim(length(routeLength_2dim));
routeLength_xyz(5) = routeLength_3dim(length(routeLength_3dim));

